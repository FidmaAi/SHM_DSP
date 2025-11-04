/*
 * SHM Preprocessing Pipeline for ESP32-S3 with ADXL355
 * Sampling Rate: 200 Hz
 * Window Size: 2s (512 samples) with 50% overlap
 * Features: Statistical + Spectral (FFT)
 * Author: Fidma Mohamed Abdelillah
 */

#include <Wire.h>
#include <arduinoFFT.h>
#include <math.h>
#include <string.h>

#define ADXL355_I2C_ADDRESS  0x1D
#define ADXL355_DEVID_AD     0x00
#define ADXL355_POWER_CTL    0x2D
#define ADXL355_XDATA3       0x08

#define SAMPLE_RATE 200.0
#define WINDOW_SIZE 512
#define OVERLAP 256  // 50% overlap

#define ACC_SCALE 0.0000039

// Filter constants
float hp_alpha = 0.9999; // high-pass alpha for slow drift removal
float lp_alpha = 0.2;    // low-pass alpha

// Buffers
float bufferX[WINDOW_SIZE];
float bufferY[WINDOW_SIZE];
float bufferZ[WINDOW_SIZE];
int bufferIndex = 0;

// Previous values for filtering
float hpfX_prev=0, hpfY_prev=0, hpfZ_prev=0;
float lpfX_prev=0, lpfY_prev=0, lpfZ_prev=0;
float rawX_prev=0, rawY_prev=0, rawZ_prev=0;

// Offset calibration
float offsetX=0, offsetY=0, offsetZ=0;

// FFT
arduinoFFT FFT = arduinoFFT();
double vReal[WINDOW_SIZE];
double vImag[WINDOW_SIZE];

// ------------------------- I2C Functions -------------------------
bool checkDeviceID() {
  Wire.beginTransmission(ADXL355_I2C_ADDRESS);
  Wire.write(ADXL355_DEVID_AD);
  Wire.endTransmission();
  Wire.requestFrom(ADXL355_I2C_ADDRESS, 1);
  if (Wire.available()) return Wire.read() == 0xAD;
  return false;
}

void configureADXL355() {
  Wire.beginTransmission(ADXL355_I2C_ADDRESS);
  Wire.write(ADXL355_POWER_CTL);
  Wire.write(0x00); // measurement mode
  Wire.endTransmission();
}

void readAcceleration(float &x, float &y, float &z) {
  uint8_t raw[9];
  Wire.beginTransmission(ADXL355_I2C_ADDRESS);
  Wire.write(ADXL355_XDATA3);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL355_I2C_ADDRESS, 9);
  for (int i=0; i<9; i++) if(Wire.available()) raw[i]=Wire.read();

  int32_t xr = ((uint32_t)raw[0]<<16 | raw[1]<<8 | raw[2]) >> 4;
  int32_t yr = ((uint32_t)raw[3]<<16 | raw[4]<<8 | raw[5]) >> 4;
  int32_t zr = ((uint32_t)raw[6]<<16 | raw[7]<<8 | raw[8]) >> 4;

  x = xr * ACC_SCALE;
  y = yr * ACC_SCALE;
  z = zr * ACC_SCALE;
}

// ------------------------- Preprocessing -------------------------

float highPass(float current, float &prevFiltered, float prevRaw){
  float filtered = hp_alpha*(prevFiltered + current - prevRaw);
  prevFiltered = filtered;
  return filtered;
}

float lowPass(float current, float &prevFiltered){
  float filtered = lp_alpha*current + (1-lp_alpha)*prevFiltered;
  prevFiltered = filtered;
  return filtered;
}

// Spike mitigation (Z-score threshold)
float spikeFilter(float value, float mean, float stdv){
  if(stdv == 0) return value;
  return (fabs(value - mean)/stdv > 5.0) ? mean : value;
}

// Missing value handling (simple linear interpolation)
float interpolate(float prevVal, float currentVal, bool valid){
  if(valid) return currentVal;
  return prevVal; 
}

// ------------------------- Feature Functions -------------------------
float computeMean(float *data, int n){
  float s=0; for(int i=0;i<n;i++) s+=data[i]; return s/n;
}

float computeStd(float *data,int n,float mean){
  float s=0; for(int i=0;i<n;i++) s+=pow(data[i]-mean,2); return sqrt(s/n);
}

float computeMin(float *data,int n){float m=data[0]; for(int i=1;i<n;i++) if(data[i]<m)m=data[i]; return m;}
float computeMax(float *data,int n){float m=data[0]; for(int i=1;i<n;i++) if(data[i]>m)m=data[i]; return m;}
float computeMedian(float *data,int n){
  float t[n]; memcpy(t,data,n*sizeof(float));
  for(int i=0;i<n-1;i++) for(int j=i+1;j<n;j++) if(t[i]>t[j]){float tmp=t[i]; t[i]=t[j]; t[j]=tmp;}
  return (n%2==0)?(t[n/2-1]+t[n/2])/2:t[n/2];
}
float computePercentile(float *data,int n,float p){
  float t[n]; memcpy(t,data,n*sizeof(float));
  for(int i=0;i<n-1;i++) for(int j=i+1;j<n;j++) if(t[i]>t[j]){float tmp=t[i]; t[i]=t[j]; t[j]=tmp;}
  int idx=int(p*(n-1)); if(idx>=n) idx=n-1; return t[idx];
}
float absoluteSumChanges(float *data,int n){float s=0;for(int i=1;i<n;i++) s+=fabs(data[i]-data[i-1]); return s;}

// ------------------------- Spectral Features -------------------------
void computeSpectralFeatures(float *data,int n,float &centroid,float &spread,float &skew,float &kurt,float &entropy,float &domFreq,float &domAmp){
  for(int i=0;i<n;i++){vReal[i]=data[i]; vImag[i]=0;}
  FFT.Windowing(vReal,n,FFT_WIN_TYP_HAMMING,FFT_FORWARD);
  FFT.Compute(vReal,vImag,n,FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal,vImag,n);

  float power[n/2]; float totalPower=0;
  for(int i=1;i<n/2;i++){power[i]=vReal[i]*vReal[i]; totalPower+=power[i];}

  domAmp = 0; int idxMax=1;
  for(int i=1;i<n/2;i++){if(power[i]>domAmp){domAmp=power[i]; idxMax=i;}}
  domFreq = idxMax*SAMPLE_RATE/n;

  float sumFreqPower=0; for(int i=1;i<n/2;i++) sumFreqPower += i*power[i];
  centroid = sumFreqPower/totalPower*(SAMPLE_RATE/n);

  float sumVar=0; for(int i=1;i<n/2;i++){float f=i*SAMPLE_RATE/n; sumVar += power[i]*(f-centroid)*(f-centroid);} spread=sqrt(sumVar/totalPower);

  float sumSkew=0,sumKurt=0;
  for(int i=1;i<n/2;i++){float f=i*SAMPLE_RATE/n; sumSkew += power[i]*pow(f-centroid,3); sumKurt += power[i]*pow(f-centroid,4);}
  skew = sumSkew/pow(spread,3)/totalPower;
  kurt = sumKurt/pow(spread,4)/totalPower;

  float H=0;
  for(int i=1;i<n/2;i++){float p = power[i]/totalPower; if(p>0) H-=p*log2(p);} entropy = H;
}

// ------------------------- Window Processing -------------------------
void processWindow(float *x,float *y,float *z,int n){
  float mean=computeMean(x,n);
  float stdv=computeStd(x,n,mean);
  float minV=computeMin(x,n);
  float maxV=computeMax(x,n);
  float median=computeMedian(x,n);
  float q25=computePercentile(x,n,0.25);
  float q75=computePercentile(x,n,0.75);
  float absSum=absoluteSumChanges(x,n);

  float centroid,spread,skew,kurt,entropy,domFreq,domAmp;
  computeSpectralFeatures(x,n,centroid,spread,skew,kurt,entropy,domFreq,domAmp);

  Serial.println("----- Features -----");
  Serial.printf("Mean: %.5f, Std: %.5f, Min: %.5f, Max: %.5f\n",mean,stdv,minV,maxV);
  Serial.printf("Median: %.5f, Q25: %.5f, Q75: %.5f, AbsSumChanges: %.5f\n",median,q25,q75,absSum);
  Serial.printf("DominantFreq: %.2f Hz, Amp: %.5f\n",domFreq,domAmp);
  Serial.printf("Centroid: %.2f, Spread: %.2f, Skew: %.2f, Kurt: %.2f, Entropy: %.4f\n",centroid,spread,skew,kurt,entropy);
  Serial.println("-------------------");
}

// ------------------------- Setup -------------------------
void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  Wire.begin();

  Serial.println("Initializing ADXL355...");
  if(!checkDeviceID()){Serial.println("ADXL355 not detected!"); while(1);}
  configureADXL355();
  Serial.println("ADXL355 configured and ready.");

  // Calibrate offset
  float sumX=0,sumY=0,sumZ=0;
  for(int i=0;i<200;i++){
    float x,y,z; readAcceleration(x,y,z);
    sumX+=x; sumY+=y; sumZ+=z;
    delay(5);
  }
  offsetX=sumX/200; offsetY=sumY/200; offsetZ
  = sumZ/200;
  Serial.printf("Offsets -> X: %.6f, Y: %.6f, Z: %.6f\n", offsetX, offsetY, offsetZ);
  Serial.println("Setup complete.");
}

// ------------------------- Main Loop -------------------------
void loop() {
  static unsigned long lastSample = 0;
  const unsigned long interval = 1000.0 / SAMPLE_RATE;

  if (millis() - lastSample >= interval) {
    lastSample = millis();

    float ax, ay, az;
    readAcceleration(ax, ay, az);

    // Offset correction
    ax -= offsetX; ay -= offsetY; az -= offsetZ;

    // High-pass (drift removal)
    ax = highPass(ax, hpfX_prev, rawX_prev);
    ay = highPass(ay, hpfY_prev, rawY_prev);
    az = highPass(az, hpfZ_prev, rawZ_prev);

    // Low-pass filtering
    ax = lowPass(ax, lpfX_prev);
    ay = lowPass(ay, lpfY_prev);
    az = lowPass(az, lpfZ_prev);

    // Spike detection (Z-score, using last window mean/std)
    float meanX = computeMean(bufferX, bufferIndex > 0 ? bufferIndex : 1);
    float stdX = computeStd(bufferX, bufferIndex > 0 ? bufferIndex : 1, meanX);
    ax = spikeFilter(ax, meanX, stdX);

    rawX_prev = ax; rawY_prev = ay; rawZ_prev = az;

    // Add to buffers
    bufferX[bufferIndex] = ax;
    bufferY[bufferIndex] = ay;
    bufferZ[bufferIndex] = az;
    bufferIndex++;

    // Process window
    if (bufferIndex >= WINDOW_SIZE) {
      processWindow(bufferX, bufferY, bufferZ, WINDOW_SIZE);

      // Shift buffer for overlap
      memmove(bufferX, bufferX + (WINDOW_SIZE - OVERLAP), OVERLAP * sizeof(float));
      memmove(bufferY, bufferY + (WINDOW_SIZE - OVERLAP), OVERLAP * sizeof(float));
      memmove(bufferZ, bufferZ + (WINDOW_SIZE - OVERLAP), OVERLAP * sizeof(float));
      bufferIndex = OVERLAP;
    }
  }
}

