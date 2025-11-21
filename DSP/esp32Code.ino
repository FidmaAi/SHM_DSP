#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

// ==================== ADXL355 CONFIGURATION ====================
#define ADXL355_I2C_ADDRESS  0x1D
#define ADXL355_DEVID_AD     0x00
#define ADXL355_POWER_CTL    0x2D
#define ADXL355_RANGE        0x2C
#define ADXL355_FILTER       0x28
#define ADXL355_XDATA3       0x08

#define SAMPLE_RATE          100.0          // Hz
#define ACC_SCALE            0.0000038147
#define CALIBRATION_SAMPLES  500

// ==================== FILTER COEFFICIENTS ====================
struct BiquadCoeffs { float b0,b1,b2,a1,a2; };

// High-pass filter 2nd order
BiquadCoeffs hpCoeffs = {0.99965296, -1.99930592, 0.99965296, -1.99930580, 0.99930604};
// Low-pass 4th order = 2 cascaded biquad sections
BiquadCoeffs lpCoeffs1 = {0.39869412,0.79738824,0.39869412,0.97472922,0.26095218};
BiquadCoeffs lpCoeffs2 = {1.0,2.0,1.0,1.24401029,0.60930591};

// ==================== FILTER STATE VARIABLES ====================
struct BiquadState { float x1,x2,y1,y2; };
BiquadState hpStateX={0}, hpStateY={0}, hpStateZ={0};
BiquadState lpState1X={0}, lpState1Y={0}, lpState1Z={0};
BiquadState lpState2X={0}, lpState2Y={0}, lpState2Z={0};

// ==================== OFFSETS ====================
float offsetX=0, offsetY=0, offsetZ=0;
bool isCalibrated=false;

// ==================== ESP-NOW / NODE CONFIG ====================
#define WINDOW_SECONDS       5
#define WINDOW_SAMPLES       ((int)(SAMPLE_RATE*WINDOW_SECONDS))
#define ROLLING_MINUTES      3
#define WINDOWS_PER_MIN      (60 / WINDOW_SECONDS)
#define ROLLING_WINDOWS      (ROLLING_MINUTES*WINDOWS_PER_MIN)
#define RAW_BUFFER_SECONDS   120
#define RAW_SAMPLES          ((int)(SAMPLE_RATE*RAW_BUFFER_SECONDS))
#define Q_SCALE              1000.0f
#define MAX_PAYLOAD          240   // safe payload size for ESP-NOW (less than 250)

const float ENERGY_THRESHOLD = 0.0005f;
const float ALARM_SCORE_THRESHOLD = 0.02f;
uint8_t gatewayMac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; 
uint8_t NODE_ID = 1;

// Buffers
float windowX[WINDOW_SAMPLES], windowY[WINDOW_SAMPLES], windowZ[WINDOW_SAMPLES];
int windowPos = 0;
float scoreHistory[ROLLING_WINDOWS];
int scorePos=0, scoreCount=0;
int16_t *rawCircular=nullptr;
volatile int rawWritePos=0;
uint32_t waveformSeq=0;
uint16_t espnowSeq=0;

// ==================== ADXL355 I2C FUNCTIONS ====================
bool checkDeviceID() {
  Wire.beginTransmission(ADXL355_I2C_ADDRESS);
  Wire.write(ADXL355_DEVID_AD);
  Wire.endTransmission();
  Wire.requestFrom(ADXL355_I2C_ADDRESS,1);
  return Wire.available() && Wire.read()==0xAD;
}

void configureADXL355() {
  Wire.beginTransmission(ADXL355_I2C_ADDRESS); Wire.write(ADXL355_RANGE); Wire.write(0x01); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(ADXL355_I2C_ADDRESS); Wire.write(ADXL355_FILTER); Wire.write(0x04); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(ADXL355_I2C_ADDRESS); Wire.write(ADXL355_POWER_CTL); Wire.write(0x00); Wire.endTransmission(); delay(100);
}

void readRawAcceleration(float &x,float &y,float &z){
  uint8_t raw[9];
  Wire.beginTransmission(ADXL355_I2C_ADDRESS);
  Wire.write(ADXL355_XDATA3);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL355_I2C_ADDRESS,9);
  for(int i=0;i<9;i++) if(Wire.available()) raw[i]=Wire.read();
  int32_t xr=((int32_t)(raw[0]<<16 | raw[1]<<8 | raw[2]))>>4;
  int32_t yr=((int32_t)(raw[3]<<16 | raw[4]<<8 | raw[5]))>>4;
  int32_t zr=((int32_t)(raw[6]<<16 | raw[7]<<8 | raw[8]))>>4;
  if(xr&0x80000) xr|=0xFFF00000; if(yr&0x80000) yr|=0xFFF00000; if(zr&0x80000) zr|=0xFFF00000;
  x = xr*ACC_SCALE; y=yr*ACC_SCALE; z=zr*ACC_SCALE;
}

// ==================== OFFSET CALIBRATION ====================
void performOffsetCalibration(){
  float sumX=0,sumY=0,sumZ=0;
  Serial.println("Starting offset calibration. Keep sensor stationary...");
  for(int i=0;i<100;i++){ float x,y,z; readRawAcceleration(x,y,z); delay(8); }
  for(int i=0;i<CALIBRATION_SAMPLES;i++){ float x,y,z; readRawAcceleration(x,y,z); sumX+=x; sumY+=y; sumZ+=z; delay(8); }
  offsetX=sumX/CALIBRATION_SAMPLES; offsetY=sumY/CALIBRATION_SAMPLES; offsetZ=sumZ/CALIBRATION_SAMPLES-1.0f;
  isCalibrated=true;
  Serial.print("Calibration done. Offsets: "); Serial.print(offsetX,6); Serial.print(" "); Serial.print(offsetY,6); Serial.print(" "); Serial.println(offsetZ,6);
}

// ==================== FILTER FUNCTIONS ====================
float applyBiquad(float input,BiquadCoeffs &coeffs,BiquadState &state){
  float w = input - coeffs.a1*state.y1 - coeffs.a2*state.y2;
  float out = coeffs.b0*w + coeffs.b1*state.y1 + coeffs.b2*state.y2;
  state.y2 = state.y1; state.y1 = w;
  return out;
}

void processSample(float rawX,float rawY,float rawZ,float &outX,float &outY,float &outZ){
  float x=rawX-offsetX,y=rawY-offsetY,z=rawZ-offsetZ;
  x=applyBiquad(x,hpCoeffs,hpStateX); y=applyBiquad(y,hpCoeffs,hpStateY); z=applyBiquad(z,hpCoeffs,hpStateZ);
  x=applyBiquad(x,lpCoeffs1,lpState1X); y=applyBiquad(y,lpCoeffs1,lpState1Y); z=applyBiquad(z,lpCoeffs1,lpState1Z);
  outX=applyBiquad(x,lpCoeffs2,lpState2X); outY=applyBiquad(y,lpCoeffs2,lpState2Y); outZ=applyBiquad(z,lpCoeffs2,lpState2Z);
}

// ==================== PCA ANOMALY FUNCTIONS ====================
float computeWindowEnergy() {
  double sum=0;
  for(int i=0;i<WINDOW_SAMPLES;i++) sum += (double)windowX[i]*windowX[i] + (double)windowY[i]*windowY[i] + (double)windowZ[i]*windowZ[i];
  return (float)(sum / WINDOW_SAMPLES);
}

float computePcaAnomalyScore() {
  double meanX=0,meanY=0,meanZ=0;
  for(int i=0;i<WINDOW_SAMPLES;i++){ meanX += windowX[i]; meanY += windowY[i]; meanZ += windowZ[i]; }
  meanX /= WINDOW_SAMPLES; meanY /= WINDOW_SAMPLES; meanZ /= WINDOW_SAMPLES;
  double Cxx=0,Cxy=0,Cxz=0,Cyy=0,Cyz=0,Czz=0;
  for(int i=0;i<WINDOW_SAMPLES;i++){
    double ux = windowX[i]-meanX, uy = windowY[i]-meanY, uz = windowZ[i]-meanZ;
    Cxx += ux*ux; Cxy += ux*uy; Cxz += ux*uz;
    Cyy += uy*uy; Cyz += uy*uz; Czz += uz*uz;
  }
  Cxx/=WINDOW_SAMPLES; Cxy/=WINDOW_SAMPLES; Cxz/=WINDOW_SAMPLES;
  Cyy/=WINDOW_SAMPLES; Cyz/=WINDOW_SAMPLES; Czz/=WINDOW_SAMPLES;

  double vx=1.0, vy=0.0, vz=0.0;
  for(int it=0; it<16; ++it){
    double nx = Cxx*vx + Cxy*vy + Cxz*vz;
    double ny = Cxy*vx + Cyy*vy + Cyz*vz;
    double nz = Cxz*vx + Cyz*vy + Czz*vz;
    double norm = sqrt(nx*nx + ny*ny + nz*nz) + 1e-12;
    vx = nx / norm; vy = ny / norm; vz = nz / norm;
  }

  double mse = 0.0;
  for(int i=0;i<WINDOW_SAMPLES;i++){
    double ux = windowX[i]-meanX, uy = windowY[i]-meanY, uz = windowZ[i]-meanZ;
    double p = ux*vx + uy*vy + uz*vz;
    double rx = p*vx, ry = p*vy, rz = p*vz;
    double ex = ux - rx, ey = uy - ry, ez = uz - rz;
    mse += ex*ex + ey*ey + ez*ez;
  }
  mse /= WINDOW_SAMPLES;
  return (float)mse;
}

void pushScore(float s){ scoreHistory[scorePos] = s; scorePos = (scorePos+1)%ROLLING_WINDOWS; if(scoreCount<ROLLING_WINDOWS) scoreCount++; }
float averageScores(){ double sum=0; for(int i=0;i<scoreCount;i++) sum += scoreHistory[i]; return scoreCount? (float)(sum/scoreCount):0; }

// ==================== ESP-NOW FUNCTIONS ====================
struct PacketHeader{ uint8_t type; uint8_t node_id; uint16_t seq; uint32_t timestamp; } __attribute__((packed));
struct WaveChunkHeader{ uint16_t chunk_index; uint16_t total_chunks; uint32_t base_seq; } __attribute__((packed));

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  // Debug
  Serial.print("esp_now_send status: ");
  Serial.println(status==ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void initEspNow(){
  if(esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_peer_info_t peer; memset(&peer,0,sizeof(peer));
  memcpy(peer.peer_addr,gatewayMac,6);
  peer.channel=0; peer.encrypt=false;
  esp_err_t r = esp_now_add_peer(&peer);
  if(r==ESP_OK) Serial.println("Added gateway peer to ESP-NOW");
  else Serial.print("esp_now_add_peer -> "); Serial.println(r);
}

void sendAlarm(float avgScore){
  PacketHeader hdr;
  hdr.type = 0x01;
  hdr.node_id = NODE_ID;
  hdr.seq = espnowSeq++;
  hdr.timestamp = (uint32_t)(millis()/1000);

  uint8_t payload[sizeof(hdr) + sizeof(float)];
  memcpy(payload, &hdr, sizeof(hdr));
  memcpy(payload + sizeof(hdr), &avgScore, sizeof(float));

  esp_err_t res = esp_now_send(gatewayMac, payload, sizeof(payload));
  if(res == ESP_OK) Serial.println("Alarm packet queued");
  else Serial.printf("Alarm send failed: %d\n", res);
}

// ==================== sendWaveformNow (full implementation) ====================
void sendWaveformNow(){
  // snapshot current circular buffer pointers to avoid race wiht writer
  int writePos = rawWritePos; // next write index
  int totalFrames = RAW_SAMPLES; // frames count (each frame = 3 int16)
  waveformSeq++;

  // Header sizes
  const int hdrSize = sizeof(PacketHeader) + sizeof(WaveChunkHeader);
  int samplesPerFrameBytes = 3 * sizeof(int16_t);

  // compute max frames per packet
  int maxFramesPerPacket = (MAX_PAYLOAD - hdrSize) / samplesPerFrameBytes;
  if(maxFramesPerPacket <= 0) maxFramesPerPacket = 1;
  int totalChunks = (totalFrames + maxFramesPerPacket - 1) / maxFramesPerPacket;

  Serial.printf("Sending waveform seq=%u totalFrames=%d framesPerPacket=%d totalChunks=%d\n",
                waveformSeq, totalFrames, maxFramesPerPacket, totalChunks);

  for(int chunk=0; chunk<totalChunks; ++chunk){
    int startFrame = chunk * maxFramesPerPacket; 
    int framesThis = min(maxFramesPerPacket, totalFrames - startFrame);
    int packetSize = hdrSize + framesThis * samplesPerFrameBytes;
    uint8_t *packet = (uint8_t*)malloc(packetSize);
    if(!packet){
      Serial.println("OOM building waveform packet");
      return;
    }

    // Build headers
    PacketHeader hdr;
    hdr.type = 0x02;
    hdr.node_id = NODE_ID;
    hdr.seq = espnowSeq++;
    hdr.timestamp = (uint32_t)(millis()/1000);

    WaveChunkHeader wch;
    wch.chunk_index = (uint16_t)chunk;
    wch.total_chunks = (uint16_t)totalChunks;
    wch.base_seq = waveformSeq;

    memcpy(packet, &hdr, sizeof(hdr));
    memcpy(packet + sizeof(hdr), &wch, sizeof(wch));

    
    for(int f=0; f<framesThis; ++f){
      int frameIdx = (writePos + startFrame + f) % RAW_SAMPLES;
      int16_t sx = rawCircular[frameIdx*3 + 0];
      int16_t sy = rawCircular[frameIdx*3 + 1];
      int16_t sz = rawCircular[frameIdx*3 + 2];
      int offset = hdrSize + f * samplesPerFrameBytes;
      memcpy(packet + offset, &sx, sizeof(int16_t));
      memcpy(packet + offset + sizeof(int16_t), &sy, sizeof(int16_t));
      memcpy(packet + offset + 2*sizeof(int16_t), &sz, sizeof(int16_t));
    }

    // send via ESP-NOW
    esp_err_t r = esp_now_send(gatewayMac, packet, packetSize);
    if(r != ESP_OK) {
      Serial.printf("Wave chunk %d send failed: %d\n", chunk+1, r);
    } else {
      Serial.printf("Wave chunk %d/%d sent (%d frames)\n", chunk+1, totalChunks, framesThis);
    }

    free(packet);
    delay(30); // small gap to reduce collision risk — 
  } // end chunks

  Serial.println("Waveform send complete.");
}

// ==================== SETUP ====================
void setup(){
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("Initializing sensor node...");

  Wire.begin(); Wire.setClock(400000);
  if(!checkDeviceID()){
    Serial.println("ADXL355 NOT DETECTED! Halting...");
    while(1) delay(1000);
  } else Serial.println("ADXL355 connected OK");

  configureADXL355();
  Serial.println("Configuring ADXL355...");
  performOffsetCalibration();
  Serial.println("Offset calibration done.");

  WiFi.mode(WIFI_STA);
  initEspNow();
  Serial.println("ESP-NOW initialized.");

  // allocate raw circular buffer in PSRAM (we have 4 MB on the esp 32 s3 ) 
  size_t needBytes = (size_t)RAW_SAMPLES * 3 * sizeof(int16_t);
  rawCircular = (int16_t*)heap_caps_malloc(needBytes, MALLOC_CAP_SPIRAM);
  if(!rawCircular){
    rawCircular = (int16_t*)malloc(needBytes);
    if(rawCircular) Serial.println("Allocated raw buffer in SRAM (no PSRAM).");
    else { Serial.println("Failed to allocate raw buffer. Reduce RAW_BUFFER_SECONDS."); while(1) delay(1000); }
  } else Serial.println("Allocated raw buffer in PSRAM.");
}

// ==================== LOOP ====================
unsigned long lastSampleMs=0;
unsigned long lastHourlySend=0;
const unsigned long HOUR_MS=3600000UL; // production: 1 hour.

void loop(){
  unsigned long now = millis();
  unsigned long sampleInterval = (unsigned long)(1000.0 / SAMPLE_RATE);

  // sample at SAMPLE_RATE
  if(now - lastSampleMs >= sampleInterval){
    lastSampleMs = now;
    float rawX,rawY,rawZ,filtX,filtY,filtZ;
    readRawAcceleration(rawX,rawY,rawZ);
    processSample(rawX,rawY,rawZ,filtX,filtY,filtZ);

    // Fill window buffer
    windowX[windowPos] = filtX; windowY[windowPos] = filtY; windowZ[windowPos] = filtZ;
    windowPos++;

    // write into circular raw buffer (quantized int16)
    int idx = rawWritePos % RAW_SAMPLES;
    rawCircular[idx*3 + 0] = (int16_t)constrain(round(filtX * Q_SCALE), -32767, 32767);
    rawCircular[idx*3 + 1] = (int16_t)constrain(round(filtY * Q_SCALE), -32767, 32767);
    rawCircular[idx*3 + 2] = (int16_t)constrain(round(filtZ * Q_SCALE), -32767, 32767);
    rawWritePos = (rawWritePos + 1) % RAW_SAMPLES;

    if(windowPos >= WINDOW_SAMPLES){
      float E = computeWindowEnergy();
      float score = (E >= ENERGY_THRESHOLD) ? computePcaAnomalyScore() : 0.0f;
      pushScore(score);
      Serial.print("Window Energy: "); Serial.print(E,6);
      Serial.print(" | PCA Score: "); Serial.println(score,6);

      float avg = averageScores();
      if(avg >= ALARM_SCORE_THRESHOLD){
        Serial.print("!!! Alarm triggered !!! Average Score: "); Serial.println(avg,6);
        sendAlarm(avg);          // high-priority alarm
        sendWaveformNow();       // immediate dump of 2-min waveform (pre/post windows not implemented in POC)
      }
      windowPos = 0;
    }
  }

  // Hourly waveform send (or test interval)
  if(now - lastHourlySend >= HOUR_MS){
    lastHourlySend = now;
    Serial.println("Hourly raw transfer triggered.");
    sendWaveformNow();
  }

  delay(0);
}
