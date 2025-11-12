#include <Wire.h>
#include <math.h>

// ==================== CONFIGURATION ====================
#define ADXL355_I2C_ADDRESS  0x1D
#define ADXL355_DEVID_AD     0x00
#define ADXL355_POWER_CTL    0x2D
#define ADXL355_RANGE        0x2C
#define ADXL355_FILTER       0x28
#define ADXL355_XDATA3       0x08

#define SAMPLE_RATE 128.0          // ODR: 128 Hz
#define ACC_SCALE 0.0000038147     // 2 / 2^19
#define CALIBRATION_SAMPLES 500    // Number of samples for offset calibration

// ==================== FILTER COEFFICIENTS ====================
struct BiquadCoeffs {
  float b0, b1, b2, a1, a2;
};

// High-pass filter 2nd order
BiquadCoeffs hpCoeffs = {0.99965296, -1.99930592, 0.99965296, -1.99930580, 0.99930604};

// Low-pass filter 4th order = 2 cascaded biquad sections
BiquadCoeffs lpCoeffs1 = {0.39869412, 0.79738824, 0.39869412, 0.97472922, 0.26095218};
BiquadCoeffs lpCoeffs2 = {1.0, 2.0, 1.0, 1.24401029, 0.60930591};

// ==================== FILTER STATE VARIABLES ====================
struct BiquadState { float x1, x2, y1, y2; };
BiquadState hpStateX = {0}, hpStateY = {0}, hpStateZ = {0};
BiquadState lpState1X = {0}, lpState1Y = {0}, lpState1Z = {0};
BiquadState lpState2X = {0}, lpState2Y = {0}, lpState2Z = {0};

// ==================== OFFSETS ====================
float offsetX = 0, offsetY = 0, offsetZ = 0;
bool isCalibrated = false;

// ==================== I2C FUNCTIONS ====================
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
  Wire.write(ADXL355_RANGE); Wire.write(0x01); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(ADXL355_I2C_ADDRESS);
  Wire.write(ADXL355_FILTER); Wire.write(0x04); Wire.endTransmission(); delay(10);
  Wire.beginTransmission(ADXL355_I2C_ADDRESS);
  Wire.write(ADXL355_POWER_CTL); Wire.write(0x00); Wire.endTransmission(); delay(100);
}

void readRawAcceleration(float &x, float &y, float &z) {
  uint8_t raw[9];
  Wire.beginTransmission(ADXL355_I2C_ADDRESS);
  Wire.write(ADXL355_XDATA3);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL355_I2C_ADDRESS, 9);
  
  for (int i = 0; i < 9; i++) if (Wire.available()) raw[i] = Wire.read();

  int32_t xr = ((int32_t)(raw[0]<<16 | raw[1]<<8 | raw[2])) >> 4;
  int32_t yr = ((int32_t)(raw[3]<<16 | raw[4]<<8 | raw[5])) >> 4;
  int32_t zr = ((int32_t)(raw[6]<<16 | raw[7]<<8 | raw[8])) >> 4;

  if (xr & 0x80000) xr |= 0xFFF00000;
  if (yr & 0x80000) yr |= 0xFFF00000;
  if (zr & 0x80000) zr |= 0xFFF00000;

  x = xr * ACC_SCALE;
  y = yr * ACC_SCALE;
  z = zr * ACC_SCALE;
}

// ==================== OFFSET CALIBRATION ====================
void performOffsetCalibration() {
  Serial.println("STARTING OFFSET CALIBRATION - Keep sensor STATIONARY");
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < 100; i++) { float x, y, z; readRawAcceleration(x,y,z); delay(8); }
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float x, y, z; readRawAcceleration(x,y,z);
    sumX += x; sumY += y; sumZ += z; delay(8);
  }

  offsetX = sumX / CALIBRATION_SAMPLES;
  offsetY = sumY / CALIBRATION_SAMPLES;
  offsetZ = sumZ / CALIBRATION_SAMPLES - 1.0; // remove gravity
  isCalibrated = true;

  Serial.println("CALIBRATION COMPLETE!");
  Serial.print("Offsets X Y Z: "); Serial.print(offsetX,6); Serial.print(" ");
  Serial.print(offsetY,6); Serial.print(" "); Serial.println(offsetZ,6);
}

// ==================== BIQUAD FILTER ====================
float applyBiquad(float input, BiquadCoeffs &coeffs, BiquadState &state) {
  float w = input - coeffs.a1*state.y1 - coeffs.a2*state.y2;
  float output = coeffs.b0*w + coeffs.b1*state.y1 + coeffs.b2*state.y2;
  state.y2 = state.y1; state.y1 = w;
  return output;
}

// ==================== DSP PIPELINE ====================
void processSample(float rawX, float rawY, float rawZ, float &outX, float &outY, float &outZ) {
  float x = rawX - offsetX, y = rawY - offsetY, z = rawZ - offsetZ;
  x = applyBiquad(x, hpCoeffs, hpStateX); y = applyBiquad(y, hpCoeffs, hpStateY); z = applyBiquad(z, hpCoeffs, hpStateZ);
  x = applyBiquad(x, lpCoeffs1, lpState1X); y = applyBiquad(y, lpCoeffs1, lpState1Y); z = applyBiquad(z, lpCoeffs1, lpState1Z);
  outX = applyBiquad(x, lpCoeffs2, lpState2X);
  outY = applyBiquad(y, lpCoeffs2, lpState2Y);
  outZ = applyBiquad(z, lpCoeffs2, lpState2Z);
}

// ==================== MAIN ====================
void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  Wire.begin(); Wire.setClock(400000);
  if(checkDeviceID()) Serial.println("ADXL355 connected OK");
  else { Serial.println("ADXL355 FAILED"); while(1) delay(1000); }
  configureADXL355();
  performOffsetCalibration();
  Serial.println("Starting continuous plotting of filtered accelerations (X Y Z)...");
}

void loop() {
  static unsigned long lastSample = 0;
  unsigned long sampleInterval = 1000.0 / SAMPLE_RATE;

  if(millis() - lastSample >= sampleInterval) {
    lastSample = millis();
    float rawX, rawY, rawZ;
    readRawAcceleration(rawX, rawY, rawZ);

    float filtX, filtY, filtZ;
    processSample(rawX, rawY, rawZ, filtX, filtY, filtZ);

    // Print filtered values only for Serial Plotter
    Serial.print(filtX, 6); Serial.print("\t");
    Serial.print(filtY, 6); Serial.print("\t");
    Serial.println(filtZ, 6);
  }
}
