# SHM Preprocessing Pipeline for ESP32-S3 with ADXL355

## Overview
This project implements a real-time preprocessing pipeline for structural health monitoring (SHM) using the ADXL355 accelerometer on an ESP32-S3 microcontroller. The pipeline collects vibration data, applies filtering and preprocessing steps, and computes both statistical and spectral features for further analysis.

## Features
- **Data Acquisition**: Raw acceleration data sampled at 200 Hz from the ADXL355 sensor.
- **Windowing**: Data is processed in 2-second windows (512 samples) with 50% overlap.
- **Preprocessing**:
  - Offset calibration
  - High-pass filtering to remove slow drift
  - Low-pass filtering to remove high-frequency noise
  - Spike detection and mitigation using Z-score threshold
  - Missing value handling through linear interpolation
- **Feature Extraction**:
  - Statistical features: mean, standard deviation, min, max, median, quartiles, absolute sum of changes
  - Spectral features: dominant frequency, amplitude, spectral centroid, spread, skewness, kurtosis, and entropy (using FFT)
- **Real-time Processing**: Supports continuous acquisition and processing with overlapping windows.

## Hardware Requirements
- Adafruit's ESP32-S3 microcontroller
- ADXL355z three-axis accelerometer
- I2C connection between ESP32-S3 and ADXL355

## Software Requirements
- Arduino IDE or compatible development environment
- `arduinoFFT` library
- Wire library for I2C communication

## Usage
1. Connect the ADXL355 sensor to the ESP32-S3 using I2C.
2. Open the `DSP.ino` file in the Arduino IDE.
3. Ensure the required libraries (`Wire`, `arduinoFFT`) are installed.
4. Upload the code to the ESP32-S3.
5. Open the Serial Monitor to view real-time extracted features.

## Code Structure
- **I2C Functions**: Communicate with the ADXL355 and read raw acceleration data.
- **Preprocessing Functions**: Implement high-pass and low-pass filtering, spike mitigation, and missing value handling.
- **Feature Functions**: Compute statistical and spectral features for each axis.
- **Main Loop**: Reads sensor data, applies preprocessing, buffers data, and computes features per window.

## Author
Fidma Mohamed Abdelillah

## License
This project is open for academic and research purposes. For other uses, please contact the author.
