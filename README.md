
#  Real-Time DSP with ADXL355Z + ESP32-S3

This repo implements a **real-time digital signal processing (DSP) pipeline** for structural vibration monitoring using the **ADXL355Z triaxial accelerometer** and the **Adafruit ESP32-S3** microcontroller.

It demonstrates how to:
- Acquire clean acceleration data via I²C,
- Perform **offset nulling** and **drift removal**,
- Apply **high-pass and low-pass digital filters** using **biquad IIR sections**,
- Stream filtered acceleration data in real time.

---

##  Hardware Overview

| Component | Description |
|------------|-------------|
| **Sensor** | ADXL355Z (Triaxial Digital MEMS Accelerometer) |
| **MCU** | Adafruit ESP32-S3 (Arduino-compatible) |
| **Interface** | I²C (400 kHz) |
| **Supply Voltage** | 3.3 V |
| **Full-scale Range** | ±2 g (configurable to ±4 g or ±8 g) |
| **ADC** | 20-bit Σ-Δ ADC |
| **ODR (Output Data Rate)** | 128 Hz (set via internal filter register) |

---

##  Wiring Diagram (I²C)

##  Wiring Diagram (I²C)

| ADXL355Z Pin | ESP32-S3 Pin | Description |
|---------------|--------------|--------------|
| VIN (P1.1), (P1.3) | 3.3 V | Power supply |
| GND (P1.5) | GND | Ground |
| SCL (P2.2) | SCL | I²C Clock |
| SDA (P2.6) | SDA | I²C Data |

>  If `SDO` is pulled high, the address becomes `0x53`.

---

##  Signal Processing Pipeline

###  Data Acquisition
- **ODR (Output Data Rate)**: 128 Hz
- **Resolution**: 20-bit
- **Scale factor**:  
```

ACC_SCALE = 2 g / 2¹⁹ = 0.0000038147 g/LSB

````
- The **ADXL355’s internal digital filter** (sinc/decimation) performs **analog anti-aliasing**, so no external analog filter is required.

---

###  Offset Nulling (DC Bias Removal)
Removes the **static offset** due to sensor bias and gravity.

At startup:
- The sensor is assumed **stationary**.
- The mean acceleration over 500 samples is computed:
```cpp
offsetZ = mean(Z) - 1.0; // remove gravity
offsetX = mean(X);
offsetY = mean(Y);
````

* The offsets are subtracted from subsequent readings.

---
The accelerometer signals are filtered in real-time using **hard-coded biquad filters**.  
These coefficients were generated in Python using `scipy.signal.butter` and converted to Arduino-compatible form.

the python code :
```
import numpy as np
from scipy import signal

# Sampling frequency
fs = 128.0  # Hz

# ============================================
# HIGH-PASS FILTER (2nd order, fc=0.01 Hz)
# ============================================
fc_hp = 0.01  # Cutoff frequency in Hz

# Generate Second-Order Sections (SOS)
sos_hp = signal.butter(
    N=2,                    # Filter order
    Wn=fc_hp,               # Cutoff frequency
    btype='high',           # High-pass
    analog=False,           # Digital filter
    output='sos',           # Second-Order Sections format
    fs=fs                   # Sampling frequency
)

print("HIGH-PASS FILTER COEFFICIENTS:")
print("SOS format (each row is one biquad):")
print(sos_hp)
print("\nFor Arduino code:")
for i, section in enumerate(sos_hp):
    b0, b1, b2, a0, a1, a2 = section
    print(f"Biquad {i+1}:")
    print(f"  b0 = {b0:.10f}")
    print(f"  b1 = {b1:.10f}")
    print(f"  b2 = {b2:.10f}")
    print(f"  a1 = {a1/a0:.10f}")  # Normalized by a0
    print(f"  a2 = {a2/a0:.10f}")  # Normalized by a0
    print()

# ============================================
# LOW-PASS FILTER (4th order, fc=50 Hz)
# ============================================
fc_lp = 50.0  # Cutoff frequency in Hz

sos_lp = signal.butter(
    N=4,                    # Filter order (will create 2 biquads)
    Wn=fc_lp,
    btype='low',            # Low-pass
    analog=False,
    output='sos',
    fs=fs
)

print("LOW-PASS FILTER COEFFICIENTS:")
print("SOS format (each row is one biquad):")
print(sos_lp)
print("\nFor Arduino code:")
for i, section in enumerate(sos_lp):
    b0, b1, b2, a0, a1, a2 = section
    print(f"Biquad {i+1}:")
    print(f"  b0 = {b0:.10f}")
    print(f"  b1 = {b1:.10f}")
    print(f"  b2 = {b2:.10f}")
    print(f"  a1 = {a1/a0:.10f}")
    print(f"  a2 = {a2/a0:.10f}")
    print()

```


###  High-Pass Filtering (Drift Removal)

Removes very low-frequency drift due to temperature or long-term bias.

| Parameter        | Value                 |
| ---------------- | --------------------- |
| Filter type      | 2nd-order Butterworth |
| Cutoff frequency | 0.01 Hz               |
| Implementation   | Single biquad section |

**Coefficients (normalized):**

```
b0 = 0.99965296,  b1 = -1.99930592,  b2 = 0.99965296
a1 = -1.99930580, a2 = 0.99930604
```

---

###  Low-Pass Filtering (Noise Reduction)

Removes high-frequency noise above the structural vibration range (typically < 50 Hz).

| Parameter        | Value                        |
| ---------------- | ---------------------------- |
| Filter type      | 4th-order Butterworth        |
| Cutoff frequency | 50 Hz                        |
| Implementation   | Two cascaded biquad sections |

**Coefficients:**

**Biquad 1**

```
b0 = 0.39869412,  b1 = 0.79738824,  b2 = 0.39869412
a1 = 0.97472922,  a2 = 0.26095218
```

**Biquad 2**

```
b0 = 1.0,  b1 = 2.0,  b2 = 1.0
a1 = 1.24401029,  a2 = 0.60930591
```


---



##  Real-Time DSP Pipeline Summary

| Step | Description                                | Implementation                     |
| ---- | ------------------------------------------ | ---------------------------------- |
| 1️⃣  | Acquire acceleration from ADXL355 (20-bit) | `readRawAcceleration()`            |
| 2️⃣  | Offset nulling at startup                  | `performOffsetCalibration()`       |
| 3️⃣  | High-pass filtering (remove drift/DC)      | `applyBiquad()` using `hpCoeffs`   |
| 4️⃣  | Low-pass filtering (noise reduction)       | Two cascaded `applyBiquad()` calls |
| 5️⃣  | Stream filtered data                       | Serial Plotter output (X, Y, Z)    |

---

##  Arduino Code Summary

### Main Features

* Configures ADXL355 via I²C for 128 Hz sampling and ±2 g range.
* Performs automatic offset calibration on startup.
* Applies cascaded biquad IIR filters for drift and noise suppression.
* Streams filtered acceleration data for plotting or anomaly detection.

### Key Functions

| Function                     | Role                                                   |
| ---------------------------- | ------------------------------------------------------ |
| `checkDeviceID()`            | Verify ADXL355 connection                              |
| `configureADXL355()`         | Configure range, filters, and measurement mode         |
| `performOffsetCalibration()` | Compute and store offsets (DC nulling)                 |
| `applyBiquad()`              | Process one sample through a biquad section            |
| `processSample()`            | Complete filtering chain (HP + LP)                     |
| `loop()`                     | Read, filter, and print acceleration data continuously |

---

##  Why Biquad Filters?

A **biquad** (bi-quadratic) filter implements a **2nd-order IIR** section in a numerically stable way.
Butterworth filters are *realized* as cascades of biquad sections.


---

##  Internal Filtering of ADXL355 

The ADXL355 uses a **digital Σ-Δ converter** with internal **sinc filtering** and **decimation**.
This internal stage:

* Acts as an **anti-aliasing filter**
* Sets the **effective ODR**
* Ensures the output signal is already clean before additional DSP

This is why the **external filters** here are *post-processing* filters (for drift and smoothing), not anti-aliasing filters.

---

##  Example Output (Serial Monitor)

```
ADXL355 connected OK
STARTING OFFSET CALIBRATION - Keep sensor STATIONARY
CALIBRATION COMPLETE!
Offsets X Y Z: 0.000012 -0.000015 -1.001013
Starting continuous plotting of filtered accelerations (X Y Z)...
-0.000010	0.000016	-0.001212
0.000004	0.000002	-0.001185
-0.000011	0.000015	-0.001195
...
```

---

##  Usage

1. Connect ADXL355Z to ESP32-S3 as per the wiring table.
2. Open the code in **Arduino IDE**.
3. Select the **Adafruit ESP32-S3 board**.
4. Upload the sketch.
5. Open **Serial Plotter** at 115200 baud to visualize live accelerations.
---

##  To add

1. missing data handling 
2. spike mitigation     
---

##  References

* **Analog Devices ADXL355 Datasheet**
  [https://www.analog.com/media/en/technical-documentation/user-guides/EVAL-ADXL354-355-UG-1030.pdf]([https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_355.pdf](https://www.analog.com/media/en/technical-documentation/user-guides/EVAL-ADXL354-355-UG-1030.pdf))
* **Butterworth Filter Design** — SciPy `signal.butter`
* **Biquad IIR Filters on Embedded Systems** — ARM CMSIS DSP Guide

---






