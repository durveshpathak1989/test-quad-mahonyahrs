# Test Quad MahonyAHRS Library

## Explain It Simply

This module is another balance filter. It is kept as a separate reference version so you can compare it with the AHRS module used by the main flight sketch.

This standalone repository provides the `MahonyAHRS` estimator. In the main firmware the same implementation is also available through the consolidated AHRS library.

## Pin Map

Mahony does not use pins directly. It consumes IMU data:

| Signal | ESP32 pin | Notes |
| --- | ---: | --- |
| SPI SCK | GPIO 5 | MPU-9250/MPU-6500 clock |
| SPI MISO | GPIO 19 | MPU data to ESP32 |
| SPI MOSI | GPIO 18 | ESP32 data to MPU |
| MPU CS | GPIO 33 | Chip select passed to `MPU9250 imu(PIN_MPU_CS)` |
| MPU INT | GPIO 27 | Optional data-ready interrupt; current firmware does not require it |
| Motor FL | GPIO 25 | Front-left ESC signal |
| Motor FR | GPIO 15 | Front-right ESC signal |
| Motor RL | GPIO 14 | Rear-left ESC signal |
| Motor RR | GPIO 32 | Rear-right ESC signal |
| iBUS RX | GPIO 16 | FS-iA6B iBUS TX into ESP32 UART2 RX |
| iBUS TX | GPIO 4 | Spare UART TX; avoids GPIO17 GPS conflict |
| I2C SDA | GPIO 21 | BMP280 and VL53L4CX ToF bus |
| I2C SCL | GPIO 22 | BMP280 and VL53L4CX ToF bus |
| GPS RX | GPIO 13 | GPS TXD into ESP32 UART1 RX |
| GPS TX | GPIO 17 | Optional GPS RXD from ESP32 UART1 TX |


## Main INO Integration Example

```cpp
#include "MahonyAHRS.h"

MahonyAHRS mahony;

void setup() {
    mahony.setGains(1.0f, 0.005f);
}

void loop() {
    MPU_SensorData s;
    AttitudeEstimate att;
    if (imu.readScaled(s)) {
        mahony.update(s, 0.0025f, att);
    }
}
```


## Why These Data Types

The quaternion and correction integrals use `float` for speed on ESP32 and enough precision for flight angles. Gains are `float` so WiFi tuning can adjust them smoothly.
