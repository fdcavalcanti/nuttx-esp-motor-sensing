# NuttX ESP32-C6 Motor Control and Sensing

This repository contains the source code for the article series "NuttX for Motor Control and Sensing" published on [Espressif's Developer Portal](https://developer.espressif.com/blog/).

- **Part 1**: [NuttX for Motor Control and Sensing](https://developer.espressif.com/blog/2025/05/nuttx-motor-control-and-sensing/) - Basic motor control with PWM and quadrature encoder
- **Part 2**: [NuttX Motor Control and Sensing: Data Transmission](https://developer.espressif.com/blog/2025/07/nuttx-motor-control-and-sensing-data-trans/) - TCP server implementation and IMU data streaming


## Overview

Demonstrates how to use multiple ESP32-C6 peripherals with NuttX RTOS:
- Motor Control PWM (MCPWM) for DC motor speed control
- Quadrature encoder for speed measurement
- ADC for potentiometer position reading
- IMU via I2C for vibration analysis (coming soon)
- Wi-Fi connectivity for data streaming (coming soon)

## Hardware Requirements

- ESP32C6-DevkitC
- CHR-GM25-370 6V DC Motor with integrated gearbox and quadrature encoder
- L298N H-Bridge motor driver module
- 10kΩ linear potentiometer
- GY521 IMU module (MPU6050)

![Test Bench Setup](imgs/test_bench.png "Motor Control Test Bench")

## GPIO Connections

- GPIO 20: MCPWM output to H-Bridge
- GPIO 3:  ADC input from potentiometer
- GPIO 10: Quadrature encoder channel A
- GPIO 11: Quadrature encoder channel B
- GPIO 5,6: I2C bus, SDA and CLK

## Building

This repository is designed to be used as an external application for NuttX. To build:

1. Clone this repository
2. Link it to your NuttX apps directory:
   ```bash
   ln -s <path-to-this-repo>/apps/ $NUTTX_PATH/apps/external
   ```
3. Configure NuttX:
   You can partially configure this project by enabling small parts of it, or, skip to item 3.4 to
   configure all peripherals at once.

   3.1. Motor control, accelerometer and quadrature encoder
   ```bash
   # Load base configuration
   ./tools/configure.sh esp32c6-devkitc:nsh

   # Merge with provided defconfig
   kconfig-merge -m .config ../apps/external/motor_sensing/config/defconfig
   make olddefconfig
   ```
   3.2. Add Wi-Fi support:
   ```
   kconfig-merge -m .config boards/risc-v/esp32c6/esp32c6-devkitc/configs/wifi/defconfig
   make olddefconfig
   ```

   3.3. IMU Support
   ```bash
   kconfig-merge -m .config boards/risc-v/esp32c6/esp32c6-devkitc/configs/mpu60x0/defconfig
   make olddefconfig
   ```

   3.4 Alternatively, use the complete defconfig with all the above and IMU configured:
   ```
   # Load base configuration
   ./tools/configure.sh esp32c6-devkitc:nsh

   # Merge with provided FULL defconfig
   kconfig-merge -m .config ../apps/external/imu/configs/imu_wifi/defconfig
   make olddefconfig
   ```


4. Build and flash:
   ```bash
   make
   make flash ESPTOOL_BINDIR=./ ESPTOOL_PORT=/dev/ttyUSB0
   ```

## Usage

After flashing, run the application from NSH:
```bash
nsh> msense
```

The application will display:
- Command speed (from potentiometer)
- Actual motor speed in RPM (from encoder)
- Sample time and encoder configuration

![Motor Control in Action](imgs/motor_spin.gif "Real-time motor control using potentiometer")

You can run this application in the background and start the `imu` program, which reads the accelerometer and transmits the data wirelessly, using sockets.

```
nsh> imu
MPU60x0 Accelerometer Test
Sample Rate: 20 ms (50 Hz)
TCP server starting on port 5000
Waiting for client connection...
```

On the client side, run the `run_client.sh` bash script provided in this repo. Make sure to set the proper IP address and port.

```
$ ./imu_client.sh 
Connecting to IMU server at 10.42.0.199:5000...
X:  1.168  Y: -0.077  Z:  0.145
```

Make sure to check the articles for more detailed instructions on this application.

## License

This project is licensed under the Apache License, Version 2.0 - see the [LICENSE](LICENSE) file for details.
