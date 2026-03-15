# UGV Base ROS QMI8658 Adapted

Adapted ESP32 lower-controller firmware for the Waveshare General Driver for Robots.

This repository is based on the original lower-controller firmware structure and was adapted to fix IMU-related startup problems and make the board work with QMI8658 accelerometer + gyroscope on the onboard I2C bus

## Current status

Working:
- firmware boots normally
- Wi-Fi control works
- motor control works
- INA219 works
- QMI8658 accelerometer works
- QMI8658 gyroscope works
- serial telemetry works

Partially working / not finished:
- magnetometer is not integrated in this build
- yaw / compass is not fully implemented
- IMU support is focused on stable accel + gyro operation

## Main changes

Compared to the original setup:
- fixed I2C bus selection for onboard devices
- adapted IMU initialization to use QMI8658
- removed the startup freeze caused by the previous IMU path
- kept the firmware stable for lower-controller tasks such as Wi-Fi and motor control

## I2C configuration used in this build

Confirmed onboard I2C bus:
- SDA = GPIO 32
- SCL = GPIO 33

Detected devices on the board during testing:
- `0x3C`
- `0x42`
- `0x6B`
- `0x7E`

## IMU notes

The original IMU path caused startup issues on this board during testing

This adapted build uses QMI8658 for:
- acceleration
- gyroscope data
- basic roll / pitch estimation

Magnetometer data is not completed in this version, so:
- `mx / my / mz` are not the focus here
- yaw is not considered final

## Dependencies

This project uses:
- ArduinoJson
- Adafruit SSD1306
- INA219_WE
- ESP32Encoder
- PID_v2
- SimpleKalmanFilter
- SCServo
- QMI8658 library

Depending on your setup, some libraries may need to be installed manually in Arduino IDE

## Tested result

Tested on the Waveshare General Driver for Robots ESP32 lower controller:
- board boots
- Wi-Fi access point appears
- serial telemetry is active
- motors can be controlled through Wi-Fi
- accel / gyro data changes correctly

## Why this repository exists

This repository was created because the IMU issue on this board can break startup, and there was no simple adapted public fix available for this exact case.

The goal of this project is practical:
- make the lower controller boot
- keep Wi-Fi and motor control working
- provide usable onboard accel + gyro support

## Notes

This is an adapted practical build, not a full rewrite of the original firmware

If someone wants to improve:
- magnetometer support
- full yaw / heading
- cleaner IMU abstraction
- better calibration flow

forks and improvements are welcome
