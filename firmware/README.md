# Firmware ⚡

Arduino sketches for every ESP32 in the system — one for the controller, one for the robot's motor board, plus a bench test sender. All of it is plain Arduino with no libraries beyond `esp_now.h`. Everything talks ESP-NOW on channel 1, no encryption, packet type distinguished purely by length.

## Sketches

| Sketch | Flashes onto | Does |
|---|---|---|
| [`controller_stick/controller_stick.ino`](controller_stick/controller_stick.ino) | single stick controller | Reads the joystick, arcade-mixes it, streams `DrivePacket{left, right}` every 40 ms |
| [`robot_motor_bts7960/robot_motor_bts7960.ino`](robot_motor_bts7960/robot_motor_bts7960.ino) | robot — motor board | Ramps toward the commanded left/right PWM on four BTS7960 channels, with a 300 ms fail-safe stop |
| [`computer_bridge/computer_bridge.ino`](computer_bridge/computer_bridge.ino) | spare ESP32 on a bench/PC | Reads `<left> <right>` over serial and re-sends it as `DrivePacket` over ESP-NOW; also listens for `EncoderPacket`/`ImuPacket` telemetry and relays them to serial as `ENC <left> <right> <ms>` / `IMU <ax> <ay> <az> <gx> <gy> <gz> <ms>` — listen-only until it is sent a drive line (`listen` returns it to silent), so it can log alongside the stick controller — lets a computer drive the robot and capture wheel-encoder and IMU data directly |
| [`robot_encoders_as5600/robot_encoders_as5600.ino`](robot_encoders_as5600/robot_encoders_as5600.ino) | robot — encoder board | Reads two AS5600 wheel encoders (one per I2C bus), broadcasts `EncoderPacket{left, right, ms}` (12 bytes) over ESP-NOW ~50x/sec |
| [`bench_motor_bts7960_test/bench_motor_bts7960_test.ino`](bench_motor_bts7960_test/bench_motor_bts7960_test.ino) | bench-test ESP32 wired to the BTS7960 drivers, USB-tethered | Runs one motor channel (`fl`/`bl`/`fr`/`br`) at a time from the Serial Monitor at a low, clamped PWM for a few seconds, with slow ramping and auto-stop — no ESP-NOW |
| [`bench_encoders_as5600_test/bench_encoders_as5600_test.ino`](bench_encoders_as5600_test/bench_encoders_as5600_test.ino) | bench-test ESP32, USB-tethered | Standalone dual-AS5600 sanity check (per-encoder magnet alignment, angle/RPM; left on GPIO 21/22, right on 18/19) with Serial-only output, no ESP-NOW |
| [`robot_imu_icm20945/robot_imu_icm20945.ino`](robot_imu_icm20945/robot_imu_icm20945.ino) | robot — IMU board | Reads a GY-ICM20945 V2 (bank-switched registers, WHO_AM_I 0xEA) over I2C, broadcasts the same 16-byte `ImuPacket{accel, gyro, ms}` over ESP-NOW ~50x/sec, gyro-bias-corrected at boot |
| [`robot_imu_mpu6050/robot_imu_mpu6050.ino`](robot_imu_mpu6050/robot_imu_mpu6050.ino) | robot — IMU board (temporary, bench-taped) | Reads a GY-521 (MPU6050) over I2C, broadcasts `ImuPacket{accel, gyro, ms}` over ESP-NOW ~50x/sec |
| [`bench_imu_mpu6050_test/bench_imu_mpu6050_test.ino`](bench_imu_mpu6050_test/bench_imu_mpu6050_test.ino) | bench-test ESP32, USB-tethered | Standalone GY-521 sanity check (gyro bias calibration, pitch/roll/yaw) with Serial-only output, no ESP-NOW |
| [`bench_imu_icm20945_test/bench_imu_icm20945_test.ino`](bench_imu_icm20945_test/bench_imu_icm20945_test.ino) | bench-test ESP32, USB-tethered | Standalone GY-ICM20945 V2 sanity check (gyro bias calibration, pitch/roll/yaw) with Serial-only output, no ESP-NOW — the GY-521's replacement candidate |

## Setup

Standard ESP32 Arduino toolchain — install the ESP32 board package, pick your board, flash over USB. The controller flashes through the same USB-C port that powers it, no disassembly.

The single stick controller broadcasts, so it drives any `robot_motor_bts7960.ino` listening on the same WiFi channel — no MAC address configuration needed. `computer_bridge.ino` broadcasts the same way once it is sent a drive line, so it works when driving from a computer instead of the stick; until then it only listens, so it can log telemetry while you drive with the stick.
