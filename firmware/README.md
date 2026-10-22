# Firmware ⚡

Arduino sketches for every ESP32 in the system — the stick controller, the robot's motor board, the sensor boards, and a bridge that plugs into a computer.

Communication is plain ESP-NOW with **no libraries beyond `esp_now.h`**: channel 1, no encryption, broadcast, no MAC pairing. Reading a sensor is a different matter — a driver library is fine there, and the VL53L0X needs one (its initialisation is a long undocumented register sequence). The AS5600 and MPU6050 are simple enough to talk to directly over I2C, so those sketches still use nothing but `Wire.h`.

Telemetry packets **identify themselves by name**, not by length. See [`libraries/LlamaSensors/LlamaSensors.h`](libraries/LlamaSensors/LlamaSensors.h): every sensor broadcasts the same 41-byte `SensorPacket` carrying a short name, a value count, the sender's `millis()`, and up to six `int32` values. The bridge prints `<NAME> <value>... <ms>` for whatever arrives, so **adding a sensor needs no change to the bridge or to any table anywhere** — the name travels in the packet.

This replaced an older scheme where a packet's type *was* its length (Drive = 4, Encoder = 12, Imu = 16). That worked only while every sensor had a distinct size, so two boards of the same kind were indistinguishable and each new sensor had to be padded to a unique length.

`DrivePacket` (4 bytes, signed PWM per side) is deliberately *not* part of this. The motor board and stick controller keep their own struct and need no reflash; the bridge tells a 4-byte drive command from 41-byte telemetry by length, which still works since 4 ≠ 41.

## Sketches

| Sketch | Flashes onto | Does |
|---|---|---|
| [`controller_stick/controller_stick.ino`](controller_stick/controller_stick.ino) | single stick controller | Reads the joystick, arcade-mixes it, streams `DrivePacket{left, right}` every 40 ms |
| [`robot_motor_bts7960/robot_motor_bts7960.ino`](robot_motor_bts7960/robot_motor_bts7960.ino) | Llama Motor Board | Ramps toward the commanded left/right PWM on four BTS7960 channels, with a 300 ms fail-safe stop |
| [`computer_bridge/computer_bridge.ino`](computer_bridge/computer_bridge.ino) | spare ESP32 on a bench/PC | Relays ESP-NOW ⇄ USB serial both ways. Prints any `SensorPacket` as `<NAME> <value>... <ms>`, overheard drive commands as `CMD <left> <right>`, and reads `<left> <right>` from serial to drive the robot — listen-only until it is sent a drive line (`listen` returns it to silent), so it can log while the stick drives |
| [`robot_encoders_as5600/robot_encoders_as5600.ino`](robot_encoders_as5600/robot_encoders_as5600.ino) | Llama Encoder Board | Reads two AS5600 wheel encoders (one per I2C bus), broadcasts `"ENC"` with cumulative counts ~50×/sec. Survives a damaged encoder: re-probes every 2 s and sends `SENSOR_VALUE_UNAVAILABLE` rather than a fake `0` |
| [`robot_imu_gy521/robot_imu_gy521.ino`](robot_imu_gy521/robot_imu_gy521.ino) | robot — IMU board | Hardened GY-521 sender: auto-detects 0x68/0x69, accepts clone `WHO_AM_I` values, enables the ~44 Hz DLPF, re-inits on disconnect, retries bias calibration if moved. Broadcasts `"IMU"` ~50×/sec |
| [`robot_imu_icm20945/robot_imu_icm20945.ino`](robot_imu_icm20945/robot_imu_icm20945.ino) | robot — IMU board (alternative) | GY-ICM20945 V2 (bank-switched registers, `WHO_AM_I` 0xEA). **Still on the old fixed-length `ImuPacket`** — the bridge keeps a legacy branch for it; convert it to `SensorPacket` to retire that branch |
| [`robot_tof_vl53l0x/robot_tof_vl53l0x.ino`](robot_tof_vl53l0x/robot_tof_vl53l0x.ino) | robot — ToF board (ESP32-C3) | Two VL53L0X range sensors on one I2C bus, fired one at a time, broadcast together as `"TOF_FRONT"` (change `SENSOR_NAME` to `"TOF_REAR"` for the second board) |
| [`bench_motor_bts7960_test/bench_motor_bts7960_test.ino`](bench_motor_bts7960_test/bench_motor_bts7960_test.ino) | bench ESP32 wired to the BTS7960 drivers, USB-tethered | Runs one motor channel (`fl`/`bl`/`fr`/`br`) at a time from the Serial Monitor at a low, clamped PWM, with slow ramping and auto-stop — no ESP-NOW |
| [`bench_encoders_as5600_test/bench_encoders_as5600_test.ino`](bench_encoders_as5600_test/bench_encoders_as5600_test.ino) | bench ESP32, USB-tethered | Dual-AS5600 check (per-encoder magnet alignment, angle/RPM), Serial-only. Probes each bus at both SDA/SCL orders and reports a missing encoder instead of reading it as a stopped wheel |
| [`bench_tof_vl53l0x_test/bench_tof_vl53l0x_test.ino`](bench_tof_vl53l0x_test/bench_tof_vl53l0x_test.ino) | bench ESP32-C3, USB-tethered | Dual-VL53L0X check with the full diagnostic set — distance, `RangeStatus`, return-signal and ambient-IR rates, measurement duration — Serial-only |
| [`bench_imu_mpu6050_test/bench_imu_mpu6050_test.ino`](bench_imu_mpu6050_test/bench_imu_mpu6050_test.ino) | bench ESP32, USB-tethered | GY-521 check (address auto-detect, gyro bias calibration, pitch/roll/yaw), Serial-only |
| [`bench_imu_icm20945_test/bench_imu_icm20945_test.ino`](bench_imu_icm20945_test/bench_imu_icm20945_test.ino) | bench ESP32, USB-tethered | GY-ICM20945 V2 check (gyro bias calibration, pitch/roll/yaw), Serial-only |

Every `bench_*` sketch except the motor one has a `visualize.py` beside it (OpenCV, reads that sketch's own USB output) for dials, bars and attitude displays while you wiggle the hardware by hand. For everything at once over the bridge, use [`../calibration/live_view.py`](../calibration/live_view.py).

## Setup

Standard ESP32 Arduino toolchain — install the ESP32 board package, pick your board, flash over USB. The controller flashes through the same USB-C port that powers it, no disassembly.

**Install the shared packet header** so `#include <LlamaSensors.h>` resolves:

```sh
ln -s "$PWD/libraries/LlamaSensors" ~/Arduino/libraries/LlamaSensors
```

A symlink rather than a copy, so the repo stays the single source of truth for the wire format.

**Sensor driver libraries**, via the Arduino Library Manager:

- `Adafruit_VL53L0X` — required by both VL53L0X sketches

**ESP32-C3 boards need `USB CDC On Boot` enabled.** The Arduino default is *Disabled*, which sends `Serial` to the UART pins and leaves you with a silent USB port. On the command line that is `-b esp32:esp32:esp32c3:CDCOnBoot=cdc`. Classic ESP32 boards use an external UART chip and need no such option, but their baud rate genuinely matters, where the C3's native USB ignores it.

**Boards are easy to mix up.** Several are the same chip with consecutive MACs, and they all enumerate as `/dev/ttyUSB*` in arbitrary order, so confirm which port is which before flashing:

```sh
esptool --port /dev/ttyUSB0 --no-stub read-mac
```

## Driving

The single stick controller broadcasts, so it drives any `robot_motor_bts7960.ino` listening on the same WiFi channel — no MAC address configuration needed. `computer_bridge.ino` broadcasts the same way once it is sent a drive line, so it works when driving from a computer instead of the stick; until then it only listens, so it can log telemetry while you drive with the stick.

**Only one ESP32 should transmit drive commands at a time.** The motor board follows whichever `DrivePacket` arrives last, so two senders fight and neither works properly. Unplug the stick controller from its battery when driving from the computer bridge, and put the bridge in listen mode (send `listen`) or unplug it when driving with the stick. The PC-side viewers never send a drive line, so they are safe to leave running either way.

## Notes worth knowing before debugging

- **A sensor that reports nothing is not the same as a sensor that is gone.** Nothing here ever transmits a fabricated `0`: an encoder that is not responding sends `SENSOR_VALUE_UNAVAILABLE` (`INT32_MIN`, because `-1` is a legitimate count), and a range sensor with nothing in front sends `SENSOR_VALUE_INVALID` (`-1`, impossible for a distance). A `0` reading means a wheel genuinely is not turning.
- **Interpretation belongs on the PC**, not in firmware — reflashing a board bolted to the robot is expensive, and filtering at the source destroys information for good. Senders put raw counts and raw millimetres on the wire; gear ratio, wheel size, median filtering and thresholds live in [`../calibration/`](../calibration/) and `config.json`. A threshold measured in one room is wrong in the next one.
- **SDA/SCL transposed against a wiring comment has twice made good hardware look dead here.** The encoder and IMU sketches now probe rather than trust their `#define`s, and say which pin order answered.
- **VL53L0X: `clearInterruptMask()` after every measurement is load-bearing.** The data-ready flag survives a measurement that timed out, and the driver then returns the *previous* reading instantly — reporting success after a 2 ms "measurement" and serving stale data forever after a single timeout. Keep the timing budget near the 33 ms default too: a much longer one exhausts the API's fixed-iteration wait loop and causes exactly that timeout.
- **Two ToF sensors that can see each other interfere.** They fire one at a time with 50 ms of dead air between, including across the cycle boundary.
- **The bridge hears every ESP-NOW broadcast on the channel**, including unrelated devices. Those are counted and summarised as `FOREIGN <n>`, and a foreign frame that happens to be 41 bytes long is rejected by `sensorPacketPlausible()` rather than printed as a reading with a garbage name.
- **Bundle sensors onto fewer boards rather than splitting them.** A packet costs roughly the same airtime whatever its size — framing and preamble dominate — so two sensors in one packet is about half the airtime of two packets.
