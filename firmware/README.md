# Firmware ⚡

Arduino sketches for every ESP32 in the system — one for the controller, one for the robot's motor board, plus a bench test sender. All of it is plain Arduino with no libraries beyond `esp_now.h`. Everything talks ESP-NOW on channel 1, no encryption, packet type distinguished purely by length.

## Sketches

| Sketch | Flashes onto | Does |
|---|---|---|
| [`single_analog_stick/single_analog_stick.ino`](single_analog_stick/single_analog_stick.ino) | single stick controller | Reads the joystick, arcade-mixes it, streams `DrivePacket{left, right}` every 40 ms |
| [`motor_receiver/motor_receiver.ino`](motor_receiver/motor_receiver.ino) | robot — motor board | Ramps toward the commanded left/right PWM on four BTS7960 channels, with a 300 ms fail-safe stop |
| [`esp_bridge/esp_bridge.ino`](esp_bridge/esp_bridge.ino) | spare ESP32 on a bench/PC | Reads `<left> <right>` over serial and re-sends it as `DrivePacket` over ESP-NOW — lets a computer drive the robot directly |

## Setup

Standard ESP32 Arduino toolchain — install the ESP32 board package, pick your board, flash over USB. The controller flashes through the same USB-C port that powers it, no disassembly.

The single stick controller broadcasts, so it drives any `motor_receiver.ino` listening on the same WiFi channel — no MAC address configuration needed. `esp_bridge.ino` also broadcasts, so it works the same way when driving from a computer instead of the stick.

Wiring diagrams, BOMs, and protocol documents are in [`build_documents/`](../build_documents/).
