# eLlama User Manual

## 1. Introduction

eLlama is a four-wheel-drive outdoor robot platform, roughly the size of a large dog, that a person can lift into a car trunk. It ships as a complete working robot: chassis, four driven wheels, and a handheld controller. Drive it out of the box with no computer, no software, and no configuration beyond charging a battery.

It is also a mount for compute. A box on the front deck takes a single-board computer and whatever sensors the job needs — a camera, a lidar, an IMU — powered from the same battery that drives the wheels. With that fitted, the same chassis becomes a teleoperated or autonomous platform.

### 1.1 Shipping Contents

- eLlama robot platform
- Onboard Llama Motor Board
- User breakout panel with 12V power
- Single-Stick Controller
- Charging dock
- Compute module

**Batteries not included.** The robot uses a common, widely available battery, so users are expected to source their own locally or have a supplier ship one directly (some batteries can't be shipped by all carriers). All documentation and specifications in this manual assume the stock SLA (sealed lead-acid) chemistry, but any battery can be used with an appropriate voltage converter.

### 1.2 Hardware Overview

#### 1.2.1 System Architecture

eLlama is built around a modular design. The Llama Motor Board is a custom PCB that carries an ESP32 DevKitV1 module, soldered onto the board. The BTS7960 driver modules are separate boards, held on the motor driver mount. The bare PCB (traces and holes only) is manufactured by JLCPCB; the header pins and the ESP32 DevKitV1 are hand-soldered on afterward. The Llama Motor Board routes the DevKitV1's pins to the driver modules' eight PWM channels, so the wiring is identical from robot to robot and the same firmware runs on every unit without per-robot pin remapping.

The Llama Motor Board handles receiving motor commands and ramping motor current and speed to match the drive target. The Llama Motor Board can receive command signals from the Single-Stick Controller or from a serial connection.

#### 1.2.2 Exterior Features

eLlama is shown below and includes:

- Llama Motor Board power button
- Motion stop and lockout button (high-voltage switch)

#### 1.2.3 Status Lights

There are LED indicators on the switches, as well as a battery status indicator on the rear of the robot.

#### 1.2.4 Switches

There are two switches on the robot. One switches the Llama Motor Board and power for the motor drivers; if this is switched on, the robot can be driven with the Single-Stick Controller. A second switch powers the compute module.

#### 1.2.5 Payloads

A compute module is included with the robot. Additional payloads such as IMUs, lidar, cameras, GPS, and single-board computers can be easily, safely, and securely mounted in the compute module, and powered with the supplied 12V DC.

#### 1.2.6 Robot Equations

```
V = (vR + vL) / 2
Theta = (vR - vL) / effective track
```

### 1.3 System Specifications

| Spec | Measurement |
| --- | --- |
| Dimension, Length | 24 in |
| Dimension, Width | 22 in |
| Dimension, Height | 10 in |
| Track Width | 18.5 in |
| Wheelbase Length | 14 in |
| Ground Clearance | 4 in |
| Mass | 37 pounds |
| Maximum Payload | 40 pounds |
| All-terrain Payload | 20 pounds |
| Maximum Speed | 1 m/s |
| Turn Rate (in-place spin), low-speed setting | ~60–80°/s |
| Turn Rate (in-place spin), high-speed setting | ~150–180°/s |
| Climb Grade | 30% |
| Sideslope | 15% |
| Operating Temperature, Min | 0 F |
| Operating Temperature, Max | 100 F |
| Operating Time, Max Power Consumption | 1.5 h |
| Operating Time, Standby | 24 h |
| Battery (not included; stock chemistry) | 2x 12V 7Ah SLA |
| Battery Charger | Two-stage float charger |
| User Power | 12V 100A |
| Communication | WiFi, UART serial |
| Wheel Encoders | None |
| Internal Sensing | None |
| Environmental | Rain resistant |

Turn rate was measured via frame-by-frame video tracking of robot orientation across concrete, gravel, grass, and carpet. High-speed carpet readings were the noisiest of the set (lower visual contrast for tracking against carpet tiles), which widens that end of the range without necessarily reflecting a real difference in turn rate on that surface.

## 2. Getting Started

The robot should be placed on a box so that the wheels can spin freely off the ground for initial testing. Ensure that no cords or wires can get wrapped up in the wheels.

### 2.1 Onboard Llama Motor Board

The USB port on the onboard Llama Motor Board is accessible, and users are encouraged to run custom firmware scripts.

### 2.2 Powering Up

Start with the robot, compute, and Single-Stick Controller off. Turn on the robot, then turn on either the Single-Stick Controller or the compute module.

### 2.3 Network Configuration

If the robot is being controlled with the Single-Stick Controller, the ESP-NOW protocol is used for low-latency execution of commands. If the robot is being teleoperated or autonomously controlled, the network layer will come from the compute module.

### 2.4 Single-Stick Controller

When running the stock firmware, there is no pairing. The ESP32 in the Single-Stick Controller broadcasts a 12-byte struct, and every ESP32 that is listening will try to execute the command.

### 2.5 Battery Charging

If the charging dock is used, the robot should be driven into the dock, and when the pogo pins in the dock make contact with the robot, the light on the charger should change from green to red and the robot will start charging.

To charge without the dock, alligator clips can be connected directly to the battery terminals for charging with a standard 12V automotive float charger.

## 3. Safety Considerations

It is the user's responsibility to operate the robot in a safe manner.

### 3.1 General Warnings

Conduct initial testing with the robot on blocks so that the wheels can spin freely. Start with slow speeds.

### 3.2 Lifting and Transport

The robot can be lifted by one person using the handle on the back of the robot. If desired, the robot's main power switch can be left on during transport, as this will provide some active braking: with the drivers powered and no drive command present, each motor's two terminals are both held near ground rather than left floating, so the motor's back-EMF is shorted through the driver and resists rotation. This only works while the power switch is on — with it off, the drivers are unpowered and the outputs float. If the robot is transported in a vehicle, it is advised to chock the tires.

The eLlama robot has a flat bottom and can be lifted on a single forklift fork and loaded into a truck bed.
