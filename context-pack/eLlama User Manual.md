# eLlama User Manual

## 1. Introduction

eLlama is a four-wheel-drive outdoor robot base, roughly the size of a large dog, that a person can lift into a car trunk. It ships as a base: chassis, four driven wheels, the Llama Motor Board, and a handheld controller. Add a battery of your own and drive it with no computer and no software.

It is also a mount for compute. A box on the front deck takes a single-board computer and whatever sensors the job needs — a camera, a lidar, an IMU. With that fitted, the same chassis becomes a teleoperated or autonomous platform.

### 1.1 Shipping Contents

- eLlama robot platform
- Onboard Llama Motor Board
- User breakout panel (battery voltage, unregulated; see 1.2.5)
- Single-Stick Controller
- Compute module (an empty bay for your own compute)

**Battery and charger not included.** You bring your own battery. Three options are supported: a 12 V 7 Ah SLA (sealed lead-acid) battery, an 18 V cordless-tool battery pack, or a 12 V LiFePO4 battery. Pick one, and charge it with its own charger off the robot. The comparison, the rules that apply to every option and current prices are in `05-battery-options.md` and `battery_bom.csv`. The specifications in this manual were measured on the 12 V SLA battery unless a row says otherwise.

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

A compute module is included with the robot. Additional payloads such as IMUs, lidar, cameras, GPS, and single-board computers can be easily, safely, and securely mounted in the compute module, and powered from the breakout panel. The panel carries battery voltage unregulated: about 12 V on SLA or LiFePO4, and roughly 16–21 V on a tool pack. Check that your equipment accepts that range, or add your own converter.

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
| Mass | 37 pounds with one 12 V SLA battery (32 pounds without a battery) |
| Maximum Payload | 40 pounds |
| All-terrain Payload | 20 pounds |
| Maximum Speed | 1 m/s on a 12 V SLA battery; a tool pack runs faster at the same command |
| Turn Rate (in-place spin), low-speed setting | ~60–80°/s |
| Turn Rate (in-place spin), high-speed setting | ~150–180°/s |
| Climb Grade | 30% |
| Sideslope | 15% |
| Operating Temperature, Min | 0 F |
| Operating Temperature, Max | 100 F |
| Operating Time, Max Power Consumption | About 1.5 h per 12 V 7 Ah SLA battery; depends on your battery |
| Operating Time, Standby | 24 h |
| Battery (not included) | Your choice of 12 V SLA, 18 V tool pack, or 12 V LiFePO4 |
| Battery Charger | Not included; use the charger that matches your battery |
| User Power | Battery voltage, unregulated, fused |
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

Start with the robot, compute, and Single-Stick Controller off. Turn on the robot, then turn on either the Single-Stick Controller or the compute module, not both (see 2.4).

### 2.3 Network Configuration

If the robot is being controlled with the Single-Stick Controller, the ESP-NOW protocol is used for low-latency execution of commands. If the robot is being teleoperated or autonomously controlled, the network layer will come from the compute module.

### 2.4 Single-Stick Controller

When running the stock firmware, there is no pairing. The ESP32 in the Single-Stick Controller broadcasts a 4-byte struct, and every ESP32 that is listening will try to execute the command.

**Use one command source at a time.** Commands do not work properly when two different ESP32s are transmitting them: the motor board executes whichever packet arrives last, so the two sources fight. To ensure proper operation:

- When driving with the computer bridge, unplug the Single-Stick Controller from its battery.
- When driving with the Single-Stick Controller, put the computer bridge in listen mode (send `listen`) or unplug it.

No firmware changes are needed to switch between them.

### 2.5 Battery and Charging

There is no dock and no charging circuit on the robot. Charge the battery off the robot with its own charger, then connect it.

1. Pick one of the three battery options in `05-battery-options.md` (12 V SLA, 18 V tool pack, or 12 V LiFePO4).
2. With the main power switch off, connect the battery. Check polarity first: red is positive.
3. Keep the voltage between 12 V and 21 V. Do not use a 36 V, 40 V, 56 V or 60 V pack.
4. Switch the robot on and drive it, following section 3.1.
5. When you are done, switch the robot off, disconnect or remove the battery, and charge it with its own charger. A 12 V SLA takes a standard SLA float charger, a tool pack its own tool charger, and a LiFePO4 battery a LiFePO4 charger. An SLA float charger is the wrong profile for LiFePO4.

If you use a tool pack, the robot runs faster at the same command than it does on SLA. Run it on blocks first and start with slow speeds.

## 3. Safety Considerations

It is the user's responsibility to operate the robot in a safe manner.

### 3.1 General Warnings

Conduct initial testing with the robot on blocks so that the wheels can spin freely. Start with slow speeds.

### 3.2 Lifting and Transport

The robot can be lifted by one person using the handle on the back of the robot. If desired, the robot's main power switch can be left on during transport, as this will provide some active braking: with the drivers powered and no drive command present, each motor's two terminals are both held near ground rather than left floating, so the motor's back-EMF is shorted through the driver and resists rotation. This only works while the power switch is on — with it off, the drivers are unpowered and the outputs float. If the robot is transported in a vehicle, it is advised to chock the tires.

The eLlama robot has a flat bottom and can be lifted on a single forklift fork and loaded into a truck bed.
