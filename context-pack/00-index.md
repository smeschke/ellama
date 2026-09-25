# eLlama — Design Context Pack

A four-wheel-drive outdoor robot platform. Ships working with a handheld controller;
accepts a single-board computer, camera, lidar, and any sensors the owner needs as an
expansion path.

## Contents

| File | Covers |
|---|---|
| `01-overview.md` | What it is, who it's for, the product tiers |
| `02-physical-design.md` | Size, weight, geometry, and why each was chosen |
| `03-drivetrain-power.md` | Motors, wheels, battery, and the tradeoffs behind them |
| `04-control-architecture.md` | Design principles only — protocol is in flux |
| `05-docking.md` | The charging dock, and why the battery chemistry makes it trivial |
| `06-manufacturing-cost.md` | BOM, in-house fabrication, batch plan |
| `07-simulation.md` | The calibrated desktop simulator, sending commands from off-board compute, and the off-board-compute product vision (lawn mowing) |
| `eLlama User Manual.md` | The product manual: specs, getting started, safety |
| `bom.csv` | Current bill of materials |
| `Testing.csv` | Test log |

## Naming

The product is named **eLlama**.

Hardware names:

| Name | What it is | Firmware |
| --- | --- | --- |
| Llama Motor Board | Green PCB with the ESP32 DevKit (socketed), 2x4 driver headers and a power terminal. Only this board has been built so far. | `robot_motor_bts7960` |
| Llama Encoder Board | Future PCB for the two AS5600 encoders (not built yet; prototype is on a breadboard) | `robot_encoders_as5600` |
| Llama Relay Board | Future PCB for relays (not built yet) | none yet |
| Llama motor board mount | 3D-printed housing that attaches the Llama Motor Board to the robot | none |
| Motor driver mount | 3D-printed part that holds the four BTS7960 modules | none |
| BTS7960 driver modules | The four blue off-the-shelf boards | none |

Future boards follow the pattern "Llama <function> Board" and "Llama <function> board mount".
