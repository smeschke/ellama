# 05 — Battery Options

eLlama ships without a battery. You bring one, the same way you bring your own compute.
There are three supported choices. Pick one, charge it off the robot, and plug it in.
The robot has no charging circuit and no dock.

Current prices for each choice are in `battery_bom.csv`.

## The three options

| | A: 12 V SLA | B: 18 V tool pack | C: 12 V LiFePO4 |
|---|---|---|---|
| Part | 12 V 7 Ah sealed lead-acid, F1 terminals | 18/20 V (5S) cordless-tool pack | 12.8 V LiFePO4 with a built-in BMS |
| Tested | Yes | Yes, with a Milwaukee M18 2 Ah | **Not yet** |
| Voltage at the robot | about 11.5–12.8 V | about 16–21 V | about 12–14.4 V |
| Speed at the same command | baseline (about 1 m/s) | noticeably faster | expected to match A |
| Weight | about 4.85 lb each | lighter than SLA | lighter than SLA |
| Connection | F1 terminals, as built | printed receiver with metal contacts | F1 or the pack's own terminals |
| Charging | any 12 V SLA float charger | your tool charger | a LiFePO4 charger |
| Good if | you want the cheapest, easiest start | you already own the tools | you want light weight at 12 V |

You can run one or two SLA batteries in parallel. A second battery adds capacity, not
voltage.

## Rules for every option

- **Voltage window: 12 V to 21 V, no higher.** The motor drivers and the 5 V converter
  are the limit. A 5S tool pack (18 V nominal, about 21 V fully charged) is inside it.
  Do not use 36 V, 40 V, 56 V or 60 V packs, and do not use a 6S (24 V) pack.
- **Use a battery with protection.** Tool packs and LiFePO4 packs have a BMS. A bare
  lithium cell does not, and eLlama does not supply one.
- **Fuse and switch stay on the robot.** The 40 A main fuse and the main power switch
  sit between the battery and everything else. Do not bypass them.
- **Check polarity before the first connection.** Red is positive. An adapter wired
  backwards will damage the motor board.
- **Charge off the robot.** Take the battery out, or disconnect it, and use its own
  charger.
- **Switch the robot off when you are done.** Leaving it on drains the battery, and
  lithium packs and SLA both dislike being run flat.

## Option A: 12 V SLA

This is the baseline the robot was designed and calibrated on. A 12 V 7 Ah SLA is
available at any hardware store, auto parts store or alarm supplier. It is the heaviest and
cheapest choice. Runtime is roughly 1.5 hours per battery at working duty, which is an
inference from logged voltage drop and not a measurement.

## Option B: 18 V tool pack

Tested with one Milwaukee M18 18 V 2 Ah pack. It worked well and was faster than two SLA
batteries. A printed receiver holds the pack, but the contacts that carry current must be
metal. Printed plastic must never be the conductor.

Things to know:

- **It is faster.** The firmware sends duty cycle, so motor voltage is the duty cycle times
  the pack voltage. At the same command a 5S pack runs the motors well above SLA voltage.
  Speed also changes with the pack's charge level.
- **Calibrate it.** The PWM numbers in `ROBOT_CHARACTERIZATION.md`, the 125 PWM ceiling in
  the MCP server, the limits in `DRIVING_POLICY.md` and the simulator speed model were all
  measured on SLA at about 12.5 V. Re-run `calibration/characterize_speed.py` on a tool
  pack before relying on them, and start lower.
- **Packs differ.** Protection circuits vary between brands and between models, and a
  trip under load cuts the drive power mid-run. Only the M18 2 Ah has been tested. Other
  18/20 V packs are expected to work but are not listed as supported until someone runs
  them.
- **Wiring.** The wiring diagram for this option is `images/wiring_diagram_fuel20v.png`.

## Option C: 12 V LiFePO4

**Not tested yet.** At 12.8 V nominal it should behave like SLA, so the existing
calibration should hold, but that is an expectation and not a result.

Things to know before buying:

- **Check the BMS rating.** Many small LiFePO4 batteries carry a 10 A or 20 A BMS. Four
  brushed motors can draw more than that at start-up or under load, and a BMS trip will cut
  power. The robot's real peak current has not been measured. Prefer a pack whose BMS is
  rated well above what you measure.
- **Use a LiFePO4 charger.** An SLA float charger is the wrong profile.
- **Check the terminals.** Not every pack has F1 spade terminals.
