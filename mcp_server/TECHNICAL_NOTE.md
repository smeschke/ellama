# Letting an AI agent drive a real robot: measured results and safeguards

eLlama is a hand-built four-wheel-drive outdoor robot (RS550 motors with 100:1 gearboxes,
10 in wheels, 12 V, ESP32 control). This note describes an [MCP server](README.md) that lets
an AI agent drive it with closed-loop tool calls, such as "go forward 10 inches" or "turn 90
degrees left", instead of open-loop "run the motors for N seconds" commands, and what was
measured while building it. Every number below comes from a logged run in
[`calibration/`](../calibration/) and is written up in full in
[`ROBOT_CHARACTERIZATION.md`](../ROBOT_CHARACTERIZATION.md).

## How it works

The agent calls tools; the server talks to the robot over USB serial through a bridge ESP32.

- `drive_distance` stops on **wheel-encoder** ticks. `turn_degrees` stops on **IMU gyro yaw**.
  `drive_arc` follows a curved path. `read_telemetry` and `look` (a phone camera on the robot)
  give the agent state to reason about between moves.
- Every tool that can move the robot requires `confirmed_safe=true`, which the agent may only
  set after asking the human operator about that specific move. The robot has no obstacle
  sensors, so the server cannot verify that the area is clear on its own.
- Each move stops and settles before starting, is capped in size and PWM, and sends `stop` when
  it ends, however it ends.
- The bridge firmware drops to listen-only if no command arrives for 500 ms, so a crashed agent
  does not leave the robot driving.

## Measurements (unit 1, painted wood floor, re-run 2026-09-29)

**Encoders overstate in-place turns.** Wheel scrub makes the wheels turn about 1.7x as much as
the chassis does: the encoder-differential estimate read about 150 degrees for each real 90
degrees at PWM 80 and above. On 2026-09-27 the same effect read about 2x (92 degrees estimated
against about 47 degrees observed by eye, with the IMU at about 49). The IMU tracked the real
turn, so `turn_degrees` uses it as the stop condition. After the change, a 90 degree request
landed at 91 to 102 degrees depending on speed, with overshoot growing from about 1 degree at
PWM 60 to about 12 degrees at PWM 125.

**Speed is repeatable and roughly linear above a deadband.** Forward and reverse agree within 3%:

| PWM | Forward (in/s) | Reverse (in/s) |
| --- | --- | --- |
| 50 | 3.92 | 3.82 |
| 60 | 5.92 | 5.87 |
| 80 | 9.28 | 9.23 |
| 100 | 11.91 | 11.59 |
| 125 | 14.32 | 14.33 |

Nothing moves at PWM 40. The curve reproduced the 2026-09-27 run within about 4%. A stop
command overshoots the target by about 1 in at PWM 50 and about 5 in at PWM 125 on a 12 in move.

**The right side is slightly stronger.** Over three forward and three reverse runs of 30 in at
PWM 80, the right wheel covered 4 to 6% more distance than the left in both directions (one warm-up
run showed 10%). The IMU showed about 3 degrees of left curve going forward and 3 to 4 degrees of the opposite
sign going in reverse, so a forward-and-back trip largely cancels. The same bias shows up in turns:
right turns run 36% faster than left at PWM 60, shrinking to 3.5% at PWM 125. Repeat runs agree
closely (the last two forward runs are within 0.2 degrees of IMU yaw), so the drift is systematic and
a controller can correct for it.

**The failsafe works.** Command-to-motion latency is 0.195 s, set by the motor board's PWM ramp
rather than the radio link. With the controlling process killed mid-drive (`kill -9`, no
graceful stop), the robot stopped in about half a second, matching the firmware's 500 ms timeout.

## What the safeguards are for

An agent given a physical robot tends toward the same mistakes: picking the PWM value that makes a
result easiest to see, applying it at full magnitude at once, treating a direction reversal as a sign
flip, and chaining moves as if each one started clean, the way they would in a simulator.
[`DRIVING_POLICY.md`](../DRIVING_POLICY.md) exists because of that, and the server enforces the
mechanical parts of it in code: ask before every move, start low and ramp, stop before reversing,
and check state between moves. The policy stresses that a real robot has no reset. A wrong command
can flip it or damage it, and nothing undoes that.

## Limits

- **No obstacle sensing.** Wheel encoders and an IMU tell the agent how far it went, not whether the
  path was clear. This is why every move needs a human confirmation. The `look()` camera is advisory
  only and does not replace it.
- **Per-unit numbers.** The tables above describe one unit, on one surface. Hand-built units differ
  in wheel alignment, and the calibration scripts (`characterize_speed.py`, `characterize_turns.py`)
  are meant to be re-run per robot. Some encoder and PWM sign constants in `robot_server.py` are
  still hard-coded to one unit.
- **Uncalibrated track width.** `track_width_in` is still a 20 in placeholder, so the encoder turn
  estimate is only a rough figure. This does not affect `turn_degrees`, which uses the IMU.
- **IMU drift.** Yaw is gyro-integrated, which is fine for turns lasting a few seconds. Drift over
  multi-minute sessions has not been characterized.
- **Distances are not independently ground-truthed on this run.** The 2026-09-29 distance runs match
  what the operator saw on the floor and match the earlier tape-measured 30 in run, but no tape
  measurement of lateral offset was logged for the drift test.
- **Small sample.** Each sweep is one pass over the PWM range, and the drift test is three runs per
  direction.

## Try it

The server, firmware, calibration scripts and full logs are in this repository, under the
[MIT License](../LICENSE). Setup and the tool reference are in [`README.md`](README.md); read
[`DRIVING_POLICY.md`](../DRIVING_POLICY.md) before sending any drive command.
