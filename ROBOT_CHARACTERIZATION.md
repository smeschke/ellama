# Robot characterization checklist

Goal: give Claude the mechanical numbers it can't measure on its own, so it can
drive the robot safely and build a path follower. Most data comes from the
encoders and IMU (recorded with `calibration/live_view.py --csv`); the tape
measure and video cover what those can't see: slip, real stopping distance,
and drift.

Assumptions: battery is kept above 80%, so battery level is not a variable.
The compute box (Pi, camera, lidar) is **not** on the robot. Repeat the
relevant tests if that changes the robot's mass.

## Ground rules for every test

- Robot on clear floor with at least 10 ft of free run, someone within reach.
- Any test that sends motor commands: tell Claude the robot is safe to move
  before it runs. Claude will cap PWM and always send `stop` on exit.
- Name each recording so it's obvious what it is, e.g. `--csv sweep_fwd.csv`.
- Note the floor surface and whether anything is on the robot.

## Video tips (for the tests that need it)

- Camera on a tripod or stack of books, side-on to the path, level with the floor.
- Tape measure laid on the floor along the path, in frame.
- 60 fps if the phone allows it.
- Say the test name and PWM value out loud at the start of each take.

---

## 0. Measurements by hand (5 minutes)

Write the values in the table and give them to Claude.

| Item | Value |
|---|---|
| Total mass of robot, as run (kg or lb) | |
| Wheel diameter (in) | |
| Track width, centre of left tyre to centre of right tyre (in) | |
| Wheelbase / caster position (in) | |
| Overall footprint, length x width x height (in) | |
| Roughly how high the centre of mass is (in) | |
| Wheel type and floor contact (rubber, hard plastic, etc.) | |
| Caster or skid type at the other end | |
| Where the bumpers, kill switch and battery sit | |

Also note anything you plan to add later (payload, mounts) and its mass.

## 1. PWM to steady speed (most important)

**What it tells us:** the deadband, the shape of the curve, and any left/right
motor mismatch.

**How:**
1. Put the robot where it has room, wheels free to run on the floor.
2. Claude runs a sweep: each wheel in turn, then both together, stepping PWM
   from low up to your max cap, forward and reverse, holding each step 2-3 s
   with a short stop between.
3. Encoders record steady-state speed at each step.

**You provide:** the max PWM cap you're comfortable with, and confirmation the
robot is safe to move.

**Data out:** `sweep_*.csv` from `live_view.py --csv`, plus Claude's PWM log.

> Status (2026-09-27, "unit1", robot on the ground, 4ft clear in front / 2ft behind):
> `calibration/characterize_speed.py` sweeps PWM via the MCP server's `drive_distance`
> tool, forward then reverse each step, and logs actual distance/time to a CSV — see
> `calibration/speed_characterization_unit1_20260927_201926.csv` (an earlier partial run,
> `..._201610.csv`, caught a real forward/reverse deadband asymmetry at pwm=40 — forward
> didn't move at all in 9.3s, reverse moved fine at 1.49in/s — before its own drift guard
> paused it; see that CSV and the git history of this file for the raw finding).
> Deadband is between pwm 40 and 50. Above it, forward and reverse speeds match closely
> (no meaningful asymmetry, unlike turning in place — see section 4): 50->4.11in/s,
> 60->6.13, 80->9.36, 100->12.03, 125->14.46. Roughly linear, ~0.14 in/s per PWM count
> above the deadband. Re-run the script per robot; this is unit1's curve, not a
> universal one.

## 2. Acceleration and stopping

**What it tells us:** how long speed takes to settle, and how far it coasts.
This sets the safe top speed for a given obstacle-detection range.

**How:**
1. Step from rest to 3 PWM levels (low, mid, high) and hold until speed settles.
2. Cut to 0 from each level and let it coast.
3. Film the coast next to the tape measure and read the true stopping distance.

**You provide:** the video, and the stopping distance read from it for each
speed.

**Data out:** `stop_*.csv` and the video.

## 3. Straight-line drift

**What it tells us:** how much of the drift is mechanical and how much
correction a path follower needs.

**How:**
1. Lay a 10 ft tape measure along the floor and a straight line beside it.
2. Command equal PWM on both wheels and drive the full 10 ft.
3. Measure the sideways offset at the end with the tape.
4. Repeat 3 times, then repeat in reverse.

**You provide:** the sideways offset for each run and its direction (left or
right).

**Data out:** `straight_*.csv` and your offset numbers.

> Status (2026-09-27, "unit1", one run, tape-measured): `drive_distance(40, "forward",
> 80, ...)` via the MCP server. Encoders: left wheel 41.56in, right wheel 43.14in (right
> ~1.6in farther over the run) -> +4.5deg computed heading; IMU agreed closely (+4.1deg).
> User confirmed by tape: left wheel traveled ~40in as commanded, right wheel a bit
> farther, and the robot drifted **left** -- exactly matching the encoder math's sign
> convention (right-farther = left drift). This lines up with section 4's finding that
> right turns are consistently faster than left on this unit: same root cause either
> way, most likely the right-side wheel/motor having a slight speed or effective-diameter
> edge over the left. Only one run so far (not the 3x-forward + 3x-reverse the "How"
> above asks for) -- re-run for a real average and to check reverse behaves the same way.

## 4. Turning in place

**What it tells us:** the lowest PWM that turns it, the top turn rate, and how
much the wheels scrub during a turn.

**How:**
1. Spin in place at several PWM values, both directions, using the IMU yaw rate.
2. Mark a start line on the floor and do a 180 degree turn at a mid PWM.
3. Film from above or side-on and note how far off 180 degrees it ended.

**You provide:** the video and the real angle turned.

**Data out:** `turn_*.csv`, video and measured angles. Do the distance
calibration (`d <inches>`) first, because the turn estimate depends on it.

> Status (2026-09-27, "unit1", robot on the ground): `calibration/characterize_turns.py`
> sweeps PWM x direction via the MCP server's `turn_degrees` tool and logs the results —
> see `calibration/turn_characterization_unit1_20260927_200733.csv`. Turned up something
> important first: the encoder-differential heading estimate overestimates in-place turns
> by ~2x due to wheel scrub (92 deg encoder vs ~47 deg real for a commanded 90 deg turn,
> confirmed by eye) — matches this section's own hunch to use "the IMU yaw rate," which
> `turn_degrees` now does exclusively for its stop condition and reported angle. Deadband
> for turning-in-place on this unit is between pwm 50 and 60 (40 and 50 both timed out at
> 12s with negligible rotation). Turn rate at pwm 60/80/100/125: left 9.7/25.7/36.8/49.8
> deg/s, right 14.7/29.3/40.1/52.6 deg/s — right turns are consistently faster than left,
> most pronounced near the deadband (~50% at pwm 60) and converging toward parity
> (~6%) at pwm 125. Read as this unit's individual wheel-alignment asymmetry, not a
> universal number — re-run the script per robot.

## 5. Latency and link loss (safety)

**What it tells us:** the delay from a command to movement, and what happens if
the radio link drops. `computer_bridge.ino` repeats the last command until told
otherwise, so we need to know whether the robot has its own timeout.

**How:**
1. Latency: Claude sends a step command and times the first encoder movement.
2. Link loss: with the robot running slowly, you unplug the bridge ESP32 (or
   stop the sending script) and watch whether the robot stops on its own.
   Be ready to grab it.

**You provide:** what you saw, and how long it kept moving.

> Status (2026-09-27, "unit1", robot on the ground): **Latency** — sent a raw drive
> command (pwm=60 forward) and polled encoder counts; first detected movement at 0.195s,
> consistent with `robot_motor_bts7960.ino`'s PWM ramp (RAMP_STEP/RAMP_DT_MS, ~333
> PWM/s, so ~0.18s to ramp from 0 to 60) dominating over ESP-NOW/encoder-broadcast
> latency. **Link loss** — started the robot moving slowly (pwm=50, jog) in a background
> process, let it run ~2.3s, then `kill -9`'d that process with zero graceful shutdown
> (no stop sent, no exit handler ran) to simulate a real crash, not a clean disconnect.
> User watched and confirmed the robot stopped almost immediately, "maybe half a
> second" — matching `computer_bridge.ino`'s `SERIAL_IDLE_TIMEOUT_MS=500` almost exactly.
> Confirms the firmware failsafe (added this session, see mcp_server/README.md) actually
> works against a real killed process, not just against a clean `stop`/`listen` line.

## 6. Surfaces and obstacles

**What it tells us:** how the numbers change on the surfaces you'll use.

**How:**
1. Repeat tests 1 to 3 on each surface (hard floor, carpet, etc.).
2. Try a door threshold and a small ramp, and note what it can climb and at
   what PWM.

**You provide:** surface names and the climb results.

**Data out:** separate CSVs per surface.

---

## Order of work

1. Section 0 (hand measurements) and the PWM cap.
2. Section 5 (latency and link loss), so we know the safety picture first.
3. Section 1 (PWM to speed).
4. Sections 2, 3, 4.
5. Section 6 for each new surface.

## Handing the data over

Put recordings and videos in one folder, with one line per file saying what it
is and any measurements you took. Claude can analyze CSVs with
`calibration/analyze_csv.py`; the video numbers go in your notes.
