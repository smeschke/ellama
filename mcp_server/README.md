# MCP server

An MCP server that lets an AI agent drive eLlama by encoder feedback instead of by
guessing a duration — "go forward 10 inches" becomes a tool call that watches the wheel
encoders and stops itself at the target, rather than a fixed-time open-loop command.

Talks over the same USB-serial link as `calibration/live_view.py`:
[`firmware/computer_bridge/computer_bridge.ino`](../firmware/computer_bridge/computer_bridge.ino)
on a spare ESP32, plugged into whatever machine runs this server. Reuses
[`calibration/bridge.py`](../calibration/bridge.py) for serial I/O, line parsing, and the
distance/heading math — one place that logic lives, not two.

**Read [`../DRIVING_POLICY.md`](../DRIVING_POLICY.md) before using this.** The short
version, enforced in code by `robot_server.py`: every tool that can move the robot
requires `confirmed_safe=true`, which must only be set after asking the human operator
about that specific move, in the moment. This server has no obstacle sensors — only
wheel encoders and an IMU — so there is no way for it to verify the area is clear on its
own. If real proximity/vision sensing ever gets wired into this server, a sensor-verified
small bump could skip the ask; until then, ask every time, regardless of move size.
First-time-testing-a-new-motor and full-power moves require asking even after that.

## Setup

```
python3 -m venv --system-site-packages mcp_server/.venv
mcp_server/.venv/bin/pip install mcp
```

(`--system-site-packages` so it reuses the `pyserial` already installed for
`calibration/`, instead of a second copy.)

Plug in the bridge ESP32 (flashed with `computer_bridge.ino`) — usually `/dev/ttyUSB0`.
`robot_encoders_as5600` and a `robot_imu_*` sketch need to be running on the robot itself
for telemetry to flow; without them, every drive tool refuses to move (see "no live
encoder telemetry" below).

Register with an MCP client via `.mcp.json` (already set up at the repo root) or run
directly to sanity-check it starts:

```
mcp_server/.venv/bin/python mcp_server/robot_server.py
```

## Tools

| Tool | Moves the robot? | Needs `confirmed_safe`? |
| --- | --- | --- |
| `read_telemetry()` | No | — |
| `look(width=640)` | No | — |
| `zero_odometry()` | No | — |
| `stop()` | Commands zero PWM (active braking) | No — always safe |
| `listen()` | Stops and releases control to the stick controller | No — always safe |
| `connect(port=None)` | No | — |
| `start_recording(name=None, auto_photo=False)` | No | No |
| `add_marker(text)` | No | No |
| `take_photo(note="")` | No | No |
| `stop_recording()` | No | No |
| `drive_distance(distance_inches, direction, pwm, confirmed_safe)` | Yes | Yes |
| `turn_degrees(degrees, direction, pwm, confirmed_safe)` | Yes | Yes |
| `drive_arc(radius_inches, degrees, direction, pwm, confirmed_safe, max_wheel_pwm=125, travel="forward")` | Yes (curved; 360° = full circle; `travel="reverse"` backs up along the arc) | Yes |
| `jog(left_pwm, right_pwm, duration_s, confirmed_safe)` | Yes (raw, time-based) | Yes |
| `line_view(width=640)` | No | — |
| `person_view()` | No | — (finds people in the live frame, returns bearing to each) |
| `turn_to_me(confirmed_safe, max_seconds=30, name=None, spin_speed_dps=18, lost_timeout_s=1)` | Yes (spins in place, holds until it ends; max_seconds / lost_timeout_s of 0 = never) | Yes — once per run; refuses to start if 0 or >1 people are in view (ask the operator) |
| `turn_to_me_set(max_seconds, lost_timeout_s, spin_speed_dps)` | No (adjusts an already-approved run) | — |
| `turn_to_me_status(wait_s=0)` | No | — |
| `start_line_follow(confirmed_safe, speed_pwm=58, max_seconds=90, name=None, show_on_phone=False)` | Yes (autonomous, until it ends) | Yes — once per run (`show_on_phone` also shows the overlay + motor panel on the phone; see `phone/README.md`) |
| `line_follow_status(wait_s=0)` | No | — |

`look()` returns a photo from the phone camera on the robot (Android "IP Camera" or "IP
Webcam" app). The phone is found by scanning the local /24 for ports 8080/4444, or set
`ELLAMA_CAM_URL` (e.g. `https://192.168.0.54:4444`) to skip the scan. A snapshot takes
~5-8s. It is advisory vision only and does not replace `confirmed_safe`.

## Recording a stick-driven run

The server can log telemetry while you drive with the stick: it only transmits when a drive
tool is called, so the bridge stays listen-only. Ask the agent to `start_recording`, drive,
ask for markers and photos as you go, then `stop_recording`. Everything lands in
`recordings/<name>/`:

- `<name>.csv` — same columns as `calibration/live_view.py --csv` plus a trailing `note`
  column, so `python3 calibration/analyze_csv.py recordings/<name>/<name>.csv` works. Rows
  are `ENC` / `IMU` samples, plus `MARK` (from `add_marker`) and `PHOTO` (from `take_photo`)
  rows that carry the state at that moment and the text or photo filename in `note`.
- `photo_001.jpg`, ... — full-size phone-camera snapshots. The `PHOTO` row is stamped when the
  photo is requested; the snapshot itself arrives ~5-8 s later.

`start_recording(auto_photo=True)` photographs the robot by itself whenever it comes to rest
(wheels still ~0.7 s) after moving at least ~4 in of wheel travel or ~10° of yaw since the
last photo, plus one at the start. The `PHOTO` row is stamped when the robot stopped. The
snapshot takes ~5-8 s, so if the robot moves again before it arrives, a `MARK` row notes
"robot moved during capture" and that photo shouldn't be trusted for pose. Thresholds are
the `AUTO_PHOTO_*` constants in `robot_server.py`.

Only one program can hold the bridge's serial port, so `live_view.py` can't run at the same
time. Only one ESP32 may transmit drive commands at a time: keep the stick controller on for
this (nothing is sent to the robot) and don't call a drive tool while it drives.

`drive_distance` and `turn_degrees` are closed-loop: they stop themselves once the
target is confirmed reached, or after a generous timeout, whichever comes first.
`drive_distance` uses the wheel encoders; `turn_degrees` uses the IMU's gyro-integrated
yaw (see "Known unknowns" below for why). Both refuse to run without live telemetry from
the sensor they depend on, always stop-and-settle before starting (so a direction
reversal is never commanded without a stop in between), and cap how far a single call
can go (`MAX_SINGLE_MOVE_IN` / `MAX_SINGLE_TURN_DEG` in `robot_server.py`) — chain
further confirmed calls for more, checking `read_telemetry()` between them, per
`DRIVING_POLICY.md`'s stop-check-decide-move loop. `jog` is the odd one out: open-loop,
raw PWM, fixed duration, no target — for bench/commissioning tests, capped at
`MAX_JOG_DURATION_S`.

## Known unknowns

- **Direction sign: verified 2026-09-27.** Robot on blocks, wheels free to spin,
  `drive_distance(distance_inches=12, direction="forward", pwm=50, ...)` — all four
  wheels visibly spun backwards. `FORWARD_PWM_SIGN` is now `-1` to correct for this.
  Re-verify the same way after any change to wiring, motor board firmware, or which
  physical wheel is "left" vs "right".
- **Wheel encoders overestimate in-place turns by ~2x — verified 2026-09-27.** Robot on
  the ground, four 90° turns left then four right, pwm=60: the encoder-differential
  estimate (`dr - dl` over `track_width_in`) reported ~92° per commanded 90° turn, but
  the robot visually rotated only ~47° each time (confirmed by eye) — wheel scrub during
  an in-place spin turns the wheels much more than the chassis actually rotates. The
  IMU's gyro-integrated yaw over those same turns averaged ~49° per turn, matching the
  ~47° observed. `turn_degrees` now uses IMU yaw as its stop condition and ground truth;
  the encoder number is still returned as `encoder_degrees_estimate`, explicitly labeled
  unreliable. This means `track_width_in` miscalibration was never the real problem for
  turns specifically (it may still matter for other uses of encoder heading, e.g.
  `calibration/live_view.py`'s own dead-reckoning, which doesn't have an IMU fallback).
  IMU yaw drift over a multi-minute session is a separate, smaller concern this hasn't
  characterized yet — fine for single turns a few seconds long, unverified for anything
  longer.
- **Gear ratio is overridden in memory, not in the file.** `calibration/config.json`
  still says `gear_ratio: 6.33`; `calibration/analyze_run.py` found the true ratio is
  `4.5` (confirmed by `calibration/30_inches.csv`, a real tape-measured run). This server
  uses `4.5` regardless of what the file says — `TRUE_GEAR_RATIO` in `robot_server.py` —
  left that way deliberately so it could be tested independently before touching the file.

## Safety net beyond the confirmation gate

- `firmware/computer_bridge/computer_bridge.ino` now falls back to listen-only if no
  serial line arrives for 500ms while transmitting, so a crashed/killed server process
  doesn't leave the robot driving forever.
- The server itself sends `stop` on normal exit, `SIGINT`/`SIGTERM`, and at the end of
  every `drive_distance`/`turn_degrees` call (`try`/`finally`), whether or not the target
  was reached.
- Neither of those is a substitute for the confirmation gate — they bound how long a
  *stuck* command runs, not whether a command should have been sent at all.

## Line following

`start_line_follow` runs a camera -> detect -> steer loop on its own thread (`line_vision.py`
finds the 3/4in tape, `line_follow.py` turns it into wheel commands, `camera_stream.py` shares
the phone's `/video/mjpeg` stream). One `confirmed_safe` covers the run. It stops by itself and
reports why: `finished` (cross strip at the end), `lost` (no line for 0.5 s), `stale` (no camera
frame), `telemetry_stale`, `timeout`, or `stopped` (`stop()` / `listen()` abort it at any time).
It never searches for a lost line. Other drive tools refuse while it runs. The run is recorded
to `recordings/<name>/` (telemetry CSV, `video.mp4`, `video_times.csv`, `follow_log.csv`).
`line_follow_sim.py` is a closed-loop sanity check of the controller, not a prediction -- its
constants are fit from `tape_manual_run1/2` and need tuning on the real robot.

