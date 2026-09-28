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
| `zero_odometry()` | No | — |
| `stop()` | Commands zero PWM (active braking) | No — always safe |
| `listen()` | Stops and releases control to the stick controller | No — always safe |
| `connect(port=None)` | No | — |
| `drive_distance(distance_inches, direction, pwm, confirmed_safe)` | Yes | Yes |
| `turn_degrees(degrees, direction, pwm, confirmed_safe)` | Yes | Yes |
| `jog(left_pwm, right_pwm, duration_s, confirmed_safe)` | Yes (raw, time-based) | Yes |

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
