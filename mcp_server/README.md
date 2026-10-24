# MCP server

An MCP server that lets an AI agent drive eLlama by encoder feedback instead of by
guessing a duration — "go forward 10 inches" becomes a tool call that watches the wheel
encoders and stops itself at the target, rather than a fixed-time open-loop command.

Talks over the same USB-serial link as `calibration/live_view.py`:
[`firmware/computer_bridge/computer_bridge.ino`](../firmware/computer_bridge/computer_bridge.ino)
on a spare ESP32, plugged into whatever machine runs this server. Reuses
[`calibration/bridge.py`](../calibration/bridge.py) for serial I/O, line parsing, and the
distance/heading/range math — one place that logic lives, not two.

**Read [`../DRIVING_POLICY.md`](../DRIVING_POLICY.md) before using this.** The short
version, enforced in code by `robot_server.py`: every tool that can move the robot
requires `confirmed_safe=true`, which must only be set after asking the human operator
about that specific move, in the moment. The robot now has forward range sensors, and
range is not clearance: two narrow cones pointing ahead say nothing about the sides, the
floor, a drop, or anything behind. A sensor-verified small bump skipping the ask is still
not wired in, so ask every time, regardless of move size. First-time-testing-a-new-motor
and full-power moves require asking even after that.

## Sensors are not a fixed list

The firmware's `SensorPacket` carries the sender's own name, so sensors appear and
disappear on their own and both are normal: an encoder next to a wheel gets damaged, a ToF
board gets added, and a robot in a walled-off lab under a ceiling camera may carry no
sensors at all and still be fully instrumented.

So this server **reports what it has** and **enforces what it was told to need**:

- [`sensors()`](sensors.py) lists every stream that is actually transmitting, with age,
  rate and per-channel availability. Nothing registers sensors in advance — flashing a
  second ToF board as `TOF_REAR` makes it appear with no code change here, in `bridge.py`,
  or in the bridge firmware.
- `set_required_sensors(names, confirmed_safe)` is the policy: which sensors the drive
  tools insist on. Default `["enc", "imu"]`. `["enc"]` when the IMU board is dead; `[]`
  when something outside the robot is closing the loop. The result says what the new set
  costs you.
- Guards follow from the sensors, because every guard reads one. **No IMU means no tilt
  guard** — reported as a fact by `guards()`, never as a refusal to move. Whether driving
  in that state is acceptable was already settled by the policy.
- `set_guard(...)` turns the tilt guard on and off and retunes its thresholds while
  running: `set_guard("tilt", limit_deg=40)` for a ramp the robot is meant to climb.

Three sensor states that must not be collapsed into each other, and which `sensors()` and
`read_range()` keep apart:

| State | Means |
|---|---|
| `live: true, measuring: true` | working |
| `live: true, measuring: false` | reporting, but with nothing to report — a range sensor with nothing in its cone, or an encoder that is not responding |
| `live: false` | silent: board dead, unpowered, unflashed or absent |

"Nothing is in front of the robot" and "the robot cannot see" are different answers. A
missing encoder likewise reports `null`, never `0` — a fabricated zero reads as a
stationary wheel.

## Setup

`robot_server.py` imports [`calibration/bridge.py`](../calibration/bridge.py) from this
same repo, so the server and `calibration/live_view.py` compute distance, heading and range
from one copy of that math.

```
python3 -m venv --system-site-packages mcp_server/.venv
mcp_server/.venv/bin/pip install mcp
```

(`--system-site-packages` so it reuses the `pyserial` already installed for
`calibration/`, instead of a second copy.)

Plug in the bridge ESP32 (flashed with `computer_bridge.ino`) — usually `/dev/ttyUSB0`.
`robot_encoders_as5600`, a `robot_imu_*` sketch and `robot_tof_vl53l0x` run on the robot
itself; whichever of them are transmitting show up in `sensors()`. With the default policy,
missing encoders or IMU make the drive tools refuse — which is a decision you can change,
not a fact about the robot (see "Sensors are not a fixed list" above).

### Environment

All of these are startup defaults; the sensor and guard ones are settable at runtime.

| Variable | Does |
|---|---|
| `ELLAMA_SERIAL_PORT` | explicit serial port; default auto-detects |
| `ELLAMA_REQUIRED_SENSORS` | comma/space separated starting policy, default `enc,imu`; set it empty to require none |
| `ELLAMA_CAM_URL` | explicit phone-camera base URL; default scans the local /24 |
| `ELLAMA_TILT_LIMIT_DEG`, `ELLAMA_TILT_LOOKAHEAD_S`, `ELLAMA_TILT_GUARD` | tilt guard start values (`set_guard` changes them live) |

Register with an MCP client via `.mcp.json` (already set up at the repo root) or run
directly to sanity-check it starts:

```
mcp_server/.venv/bin/python mcp_server/robot_server.py
```

## Tools

| Tool | Moves the robot? | Needs `confirmed_safe`? |
| --- | --- | --- |
| `read_telemetry()` | No | — |
| `sensors()` | No | — (every stream that is transmitting, what each guard can do, the required set and what is missing) |
| `read_range()` | No | — (closest obstacle from the forward ToF cones; distinguishes "nothing in range" from "not reporting") |
| `set_required_sensors(names, confirmed_safe)` | No | `confirmed_safe` — it changes what the robot will do with no feedback |
| `guards()` | No | — |
| `set_guard(name="tilt", mode=None, limit_deg=…, lookahead_s=…)` | No | No (it cannot clear a latched fault) |
| `look(width=640)` | No | — |
| `save_photo(name="", preview_width=0, open_in_browser=False)` | No | — (saves a full-res photo to `recordings/photos/` with a `.json` of the robot's pose; `preview_width` > 0 also returns the picture) |
| `phone_status()` | No | — (the phone's battery percent and torch state, from the camera app's `/info.json`; `low_battery` at or below 20%) |
| `torch(on)` | No | — (phone flashlight on/off, state read back; turn it on when the view is dark, off when idle -- it drains the phone) |
| `zero_odometry()` | No | — |
| `stop()` | Commands zero PWM (active braking) | No — always safe |
| `listen()` | Stops and releases control to the stick controller | No — always safe |
| `connect(port=None)` | No | — |
| `start_recording(name=None, auto_photo=False)` | No | No |
| `add_marker(text)` | No | No |
| `take_photo(note="")` | No | No |
| `stop_recording()` | No | No |
| `drive_distance(distance_inches, direction, pwm, confirmed_safe, encoder="both")` | Yes | Yes |
| `turn_degrees(degrees, direction, pwm, confirmed_safe)` | Yes | Yes |
| `drive_arc(radius_inches, degrees, direction, pwm, confirmed_safe, max_wheel_pwm=125, travel="forward", encoder="both")` | Yes (curved; 360° = full circle; `travel="reverse"` backs up along the arc) | Yes |
| `drive_until_range(target_mm, pwm, confirmed_safe, facing="auto", sensor=None, tolerance_mm=15, min_pwm=50)` | Yes (straight, toward whatever the ToF faces, until it reads `target_mm`) | Yes |
| `jog(left_pwm, right_pwm, duration_s, confirmed_safe, require_telemetry=True)` | Yes (raw, time-based) | Yes |
| `speak_then_drive(text, pwm, duration_s, confirmed_safe, direction="forward", warn_lights=True)` | Yes (`speak` then straight open-loop `jog`, one call, no model round trip between) | Yes |
| `set_level_reference(confirmed_level)` | No (records the current orientation as "level" for the tilt guard; refused while moving or if it would shift the reference by > half the limit) | `confirmed_level` — only after the operator says the robot is on level ground |
| `clear_tilt_fault(confirmed_safe)` | No (releases a tripped tilt guard; refused while still leaning within 5° of the limit) | `confirmed_safe` — only after the operator has looked at the robot |
| `lights(on=True, brightness, color, effect, speed, intensity)` | No (WLED strip at `ELLAMA_WLED_HOST`, default 192.168.0.57) | — |
| `list_light_effects()` | No | — |
| `lights_blink(blinks=2, phase_s=0.3, color="00ff00", brightness=127, restore=True, fade=True)` | No (on off on off on = 1.5 s by default, each phase fading) | — |
| `speak(text, speed=1.3, max_brightness=127)` | No (Piper TTS out of this computer's speakers, WLED strip animates a mouth; blocks until done. Env: `ELLAMA_PIPER`, `ELLAMA_PIPER_VOICE`) | — |
| `perform(midi_path=None, bpm=None, audio_path=None, synth=False, dance=False, confirmed_safe=False, dance_pwm=60, spin_pwm=85, brightness=127, lead_in_s=3, audio_offset_s=0, start_s=0, max_seconds=None)` | Only with `dance=True` (open-loop wheel moves, one per beat) | Only with `dance=True` — lights + sound need none |
| `perform_status(wait_s=0)` | No | — |
| `hear(wait_seconds=10, max_seconds=20, pause_seconds=0.8)` | No (records the webcam mic on this computer, waits for speech, stops at a pause, transcribes with faster-whisper `base.en`; blocks. Alternate with `speak` for a conversation. Env: `ELLAMA_MIC`, `ELLAMA_MIC_MATCH`, `ELLAMA_WHISPER`. Test: `.venv/bin/python hearing.py`) | — |

### `perform` — a song on the mouth bar (and a dance)

Starts a light show timed to a MIDI file (or just a tempo) and returns at once; `perform_status(wait_s)`
follows it, `stop()` ends it. Engine: [`perform.py`](perform.py) (pure logic, tested offline); the thread
and the tools are in `robot_server.py`.

- **Timing**: `midi_path` (notes drive the bar: pitch = where a coloured spot lands, kick = burst from the
  middle, snare = flash at both ends, hats = sparkle; the tempo map gives the beat) or `bpm` + `duration_s`
  (beat pulses only — the safer choice when your recording won't line up with any MIDI you have).
- **Sound**: `audio_path` plays a recording from this computer (ffplay) in sync; `synth=True` renders the MIDI
  as a chiptune ("the robot sings"); neither = start your own player at the end of the countdown. ffplay takes a
  few tenths of a second to start, so tune `audio_offset_s` once. Rehearse a bit with `start_s` / `max_seconds`.
- **Dance** (`dance=True`): one open-loop move per beat, re-sent every 0.1 s — an 8-beat phrase of two arc sways,
  rock forward/back, spin each way. It needs `confirmed_safe=true` like any drive tool (and live telemetry,
  unless `require_telemetry=false`), a green warning blink first, and a stop is inserted whenever a wheel has to
  reverse. Forward effort and spins are separate knobs (`dance_pwm`, `spin_pwm`) because spins need far more PWM
  to beat wheel scrub. The sway's inner wheel (40% of `dance_pwm`) is under the motor deadband at 60, so the
  sway is really a pivot on one wheel. Not yet run on the robot; try it on blocks first.
- No MIDI of a copyrighted song is bundled; supply your own `.mid` (and recording).

`look()` returns a full-res photo from the phone camera on the robot (Android "IP Camera" or
"IP Webcam" app), or a clear error; it never substitutes a frame of the live stream. The
address is remembered in `~/.cache/ellama/camera_url`; if that is missing or the phone has
moved, the phone is found by scanning the local /24 for ports 8080/4444 (slow, 10-30 s, and it
seems to stall the phone's server), or set `ELLAMA_CAM_URL` (e.g. `https://192.168.0.54:4444`)
to skip the scan. A photo takes ~1-8 s and fails after 12 s. If the app answers but no photo
comes back, the error says so (restart the camera app on the phone). It is advisory vision
only and does not replace `confirmed_safe`.

`save_photo()` is the same camera, saved to a file instead of shown: it writes
`recordings/photos/<time>[_name].jpg` (override with `ELLAMA_PHOTO_DIR`) plus a `.json` with the
robot's odometry pose at that moment, the image brightness, and `moved_during_capture`. The
folder ignores itself in git. It is the hook for off-robot image work and for the loop *drive ->
stop -> photo -> analyse -> decide -> drive*: the `drive_*` / `turn_degrees` tools stop
themselves, `save_photo` takes the picture and returns its path (add `preview_width=640` to
see it in the same call), and any script or model on this computer can read the file. Compare
`robot.dist_avg_in` / `heading_deg` between photos to place them; call `zero_odometry()` first
if you want them relative to a start point.

## Recording a stick-driven run

The server can log telemetry while you drive with the stick: it only transmits when a drive
tool is called, so the bridge stays listen-only. Ask the agent to `start_recording`, drive,
ask for markers and photos as you go, then `stop_recording`. Everything lands in
`recordings/<name>/`:

- `<name>.csv` — same columns as `calibration/live_view.py --csv`, plus `range_closest_mm`
  and a trailing `note` column, so `python3 calibration/analyze_csv.py
  recordings/<name>/<name>.csv` works. Rows are `ENC` / `IMU` samples, a row per packet from
  any other sensor with its own name as the source (`TOF_FRONT`, …), plus `MARK` (from
  `add_marker`), `PHOTO` (from `take_photo`) and `CONFIG` (a guard or sensor-policy change,
  so a run shows what it was driven under) rows that carry the state at that moment and the
  text, filename or setting in `note`.
  A numeric cell that is not known holds `nan`, never `0` — `enc_heading_deg` is `nan`
  whenever an encoder was not responding, since a heading from one live wheel and one frozen
  one would look plausible and drift without bound. `analyze_csv.py` loads those fine
  (`float("nan")` is a number), but an unguarded mean over such a column comes out NaN,
  which is the honest answer: part of that run was not measured.
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

`drive_distance`, `turn_degrees` and `drive_until_range` are closed-loop: they stop
themselves once the target is confirmed reached, or after a generous timeout, whichever
comes first. `drive_distance` uses the wheel encoders; `turn_degrees` uses the IMU's
gyro-integrated yaw (see "Known unknowns" below for why); `drive_until_range` uses the ToF
median and needs neither encoder nor yaw, which is what makes it usable when the encoders
are not trustworthy. What they do without that sensor depends on the
policy, and every result says which feedback it actually used: `drive_distance` falls back
to a timed move off the measured PWM-speed curve with `closed_loop: false` and
`distance_source` saying it was predicted rather than measured; `turn_degrees` falls back to
the encoder differential divided by the measured 1.96x scrub factor, with
`feedback_source: "encoders"`. `drive_arc` has no fallback and says so — "curve until the
yaw reads N degrees" has no meaning without yaw, and a fixed wheel split for a fixed time is
just `jog`. All of them abort mid-move, rather than raising, if an encoder drops out while
the wheels are turning. They always stop-and-settle before starting (so a direction
reversal is never commanded without a stop in between), and cap how far a single call
can go (`MAX_SINGLE_MOVE_IN` / `MAX_SINGLE_TURN_DEG` in `robot_server.py`) — chain
further confirmed calls for more, checking `read_telemetry()` between them, per
`DRIVING_POLICY.md`'s stop-check-decide-move loop. `jog` is the odd one out: open-loop,
raw PWM, fixed duration, no target — for bench/commissioning tests, capped at
`MAX_JOG_DURATION_S`.

**Which encoder measures a move.** `drive_distance` and `drive_arc` take
`encoder="both"` (the default, averaging the wheels), `"left"` or `"right"`. One wheel is
the same distance as the average on a straight line, and the only honest answer when the
other channel is dead. The default is not a safe default on a robot with a bad encoder: an
average that includes a channel frozen at a constant count rises at half rate, so a 12in
request drives 24in. Both tools now refuse that instead — a selected channel that has not
ticked while another selected wheel has covered `FROZEN_COMPARE_IN` (0.25in), or that has
not ticked at all within `ENCODER_FIRST_TICK_S` (0.6s), aborts the move and names the
wheel. Note a single encoder is NOT offered on `turn_degrees`: during a spin in place the
wheels turn opposite ways and scrub, so neither one wheel nor their average is a distance
the robot travelled, which is why that tool uses the IMU. On an arc, the measuring wheel
runs its own radius — half a track wider (outer) or tighter (inner) than the robot's
centre — so the curvature loop compares against `1/(radius ± half track)` for that wheel
and scales the reported path back to the centre; a single encoder is refused when the
inner wheel would be pivoting inside `MIN_ARC_RADIUS_IN`.

**Approaching something by range.** `drive_until_range(target_mm, pwm, ...)` drives
straight toward whatever a ToF sensor is looking at until its median reads `target_mm`,
then stops. It drives in the direction the sensor *faces*, since that is the only
direction it can measure; with `facing="auto"` that is read off the sensor's **name**
(`TOF_REAR` → the robot reverses, anything else → forward), so a board mounted on the back
but flashed `TOF_FRONT` will send the robot the wrong way — pass
`facing="forward"`/`"reverse"` to override, and the result always says what it decided and
why. Two phases, because the sensor is slow relative to the robot: a continuous one that
tapers so it never commands more speed than it can stop within the gap that is left, then
short pulses at `min_pwm`, each followed by a full stop and a fresh median, for the last
few centimetres. It stops short rather than overshoot, and aborts — saying which — if the
sensor goes quiet, its median goes out of range (blind, not clear), or nothing gets closer
for `RANGE_NO_PROGRESS_S`. The ToF median lags by half its window while the robot moves
(observed 355mm against a raw 257mm at ~170mm/s), which is why the last stretch is pulsed
instead of crept: `RANGE_LATENCY_S` folds that lag, one sample interval and the braking
lead together.

`drive_distance`, `turn_degrees`, `drive_arc` and `jog` take `warn_lights=True`: right before the motors start the WLED strip does the 1.5 s fading `lights_blink` pattern (on off on off on, then solid green at 50% while moving) and is restored when the move ends. Pass `warn_lights=False` to skip it. A strip that can't be reached never blocks a move.

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

## Tilt guard (`tilt_guard.py`)

The robot is low and can run banks up to ~25 deg; past that it tips. The IMU feeds a guard
that watches the lean of the chassis and stops the robot before it goes over.

- **What it measures:** the angle between "up" now and "up" at the *level reference*, so
  the IMU's own mounting offset (a few degrees on a flat floor) doesn't count and a diagonal
  bank counts in full. The reference is captured automatically from the first ~0.5 s of a
  still, live IMU after the server starts; `set_level_reference` re-takes it.
- **When it trips:** tilt plus a short look-ahead on how fast it is growing reaches the limit
  (default 25 deg; the look-ahead adds at most 30% of the limit, so a fast lean trips at about
  17.5 deg and a slow bank at 25), for 2 IMU samples in a row. Replayed against the 11
  recordings in `recordings/` that carry tilt data, the worst ordinary tilt seen was 5.7 deg, no trips.
- **What it does:** sends `stop` from the IMU thread within ~20 ms of the sample, **latches**,
  and refuses every drive command (`send_drive` is the single choke point, so `jog`,
  `drive_*`, `turn_*` and `perform` are all covered) until
  `clear_tilt_fault`. Autonomous runs are aborted; blocking tools return
  `{"ok": false, "error": "tilt guard: ...", "tilt_guard": {...}}`.
- **No IMU is a fact, not a refusal:** with the IMU silent for 0.5 s this guard reports
  `active: false` with `inactive_because: "no live IMU"` and stops judging. Whether the robot
  may drive unguarded is the sensor policy's call — by default `imu` is required, so it
  still refuses; drop it with `set_required_sensors(["enc"])` and the robot drives with no
  tilt guard, which `sensors()` states plainly. With a live IMU but no level reference yet,
  it asks you to wait ~0.5 s. Zero/stop commands always go out regardless.
- **Retunable while running:** `set_guard("tilt", mode=…, limit_deg=…, lookahead_s=…)`.
  `mode="warn"` records what it would have tripped on without ever stopping the robot —
  the way to tune on a real robot. Switching it off suspends enforcement but does **not**
  erase a latched fault; it comes back when you switch it on again.
- **Survives `connect()`:** the guard lives for the server process, so reconnecting cannot clear
  a fault or re-learn "level" from a robot that is leaning. Restarting the server resets it.
- **Reporting:** `read_telemetry()` has a `tilt` block (tilt, rate, limit, reference, fault).
- **Env (start-up values):** `ELLAMA_TILT_LIMIT_DEG`, `ELLAMA_TILT_LOOKAHEAD_S`,
  `ELLAMA_TILT_GUARD=off`. `set_guard` overrides them at runtime.
- **Limits:** it only covers commands that go through this server -- the stick controller is
  not guarded. A stop cannot undo momentum: the guard buys the ~0.3 s the motor ramp needs, it
  does not make a bank safe. A guard in the motor-board firmware would be the stronger layer.
- **Tests:** `ELLAMA_SERIAL_PORT=/dev/none .venv/bin/python test_tilt_guard.py` (no hardware;
  also replays the recordings).

## Tests

All offline, no robot needed:

```
ELLAMA_SERIAL_PORT=/dev/none .venv/bin/python test_sensors.py       # 24: registry, policy, degraded modes
ELLAMA_SERIAL_PORT=/dev/none .venv/bin/python test_tilt_guard.py    # 14: + the send_drive choke point
ELLAMA_SERIAL_PORT=/dev/none .venv/bin/python test_drive_feedback.py # 35: encoder= selection, range approach, jog guard restore
ELLAMA_SERIAL_PORT=/dev/none .venv/bin/python test_phone_status.py  #  6
ELLAMA_SERIAL_PORT=/dev/none .venv/bin/python test_save_photo.py    #  6
```

`test_sensors.py` covers the states that are slow to stage on a real robot: an encoder
dropping out mid-run, a ToF reporting with nothing in range, a stream going quiet, and the
policy letting the robot drive with sensors absent.

## Safety net beyond the confirmation gate

- `firmware/computer_bridge/computer_bridge.ino` now falls back to listen-only if no
  serial line arrives for 500ms while transmitting, so a crashed/killed server process
  doesn't leave the robot driving forever.
- The server itself sends `stop` on normal exit, `SIGINT`/`SIGTERM`, and at the end of
  every `drive_distance`/`turn_degrees` call (`try`/`finally`), whether or not the target
  was reached.
- Neither of those is a substitute for the confirmation gate — they bound how long a
  *stuck* command runs, not whether a command should have been sent at all.

## Removed experiments

Line following (`start_line_follow`, `line_view`, `line_follow_status`, `line_follow_report`) and
person tracking (`turn_to_me*`, `person_view`) were built on 2026-10-02/03 and taken out on
2026-10-12 to keep this server simple. Code, run logs and stills are in git at the tag
`before-experiment-removal`; the run videos and pose models are in
`~/ellama_experiments_archive/`. The write-up of what worked and what did not is the doc
"eLlama experiments: line following and person tracking".
