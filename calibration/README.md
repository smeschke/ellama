# Calibration

Python tools that read telemetry from the ESP32 running
[`computer_bridge`](../firmware/computer_bridge/computer_bridge.ino), to validate and
calibrate the wheel encoders and IMU. The bridge is listen-only until a script sends it a
drive line, and nothing here sends one — so you can drive with the stick controller while
these run.

Needs on the robot: `robot_encoders_as5600` and a `robot_imu_*` sketch. Python: `pyserial`,
`matplotlib`.

| File | What it does |
|---|---|
| Operator dashboard | Live plots + distance/heading readout: run the robot daemon with `--http` and open http://127.0.0.1:8765 (see [`mcp_server/`](../mcp_server/README.md)). Replaces the old `live_view.py` desktop window (removed; in git history if needed). |
| [`../bridge/`](../bridge/) | Serial reader, line parsing, config load/save, and the distance/orientation math shared with the MCP server. |
| `config.json` | Wheel diameter, gear ratio (4.5: ~90t/20t teeth, confirmed by the tape-measured 30 in run), track width. Edit by hand and restart the daemon to apply. Old recordings baked 6.33 — see `analyze_run.py`. |
| `analyze_csv.py` / `analyze_run.py` | Post-run analysis of recording CSVs. |
| `*.csv` | Kept measurements, e.g. the PWM → speed characterization runs and `30_inches.csv`. Cited by `mcp_server/drive_math.py`; `ROBOT_CHARACTERIZATION.md` reports the numbers. |

## Check the sensors

```
mcp_server/.venv/bin/python mcp_server/robot_server.py --http --port 8765
```

then open http://127.0.0.1:8765. Only the daemon opens the serial port; the
page polls its `GET /state`, so it never fights the daemon for the bridge.

Just eyeballing whether a sensor is alive? Stop the daemon and stream the
bridge's raw serial prints (dense `@ seq NAME … *CC` lines; Ctrl+] quits):

```
mcp_server/.venv/bin/python -m serial.tools.miniterm /dev/ttyUSB0 921600
```

## Calibrate by hand

The old `d`/`t` typing is gone with `live_view.py`; the same corrections as
edits to `config.json` (daemon restart applies them):

1. Put the robot at a mark, note the dashboard distance, drive one foot with
   the stick. Multiply wheel diameter by (real inches) / (shown inches).
2. For turns: zero odometry, spin through a measured angle, multiply track
   width by (real degrees) / (shown heading degrees). Do distance first; the
   turn estimate depends on it.

`track_width_in` is 19 in, measured wheel-to-wheel on this unit. Units differ, so measure
yours or calibrate it with `t`. Precision matters less than it looks: every encoder-derived
turn figure is dominated by wheel scrub, which is far larger than a half-inch of track
width, so `turn_degrees` goes by the IMU regardless.
