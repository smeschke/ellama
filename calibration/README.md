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
| `live_view.py` | Live plots + a big readout of distance/heading since you last pressed `z`. Type `d <inches>` / `t <degrees>` to calibrate wheel diameter / track width from a real measurement. |
| `bridge.py` | Serial reader, line parsing, config load/save, and the distance/orientation math that `live_view.py` uses. |
| `config.json` | Wheel diameter, gear ratio (6.33, same as the bench sketch), track width. Read by `live_view.py`, which writes it when you calibrate. |
| `results/` | Kept measurements, e.g. the PWM → speed table. |

## Drive one foot

```
python3 calibration/live_view.py
```

1. Put the robot at a mark, press `z` in the plot window.
2. Drive one foot with the stick. The readout shows distance in inches and feet.
3. If it's off, type the measured distance in the terminal (`d 12`). The wheel diameter in
   `config.json` is rescaled and applied immediately. Press `z` and repeat to confirm.
4. For turns: `z`, spin the robot through a measured angle, type `t 90` (the real angle).
   Do the distance calibration first; the turn estimate depends on it.

`track_width_in` ships as a placeholder (20 in) — measure your wheel-to-wheel distance or
calibrate it with `t`.
