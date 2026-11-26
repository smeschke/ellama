# Robot profiles

One dashboard serves several robots. Each robot gets a profile here, and the
dashboard runs with `ELLAMA_ROBOT=<name>` (or `./run_dashboard.sh <name>`).
Unset means the historical default: [`../config.json`](../config.json).

## Adding a robot

Copy [`../config.json`](../config.json) to `<name>.json` and measure that
robot's values into it. A profile holds the same calibration keys plus one
optional policy key:

| Key | Meaning |
| --- | --- |
| `wheel_diameter_in`, `ticks_per_rev`, `gear_ratio`, `track_width_in`, `encoder_sign` | Same as `config.json`: measured per robot. Restart the daemon to apply. |
| `required_sensors` | Optional list (e.g. `["enc"]` for an IMU-less robot, `[]` if something outside the robot closes the loop). An explicit `ELLAMA_REQUIRED_SENSORS` env var always wins over this. Missing key means the default `["enc", "imu"]`. |
| `drive_swap_sides` | Optional boolean. `true` if a spin-left command turns this robot's nose clockwise (mirrored motor wiring, like the original unit); `false` if spin-left already turns it counter-clockwise. Missing key means the `DRIVE_SWAP_SIDES` default in `drive_math.py` (currently `true`). Must be a real JSON boolean, not a string. |

A named profile that does not exist is a startup error, never a silent
fallback — a typo must not mis-calibrate distances.

## The robots

| Name | Profile | Encoder firmware | Notes |
| --- | --- | --- | --- |
| Lisa | [`lisa.json`](lisa.json) | [`robot_encoders_as5600`](../../firmware/robot_encoders_as5600/) (one classic ESP32, two I2C buses) | The original unit: mirrored motor wiring (`drive_swap_sides: true`), forward drives read negative encoder counts (`encoder_sign [-1,-1]`). Same values as `../config.json`. |
| Lenny | [`lenny.json`](lenny.json) | [`robot_encoders_as5600_lenny`](../../firmware/robot_encoders_as5600_lenny/) (two ESP32-C3 boards, one encoder each) | Straight motor wiring (`drive_swap_sides: false`), `encoder_sign [1,1]`. |

New robots get a name and a `<name>.json` here; robot-specific firmware goes in a sketch folder suffixed `_<name>`.
