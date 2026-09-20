# eLlama

Four-wheel-drive outdoor robot platform, walking-speed, that ships as a complete working robot and doubles as a mount for teleoperated or autonomous compute.

RS550 brushed motors with 100:1 gearboxes, 10 in pneumatic wheels, dual 12V 7Ah SLA batteries, BTS7960 motor drivers, ESP32-based control over ESP-NOW.

## Repo layout

| Path | What's there |
| --- | --- |
| [`context-pack/`](context-pack/) | Design docs and the [user manual](<context-pack/eLlama User Manual.md>) |
| [`firmware/`](firmware/) | ESP32 Arduino sketches (controller, motor board, bridge) |
| [`teleop/`](teleop/) | Python client/server scripts for driving, video, and lidar over the network |
| [`simulation/`](simulation/) | Calibrated 2D/3D drive-command simulators and a CSV replay script |
| [`images/`](images/) | Photos and diagrams |
