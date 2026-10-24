# Teleop

Python client/server pairs for driving eLlama, watching its camera, and viewing lidar sweeps over the network. Servers run on the robot's compute module (a Pi); clients run on your laptop.

## Scripts

| Script | Runs on | Does |
| --- | --- | --- |
| [`command_server.py`](command_server.py) | Pi | Receives drive commands (`FWD`, `REV`, `SPIN <nx>`, `STOP`) over TCP and forwards motion to the ESP-NOW sender over serial at 20 Hz |
| [`command_client.py`](command_client.py) | laptop | Connects to `command_server.py` and drives with the keyboard (w/s/a/d, up/down for speed, space to stop) |
| [`video_server.py`](video_server.py) | Pi | Captures the camera and streams JPEG frames over TCP |
| [`video_client.py`](video_client.py) | laptop | Connects to `video_server.py` and displays the live feed |
| [`lidar_server.py`](lidar_server.py) | Pi | Reads LD06 lidar packets over serial, accumulates one 360° sweep at a time, and streams each finished sweep over TCP |
| [`lidar_client.py`](lidar_client.py) | laptop | Connects to `lidar_server.py` and displays each completed sweep |

Each pair is independent — run whichever combination you need (e.g. `command_server.py` + `video_server.py` together for full teleop, or just the lidar pair on its own).

Servers listen on plain TCP with no encryption or auth, matching the firmware's ESP-NOW link — see [`firmware/README.md`](../firmware/README.md).
