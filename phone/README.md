# Phone notes

The robot's Android phone runs Termux (F-Droid build) with Python, so the computer can run
small experiments on it. It is also the robot's camera (IP Webcam app).

## One-time setup on the phone (Termux)
`pkg install python openssh python-numpy python-pillow tur-repo python-onnxruntime termux-api`,
install the **Termux:API** Android app too, and authorize the computer's SSH key
(`~/.ssh/ellama_phone_ed25519.pub` -> `~/.ssh/authorized_keys` in Termux). Start `sshd`
(port 8022) after every Termux restart. The phone address is whatever the camera app shows.

## Facts
- Running a YOLO11n-pose ONNX model on the phone took ~97 ms/frame at 320x192 with 6 threads
  on the A13-class phone (~230 ms at 480x288).
- This phone has **no gyroscope or magnetometer** (accelerometer only), so yaw still comes from
  the robot's own IMU.

## Removed on 2026-10-12
The live-display server (`display_server.py`, `show.html`, `show.sh`) and the pose benchmark
(`bench.py`) went out with the line-following and turn-to-me experiments. They are in git at
the tag `before-experiment-removal`; the model files are in `~/ellama_experiments_archive/`.
