# Phone display and benchmarks

The robot's Android phone runs Termux (F-Droid build) with Python, so the computer can put
live pictures on its screen and run small experiments there.

## One-time setup on the phone (Termux)
`pkg install python openssh python-numpy python-pillow tur-repo python-onnxruntime termux-api`,
install the **Termux:API** Android app too, and authorize the computer's SSH key
(`~/.ssh/ellama_phone_ed25519.pub` -> `~/.ssh/authorized_keys` in Termux). Start `sshd`
(port 8022) after every Termux restart. The phone address is whatever the camera app shows
(was 192.168.0.54).

## Live display (used by `start_line_follow(show_on_phone=True)`)
1. Copy `display_server.py` and `show.html` to `~/ellama/` on the phone:
   `scp -i ~/.ssh/ellama_phone_ed25519 -P 8022 display_server.py show.html termux@<phone>:ellama/`
2. In Termux: `cd ~/ellama && python display_server.py &`  (serves port 8080; kill it with
   `pkill -f display_server` from a *separate* command, not from the same ssh command line).
3. On the phone: `termux-open-url http://127.0.0.1:8080/show.html` (or open that address in the
   browser). Opening the plain image file with `termux-open` fails ("unable to display shared
   content"); the browser page works.
4. From the computer: `phone/show.sh picture.jpg` shows any image; the MCP server posts live
   frames with a motor-command panel when a run is started with `show_on_phone=true`
   (mcp_server/phone_display.py). Override the address with `ELLAMA_PHONE_DISPLAY`.

The display server has no login: anyone on the Wi-Fi can replace the picture. It only ever
writes `show.jpg`.

## Detector benchmark
`bench.py` runs the pose model (ONNX, numpy + onnxruntime + Pillow only) on an image and
times it: ~97 ms/frame at 320x192 with 6 threads on the A13-class phone (~230 ms at 480x288).
Models live in `models/` (gitignored; re-export from yolo11n-pose with imgsz=[320,192]).
This phone has **no gyroscope or magnetometer** (accelerometer only), so yaw still comes from
the robot's own IMU.
