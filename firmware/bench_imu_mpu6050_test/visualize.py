#!/usr/bin/env python3
"""Live OpenCV view of bench_imu_mpu6050_test.ino serial output.

Shows an attitude display (pitch/roll as a tilting horizon, yaw as a compass),
the three accelerometer axes in g, the three gyro rates in deg/s, and a rolling
plot of pitch, roll and yaw. Rock the board by hand and check pitch and roll
track it; yaw drifts steadily, which is expected with no magnetometer.

The raw LSB values are shown alongside the converted ones, because those are the
numbers robot_imu_gy521.ino puts in its SensorPacket -- so this view doubles as a
check on what will go over the air.

Usage: python visualize.py [/dev/ttyUSB1] [baud]
Needs: pip install pyserial opencv-python numpy
Keys: q/ESC quit, r clear the plot and zero the yaw reference.

Opening the port resets the ESP32, so the gyro bias calibration runs right after
this script connects -- keep the board still and level for the first second, or
the gyro will read a constant offset forever after.
"""
import collections
import math
import re
import sys

import cv2
import numpy as np
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

# "RAW ax=1 ay=-2 az=16380 gx=0 gy=1 gz=-1 pitch=0.12 roll=-0.03 yaw=1.40 t=48213"
LINE_RE = re.compile(
    r"^RAW ax=(?P<ax>-?\d+) ay=(?P<ay>-?\d+) az=(?P<az>-?\d+)"
    r" gx=(?P<gx>-?\d+) gy=(?P<gy>-?\d+) gz=(?P<gz>-?\d+)"
    r" pitch=(?P<pitch>-?[\d.]+) roll=(?P<roll>-?[\d.]+) yaw=(?P<yaw>-?[\d.]+)"
    r" t=(?P<t>\d+)$"
)

# Must match the sketch's scale factors (accel +-2g, gyro +-250 dps).
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

W, H = 1040, 700
HIST = 400
ACCEL_FULL_G = 2.0          # bar full scale
GYRO_FULL_DPS = 250.0
PALETTE = {"x": (255, 160, 60), "y": (80, 200, 255), "z": (120, 220, 140)}  # BGR
ANGLE_COLORS = {"pitch": (255, 160, 60), "roll": (80, 200, 255), "yaw": (120, 220, 140)}
GREY = (110, 110, 110)
DIM = (70, 70, 70)
WHITE = (235, 235, 235)

state = dict(ax=0, ay=0, az=0, gx=0, gy=0, gz=0, pitch=0.0, roll=0.0, yaw=0.0, t=0,
             seen=False)
hist = {k: collections.deque(maxlen=HIST) for k in ("pitch", "roll", "yaw")}
yaw_ref = 0.0
boot_lines = collections.deque(maxlen=5)


def parse(line):
    m = LINE_RE.match(line)
    if not m:
        return False
    g = m.groupdict()
    for k in ("ax", "ay", "az", "gx", "gy", "gz", "t"):
        state[k] = int(g[k])
    for k in ("pitch", "roll", "yaw"):
        state[k] = float(g[k])
    state["seen"] = True
    hist["pitch"].append(state["pitch"])
    hist["roll"].append(state["roll"])
    hist["yaw"].append(state["yaw"] - yaw_ref)
    return True


def text(img, s, org, scale=0.5, color=WHITE, thick=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def draw_horizon(img, cx, cy, r):
    """Artificial horizon: the line tilts with roll and slides with pitch."""
    pitch, roll = state["pitch"], state["roll"]
    cv2.circle(img, (cx, cy), r, (45, 45, 45), -1)

    # Horizon offset: 1 degree of pitch = r/60 pixels, so +-60 deg fills the dial.
    off = max(-r, min(r, pitch * r / 60.0))
    a = math.radians(-roll)
    dx, dy = math.cos(a), math.sin(a)
    # Midpoint of the horizon line, displaced perpendicular to it by the pitch offset.
    mx, my = cx + dy * off, cy + dx * off
    p1 = (int(mx - dx * r), int(my - dy * r))
    p2 = (int(mx + dx * r), int(my + dy * r))

    mask = np.zeros(img.shape[:2], np.uint8)
    cv2.circle(mask, (cx, cy), r, 255, -1)
    ground = np.zeros_like(img)
    poly = np.array([p1, p2,
                     (int(p2[0] - dy * 2 * r), int(p2[1] - dx * 2 * r)),
                     (int(p1[0] - dy * 2 * r), int(p1[1] - dx * 2 * r))], np.int32)
    cv2.fillPoly(ground, [poly], (60, 45, 30))
    img[:] = np.where((mask[:, :, None] > 0) & (ground > 0), ground, img)

    cv2.line(img, p1, p2, WHITE, 2, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), r, GREY, 1)
    # Fixed aircraft reference, so you can see the horizon move against it.
    cv2.line(img, (cx - 30, cy), (cx - 8, cy), (60, 190, 240), 2)
    cv2.line(img, (cx + 8, cy), (cx + 30, cy), (60, 190, 240), 2)
    text(img, "pitch / roll", (cx - 44, cy + r + 22), 0.45, GREY)


def draw_compass(img, cx, cy, r):
    yaw = state["yaw"] - yaw_ref
    cv2.circle(img, (cx, cy), r, (45, 45, 45), -1)
    cv2.circle(img, (cx, cy), r, GREY, 1)
    for deg in range(0, 360, 30):
        a = math.radians(deg - 90)
        x0, y0 = cx + math.cos(a) * (r - 8), cy + math.sin(a) * (r - 8)
        x1, y1 = cx + math.cos(a) * r, cy + math.sin(a) * r
        cv2.line(img, (int(x0), int(y0)), (int(x1), int(y1)), GREY, 1)
    a = math.radians(yaw - 90)
    cv2.line(img, (cx, cy),
             (int(cx + math.cos(a) * (r - 12)), int(cy + math.sin(a) * (r - 12))),
             ANGLE_COLORS["yaw"], 3, cv2.LINE_AA)
    text(img, f"yaw {yaw:+.0f}", (cx - 34, cy + r + 22), 0.45, GREY)


def draw_bars(img, x0, y0, title, items, full_scale, unit):
    text(img, title, (x0, y0 - 10), 0.5, WHITE)
    for i, (axis, value, raw) in enumerate(items):
        by = y0 + i * 34
        cv2.rectangle(img, (x0, by), (x0 + 300, by + 20), (45, 45, 45), -1)
        mid = x0 + 150
        cv2.line(img, (mid, by), (mid, by + 20), GREY, 1)   # zero at the centre
        w = int(150 * max(-1.0, min(1.0, value / full_scale)))
        if w >= 0:
            cv2.rectangle(img, (mid, by), (mid + w, by + 20), PALETTE[axis], -1)
        else:
            cv2.rectangle(img, (mid + w, by), (mid, by + 20), PALETTE[axis], -1)
        text(img, axis, (x0 - 18, by + 15), 0.45, PALETTE[axis])
        text(img, f"{value:+7.2f} {unit}   raw {raw:+6d}", (x0 + 312, by + 15), 0.42, GREY)


def draw_plot(img, y0):
    x0, w, h = 60, W - 120, 150
    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (45, 45, 45), 1)
    text(img, "pitch / roll / yaw, degrees (newest right, autoscaled)",
         (x0, y0 - 8), 0.45, GREY)
    allv = [v for q in hist.values() for v in q]
    if not allv:
        return
    lo, hi = min(allv), max(allv)
    span = max(hi - lo, 10.0)
    for n, (name, q) in enumerate(hist.items()):
        pts = list(q)
        if len(pts) < 2:
            continue
        step = w / max(len(pts) - 1, 1)
        poly = [(int(x0 + i * step), int(y0 + h - h * (v - lo) / span))
                for i, v in enumerate(pts)]
        cv2.polylines(img, [np.array(poly, np.int32)], False, ANGLE_COLORS[name], 2,
                      cv2.LINE_AA)
        text(img, f"{name} {pts[-1]:+.1f}", (x0 + w + 8 - 120 * 0, y0 + 18 + 18 * n),
             0.42, ANGLE_COLORS[name])
    text(img, f"{hi:+.0f}", (x0 - 46, y0 + 12), 0.4, DIM)
    text(img, f"{lo:+.0f}", (x0 - 46, y0 + h), 0.4, DIM)


def main():
    global yaw_ref
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"Could not open {PORT}: {e}")

    buf = b""
    win = "GY-521 bench"
    cv2.namedWindow(win)
    while True:
        buf += ser.read(4096)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            if not parse(line) and not line.startswith("accel(g)="):
                # Boot chatter: WHO_AM_I, the gyro bias figures. Worth showing -- a bad
                # bias calibration (board moved during the first second) shows up only
                # here, and explains a gyro that reads a constant offset.
                boot_lines.append(line)

        img = np.zeros((H, W, 3), np.uint8)
        img[:] = (24, 24, 24)
        text(img, f"GY-521 (MPU6050) bench  --  {PORT} @ {BAUD}", (30, 36), 0.7, WHITE, 2)
        text(img, "q/ESC quit   r clear plot + zero yaw", (W - 350, 36), 0.45, GREY)

        if not state["seen"]:
            text(img, "waiting for data...", (30, 110), 0.7, GREY)
        else:
            draw_horizon(img, 150, 180, 100)
            draw_compass(img, 390, 180, 100)
            draw_bars(img, 580, 110, "accelerometer", [
                ("x", state["ax"] / ACCEL_LSB_PER_G, state["ax"]),
                ("y", state["ay"] / ACCEL_LSB_PER_G, state["ay"]),
                ("z", state["az"] / ACCEL_LSB_PER_G, state["az"]),
            ], ACCEL_FULL_G, "g")
            draw_bars(img, 580, 250, "gyro", [
                ("x", state["gx"] / GYRO_LSB_PER_DPS, state["gx"]),
                ("y", state["gy"] / GYRO_LSB_PER_DPS, state["gy"]),
                ("z", state["gz"] / GYRO_LSB_PER_DPS, state["gz"]),
            ], GYRO_FULL_DPS, "dps")
            text(img, f"sender t={state['t']} ms", (580, 350), 0.42, DIM)

        draw_plot(img, H - 210)
        for i, line in enumerate(boot_lines):
            text(img, line[:120], (30, H - 42 + i * 13), 0.35, DIM)

        cv2.imshow(win, img)
        k = cv2.waitKey(20) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("r"):
            yaw_ref = state["yaw"]
            for q in hist.values():
                q.clear()

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
