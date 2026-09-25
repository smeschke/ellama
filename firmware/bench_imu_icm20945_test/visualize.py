#!/usr/bin/env python3
"""Live OpenCV view of bench_imu_icm20945_test.ino serial output.

Usage: python visualize.py [/dev/ttyUSB0] [baud]
Needs: pip install pyserial opencv-python numpy
Keys: q/ESC quit, r reset yaw display offset.
"""
import re
import sys
import collections
import numpy as np
import cv2
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

LINE_RE = re.compile(
    r"accel\(g\)=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\s+"
    r"gyro\(dps\)=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\s+\|\|\s+"
    r"pitch=(-?[\d.]+)\s+roll=(-?[\d.]+)\s+yaw=(-?[\d.]+)"
)

W, H = 1000, 600
HIST = 200
COLORS = [(80, 80, 255), (80, 220, 80), (255, 140, 60)]  # BGR for x, y, z


def rot(pitch, roll, yaw):
    p, r, y = np.radians([pitch, roll, yaw])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def project(pts, center, scale):
    # simple oblique view: x right, y up-ish, z up
    out = []
    for x, y, z in pts:
        sx = center[0] + scale * (y + 0.35 * x)
        sy = center[1] - scale * (z + 0.35 * x)
        out.append((int(sx), int(sy)))
    return out


def draw_cube(img, R, center, scale):
    v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1.4, 1.4) for sz in (-0.25, 0.25)])
    v = (R @ v.T).T
    p = project(v, center, scale)
    for i in range(8):
        for j in range(i + 1, 8):
            if bin(i ^ j).count("1") == 1:
                cv2.line(img, p[i], p[j], (200, 200, 200), 1, cv2.LINE_AA)
    for axis, col in zip(np.eye(3) * 1.8, COLORS):
        a = project([R @ axis], center, scale)[0]
        cv2.arrowedLine(img, center, a, col, 2, cv2.LINE_AA, tipLength=0.15)


def draw_plot(img, x0, y0, w, h, data, title, ylim, labels):
    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (70, 70, 70), 1)
    cv2.putText(img, title, (x0 + 6, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
    cv2.line(img, (x0, y0 + h // 2), (x0 + w, y0 + h // 2), (60, 60, 60), 1)
    for k in range(3):
        pts = []
        for i, row in enumerate(data):
            v = np.clip(row[k], -ylim, ylim)
            pts.append((x0 + int(i * w / HIST), y0 + int(h / 2 - v / ylim * (h / 2 - 4))))
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts)], False, COLORS[k], 1, cv2.LINE_AA)
        cv2.putText(img, labels[k], (x0 + w - 60 + 20 * k, y0 + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[k], 1)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.01)
    accel = collections.deque(maxlen=HIST)
    gyro = collections.deque(maxlen=HIST)
    pitch = roll = yaw = 0.0
    yaw_off = 0.0

    while True:
        # drain everything waiting on the port
        while True:
            raw = ser.readline()
            if not raw:
                break
            m = LINE_RE.search(raw.decode(errors="ignore"))
            if m:
                f = list(map(float, m.groups()))
                accel.append(f[0:3])
                gyro.append(f[3:6])
                pitch, roll, yaw = f[6:9]

        img = np.full((H, W, 3), 25, np.uint8)
        draw_cube(img, rot(pitch, roll, yaw - yaw_off), (250, 250), 110)
        cv2.putText(img, f"pitch {pitch:6.1f}  roll {roll:6.1f}  yaw {yaw - yaw_off:6.1f}",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        draw_plot(img, 520, 20, 460, 260, accel, "accel (g)", 2.0, "xyz")
        draw_plot(img, 520, 310, 460, 260, gyro, "gyro (deg/s)", 250.0, "xyz")
        cv2.imshow("ICM20945", img)

        k = cv2.waitKey(15) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("r"):
            yaw_off = yaw

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
