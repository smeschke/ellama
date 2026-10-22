#!/usr/bin/env python3
"""Live OpenCV view of bench_encoders_as5600_test.ino serial output.

Shows a dial per encoder (needle = magnet angle, 0 at the top, increasing
clockwise on screen), the unwrapped revs and RPM, the magnet status the sketch
reported at boot, and a rolling RPM plot for both encoders. Spin each wheel by
hand and check the matching dial moves.

Usage: python visualize.py [/dev/ttyUSB0] [baud]
Needs: pip install pyserial opencv-python numpy
Keys: q/ESC quit, r zero the revs display for both encoders.

Opening the port resets the ESP32, so the boot-time magnet check lines arrive
right after this script connects.
"""
import re
import sys
import collections
import numpy as np
import cv2
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

# "left  angle=  12.3 deg  revs=   0.034  rpm=    1.2  raw= 140  || wheel_revs=   0.005  wheel_rpm=   0.2"
LINE_RE = re.compile(
    r"^(left|right)\s+angle=\s*(-?[\d.]+)\s+deg\s+revs=\s*(-?[\d.]+)\s+rpm=\s*(-?[\d.]+)\s+raw=\s*(\d+)"
    r"\s+\|\|\s+wheel_revs=\s*(-?[\d.]+)\s+wheel_rpm=\s*(-?[\d.]+)"
)
STATUS_RE = re.compile(r"^\[(left|right)\]\s+(.*)$")
# "left  NOT_RESPONDING" -- the sketch prints this instead of a row of zeros when nothing
# answers on that encoder's bus. The encoders sit by the wheels and get damaged, so this
# is an expected state: the view keeps running on whichever encoder is left.
MISSING_RE = re.compile(r"^(left|right)\s+NOT_RESPONDING$")

W, H = 1000, 640
HIST = 300              # ~30 s of RPM history at 10 Hz
NAMES = ("left", "right")
COLORS = {"left": (255, 160, 60), "right": (80, 200, 255)}  # BGR
DIAL_R = 130
DIAL_CENTERS = {"left": (260, 190), "right": (740, 190)}

# Per-encoder state
enc = {
    n: dict(angle=0.0, revs=0.0, rpm=0.0, raw=0, wheel_revs=0.0, wheel_rpm=0.0,
            revs_off=0.0, seen=False, missing=False,
            status="waiting for boot check...", hist=collections.deque(maxlen=HIST))
    for n in NAMES
}


def handle_line(text):
    m = LINE_RE.match(text)
    if m:
        name = m.group(1)
        f = list(map(float, m.groups()[1:]))
        e = enc[name]
        e["angle"], e["revs"], e["rpm"], e["raw"], e["wheel_revs"], e["wheel_rpm"] = f
        e["seen"] = True
        e["missing"] = False
        e["hist"].append(e["rpm"])
        return
    m = MISSING_RE.match(text)
    if m:
        e = enc[m.group(1)]
        e["missing"] = True
        e["status"] = "not responding -- check wiring/power"
        # Feed 0 rpm so the plot keeps its time axis honest rather than freezing the
        # last value, which would read as a wheel still turning.
        e["hist"].append(0.0)
        return
    m = STATUS_RE.match(text)
    if m:
        name, msg = m.groups()
        if ("Magnet detected" in msg or "MAGNET" in msg or "NOT RESPONDING" in msg
                or "came back" in msg or "WARNING" in msg):
            enc[name]["status"] = msg.strip()
            if "came back" in msg or "Magnet detected" in msg:
                enc[name]["missing"] = False
            elif "NOT RESPONDING" in msg:
                enc[name]["missing"] = True
        elif "AGC" in msg:
            enc[name]["agc"] = msg.strip()


def status_color(status):
    s = status.lower()
    if "not responding" in s:
        return (60, 60, 255)      # red
    if "not detected" in s or "warning" in s:
        return (60, 60, 255)      # red
    if "too weak" in s or "too strong" in s:
        return (0, 200, 255)      # amber
    if "detected" in s:
        return (100, 220, 100)    # green
    return (160, 160, 160)


def draw_dial(img, name):
    e = enc[name]
    cx, cy = DIAL_CENTERS[name]
    col = COLORS[name]
    cv2.putText(img, name.upper(), (cx - 30, cy - DIAL_R - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), DIAL_R, (90, 90, 90), 2, cv2.LINE_AA)
    for k in range(12):
        a = np.radians(k * 30)
        r0 = DIAL_R - (14 if k % 3 == 0 else 7)
        p0 = (int(cx + r0 * np.sin(a)), int(cy - r0 * np.cos(a)))
        p1 = (int(cx + DIAL_R * np.sin(a)), int(cy - DIAL_R * np.cos(a)))
        cv2.line(img, p0, p1, (110, 110, 110), 1, cv2.LINE_AA)
    if e["missing"]:
        # No needle at all: a needle frozen at its last angle looks like a stopped wheel.
        cv2.putText(img, "NOT", (cx - 34, cy - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (60, 60, 255), 2, cv2.LINE_AA)
        cv2.putText(img, "RESPONDING", (cx - 72, cy + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (60, 60, 255), 1, cv2.LINE_AA)
    elif e["seen"]:
        a = np.radians(e["angle"])
        tip = (int(cx + (DIAL_R - 10) * np.sin(a)), int(cy - (DIAL_R - 10) * np.cos(a)))
        cv2.line(img, (cx, cy), tip, col, 3, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 5, col if not e["missing"] else (70, 70, 70), -1, cv2.LINE_AA)

    ty = cy + DIAL_R + 28
    if e["missing"]:
        live = "--- no data ---"
    elif not e["seen"]:
        live = "no data"
    else:
        live = f"angle {e['angle']:6.1f} deg   raw {int(e['raw'])}"
    cv2.putText(img, live, (cx - 130, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    if e["missing"]:
        cv2.putText(img, "revs    ---        rpm    ---", (cx - 130, ty + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 120, 120), 1, cv2.LINE_AA)
        cv2.putText(img, "wheel revs  ---    rpm   ---", (cx - 130, ty + 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (110, 110, 110), 1, cv2.LINE_AA)
    else:
        cv2.putText(img, f"revs {e['revs'] - e['revs_off']:8.3f}    rpm {e['rpm']:7.1f}",
                    (cx - 130, ty + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(img, f"wheel revs {e['wheel_revs'] - e['revs_off'] / 6.33:7.3f}  rpm {e['wheel_rpm']:6.1f}",
                    (cx - 130, ty + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(img, e["status"][:44], (cx - 130, ty + 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                status_color(e["status"]), 1, cv2.LINE_AA)


def draw_rpm_plot(img, x0, y0, w, h):
    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (70, 70, 70), 1)
    cv2.putText(img, "encoder RPM", (x0 + 6, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
    cv2.line(img, (x0, y0 + h // 2), (x0 + w, y0 + h // 2), (60, 60, 60), 1)
    peak = max([50.0] + [abs(v) for n in NAMES for v in enc[n]["hist"]])
    peak = float(np.ceil(peak / 50.0) * 50.0)
    cv2.putText(img, f"+{peak:.0f}", (x0 + 6, y0 + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (130, 130, 130), 1)
    cv2.putText(img, f"-{peak:.0f}", (x0 + 6, y0 + h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (130, 130, 130), 1)
    for k, n in enumerate(NAMES):
        pts = []
        for i, v in enumerate(enc[n]["hist"]):
            pts.append((x0 + int(i * w / HIST), y0 + int(h / 2 - v / peak * (h / 2 - 6))))
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts)], False, COLORS[n], 1, cv2.LINE_AA)
        cv2.putText(img, n, (x0 + w - 110 + 55 * k, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[n], 1)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.01)

    while True:
        # drain everything waiting on the port
        while True:
            raw = ser.readline()
            if not raw:
                break
            handle_line(raw.decode(errors="ignore").strip())

        img = np.full((H, W, 3), 25, np.uint8)
        for n in NAMES:
            draw_dial(img, n)
        draw_rpm_plot(img, 20, 460, W - 40, 160)
        cv2.imshow("AS5600 encoders", img)

        k = cv2.waitKey(15) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("r"):
            for n in NAMES:
                enc[n]["revs_off"] = enc[n]["revs"]

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
