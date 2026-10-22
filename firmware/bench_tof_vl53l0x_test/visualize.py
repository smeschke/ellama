#!/usr/bin/env python3
"""Live OpenCV view of bench_tof_vl53l0x_test.ino serial output.

Shows a distance bar per VL53L0X (0 at the sensor, 2 m at the far end), the
reading in mm and inches, the RangeStatus the sensor reported, the return
signal and ambient IR rates, and a rolling distance plot for both sensors.
Wave your hand in front of a sensor and check the matching bar moves.

The sketch fires one laser at a time with dead air between them, so each
sensor reports on its own line and arrives at its own moment -- A first, then
B about a gap plus a timing budget later. This view therefore tracks the two
sensors independently rather than as a pair, and shows each one's age so a
sensor that has stopped reporting is obvious.

The gap figure the sketch measures (real quiet time between one laser
stopping and the next firing) is displayed per sensor, so you can confirm the
staggered schedule is actually being honoured.

The raw millimetre value is shown even when the status says the reading is
bad -- dimmed, with the status spelled out. A sensor can report a strong
return signal and still refuse to range, and seeing both numbers at once is
the fastest way to tell "nothing in front of it" from "it sees something but
won't trust the distance".

Usage: python visualize.py [/dev/ttyACM0] [baud]
Needs: pip install pyserial opencv-python numpy
Keys: q/ESC quit, r clear the history plot.

Opening the port resets the ESP32, so the boot-time bring-up lines (I2C scan,
address assignment) arrive right after this script connects.
"""
import collections
import re
import sys
import time

import cv2
import numpy as np
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

# "A mm=812 in=32.0 st=0 sig=3.21 amb=1.05 t=48213 gap=50 dur=35 err=0 ok=1"
NAMES = ("A", "B")
LINE_RE = re.compile(
    r"^(?P<name>[A-Z]) mm=(?P<mm>-?\d+) in=(?P<in>-?[\d.]+) st=(?P<st>\d+)"
    r" sig=(?P<sig>[\d.]+) amb=(?P<amb>[\d.]+) t=(?P<t>\d+) gap=(?P<gap>\d+)"
    r" dur=(?P<dur>\d+) err=(?P<err>-?\d+) ok=(?P<ok>[01])$"
)

# VL53L0X PAL range status. Only 0 is a measurement you can use.
STATUS_TEXT = {
    0: "valid",
    1: "sigma fail",
    2: "signal fail",
    3: "min range",
    4: "no return",
    5: "hardware fail",
}

W, H = 1000, 700
HIST = 300                  # a few minutes of history at this report rate
RANGE_MAX_MM = 2000         # the sensor's ceiling, and the bar's full scale
SIG_FULL_SCALE = 8.0        # MCps for a full signal/ambient meter
STALE_S = 2.0               # no line for this long = the sensor stopped reporting
COLORS = {"A": (255, 160, 60), "B": (80, 200, 255)}  # BGR, matching the encoder viewer
GREY = (110, 110, 110)
DIM = (70, 70, 70)
WHITE = (235, 235, 235)
GREEN = (90, 220, 120)
AMBER = (60, 190, 240)
RED = (80, 90, 230)

BAR_X0, BAR_X1 = 250, 950
BAR_Y = {"A": 160, "B": 330}
BAR_H = 54

tof = {
    n: dict(mm=0, inches=0.0, status=None, sig=0.0, amb=0.0, gap=0, t=0, ok=False,
            dur=0, err=0, last_rx=0.0, hist=collections.deque(maxlen=HIST))
    for n in NAMES
}
boot_lines = collections.deque(maxlen=6)


def parse(line):
    """Fill tof[] from one sensor's report line. Returns True if it was one."""
    m = LINE_RE.match(line)
    if not m or m.group("name") not in tof:
        return False
    s = tof[m.group("name")]
    s["mm"] = int(m.group("mm"))
    s["inches"] = float(m.group("in"))
    s["status"] = int(m.group("st"))
    s["sig"] = float(m.group("sig"))
    s["amb"] = float(m.group("amb"))
    s["t"] = int(m.group("t"))
    s["gap"] = int(m.group("gap"))
    s["dur"] = int(m.group("dur"))
    s["err"] = int(m.group("err"))
    s["ok"] = m.group("ok") == "1"
    s["last_rx"] = time.monotonic()
    # History holds the raw reading regardless of status, so the plot shows what the
    # sensor saw rather than hiding every rejected measurement.
    s["hist"].append(s["mm"])
    return True


def text(img, s, org, scale=0.5, color=WHITE, thick=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def draw_sensor(img, name):
    s = tof[name]
    y = BAR_Y[name]
    colour = COLORS[name]
    age = time.monotonic() - s["last_rx"] if s["last_rx"] else None
    stale = age is None or age > STALE_S

    text(img, f"sensor {name}", (30, y + 20), 0.7, colour, 2)
    if s["status"] is None:
        text(img, "waiting...", (30, y + 46), 0.5, GREY)
    else:
        st = STATUS_TEXT.get(s["status"], f"st{s['status']}")
        text(img, f"st{s['status']} {st}", (30, y + 46), 0.5, GREEN if s["ok"] else AMBER)
    if stale and s["status"] is not None:
        text(img, f"stale {age:.1f}s", (30, y + 68), 0.45, RED)
    elif not stale:
        # The sketch's own measurement of the quiet time before this laser fired, and how
        # long the measurement itself took -- a dur well under the timing budget means the
        # driver handed back stale data instead of ranging.
        text(img, f"gap {s['gap']} ms  dur {s['dur']} ms", (30, y + 68), 0.45, DIM)
        if s["err"] != 0:
            text(img, f"err {s['err']}", (30, y + 88), 0.45, RED)

    # Bar track, with a tick every 250 mm.
    cv2.rectangle(img, (BAR_X0, y), (BAR_X1, y + BAR_H), (45, 45, 45), -1)
    for mm in range(0, RANGE_MAX_MM + 1, 250):
        x = int(BAR_X0 + (BAR_X1 - BAR_X0) * mm / RANGE_MAX_MM)
        cv2.line(img, (x, y + BAR_H), (x, y + BAR_H + 6), GREY, 1)
        if mm % 500 == 0:
            text(img, f"{mm//10}cm", (x - 16, y + BAR_H + 22), 0.38, GREY)

    if s["status"] is None:
        text(img, "no data", (BAR_X0 + 12, y + 35), 0.6, GREY)
        return

    mm = max(0, min(s["mm"], RANGE_MAX_MM))
    fill = int((BAR_X1 - BAR_X0) * mm / RANGE_MAX_MM)
    # A rejected reading still gets drawn, just dimmed -- it is information, not noise.
    cv2.rectangle(img, (BAR_X0, y), (BAR_X0 + fill, y + BAR_H), colour if s["ok"] else DIM, -1)
    cv2.rectangle(img, (BAR_X0 + fill - 2, y), (BAR_X0 + fill + 2, y + BAR_H),
                  WHITE if s["ok"] else GREY, -1)

    label = f"{s['mm']:4d} mm   {s['inches']:5.1f} in"
    if not s["ok"]:
        label += "   (not trusted)"
    text(img, label, (BAR_X0 + 12, y - 10), 0.6, WHITE if s["ok"] else GREY)

    # Signal / ambient meters: the pair that explains a refused reading. Strong signal
    # with a bad status means the sensor saw light but rejected the distance; signal
    # near zero means nothing came back at all.
    for j, (key, lab) in enumerate((("sig", "signal"), ("amb", "ambient"))):
        mx0, my = BAR_X0, y + BAR_H + 34 + j * 18
        wfill = int(200 * min(s[key], SIG_FULL_SCALE) / SIG_FULL_SCALE)
        cv2.rectangle(img, (mx0, my), (mx0 + 200, my + 10), (45, 45, 45), -1)
        cv2.rectangle(img, (mx0, my), (mx0 + wfill, my + 10), colour, -1)
        text(img, f"{lab} {s[key]:5.2f} MCps", (mx0 + 212, my + 9), 0.4, GREY)


def draw_plot(img):
    x0, y0, w, h = 60, 520, W - 120, 140
    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (45, 45, 45), 1)
    text(img, f"distance history (0-{RANGE_MAX_MM} mm, newest right)", (x0, y0 - 8), 0.45, GREY)
    for name in NAMES:
        pts = list(tof[name]["hist"])
        if len(pts) < 2:
            continue
        step = w / max(len(pts) - 1, 1)
        poly = [(int(x0 + i * step),
                 int(y0 + h - h * min(mm, RANGE_MAX_MM) / RANGE_MAX_MM))
                for i, mm in enumerate(pts)]
        cv2.polylines(img, [np.array(poly, np.int32)], False, COLORS[name], 2, cv2.LINE_AA)
        text(img, name, (x0 + w + 8, y0 + 20 + 20 * NAMES.index(name)), 0.5, COLORS[name])


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.01)
    except serial.SerialException as e:
        sys.exit(f"Could not open {PORT}: {e}")

    buf = b""
    win = "VL53L0X bench"
    cv2.namedWindow(win)
    while True:
        buf += ser.read(4096)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            if not parse(line):
                # Boot chatter: I2C scan, address assignment, the legend. Worth showing,
                # since a sensor that failed to come up says so only here.
                boot_lines.append(line)

        img = np.zeros((H, W, 3), np.uint8)
        img[:] = (24, 24, 24)
        text(img, f"VL53L0X bench  --  {PORT} @ {BAUD}", (30, 40), 0.7, WHITE, 2)
        text(img, "one laser at a time, 50 ms apart", (30, 64), 0.45, GREY)
        text(img, "q/ESC quit   r clear plot", (W - 290, 40), 0.45, GREY)
        for name in NAMES:
            draw_sensor(img, name)
        draw_plot(img)
        for i, line in enumerate(boot_lines):
            text(img, line[:110], (30, H - 76 + i * 13), 0.35, DIM)

        cv2.imshow(win, img)
        k = cv2.waitKey(20) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("r"):
            for name in NAMES:
                tof[name]["hist"].clear()

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
