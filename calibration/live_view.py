#!/usr/bin/env python3
"""Live OpenCV dashboard for everything computer_bridge.ino relays.

One window showing all four streams at once, so you can drive with the stick and
watch the robot respond:

  DRIVE    the left/right PWM the bridge overheard from the stick controller
  ENCODERS per-wheel distance and speed, and the differential-drive heading
  IMU      pitch/roll/yaw as a horizon and compass, plus the raw accel axes
  RANGE    the VL53L0X distances, one bar per channel, by sensor name

The point of the layout is liveness. Every stream has its own packet rate and
age in the status strip, so "the sensor is dead" and "the sensor is reporting
nothing interesting" look completely different: a silent stream goes red and
says how long it has been quiet, while a sensor that is reporting but has
nothing in range says so in words. Nothing on screen is ever a fabricated zero.

READ-ONLY: this never writes to the serial port, so the bridge stays in
listen-only mode and will not fight the stick controller for the motor board.
Safe to leave running while you drive.

Usage: python live_view.py [--port /dev/ttyUSB0]
Needs: pip install pyserial opencv-python numpy
Keys:  q/ESC quit, z zero distance and yaw, r clear the history plots.

Opening the port resets the bridge ESP32, so expect a second of silence and its
boot banner right after this script connects.

The previous version of this file was a matplotlib viewer that also carried the
wheel-diameter and track-width calibration commands (d/t keys writing to
config.json). Those are not here -- this is a monitor, not a calibrator.
"""
import argparse
import collections
import math
import queue
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # for bridge/
from bridge import (BAUD_RATE, RESET_SETTLE_S, SENSOR_VALUE_INVALID, EncoderState,
                    OrientationState, SerialReader, TofState, load_config, parse_line,
                    pick_port)

W, H = 1280, 820
HIST = 400
RANGE_MAX_MM = 2000
ACCEL_FULL_G = 2.0
PWM_FULL = 255

# A stream quiet for longer than this is treated as dead. The slowest sensor here is
# the ToF board at ~6 Hz, so 1.5 s is several missed packets rather than one late one.
STALE_S = 1.5

# BGR, matching the per-sketch viewers in firmware/*/visualize.py so the whole project
# reads as one set of tools.
C_LEFT = (255, 160, 60)
C_RIGHT = (80, 200, 255)
C_THIRD = (120, 220, 140)
C_FOURTH = (200, 150, 255)
WHITE = (235, 235, 235)
GREY = (110, 110, 110)
DIM = (70, 70, 70)
GREEN = (90, 220, 120)
AMBER = (60, 190, 240)
RED = (80, 90, 230)
PANEL_BG = (34, 34, 34)
TRACK = (48, 48, 48)

cfg = load_config()
enc = EncoderState(cfg)
ori = OrientationState()
tof = TofState()

# Per-stream liveness. stamps feeds the rate figure; last_rx drives the age.
streams = {name: dict(stamps=collections.deque(maxlen=40), last_rx=0.0, count=0)
           for name in ("DRIVE", "ENC", "IMU", "RANGE")}

cmd = dict(left=0, right=0)
accel_g = (0.0, 0.0, 0.0)
hist = dict(dist_l=collections.deque(maxlen=HIST), dist_r=collections.deque(maxlen=HIST),
            yaw=collections.deque(maxlen=HIST))
tof_hist = {}            # (name, index) -> deque of mm or None
yaw_ref = 0.0
boot_lines = collections.deque(maxlen=4)


def note(stream):
    s = streams[stream]
    now = time.monotonic()
    s["stamps"].append(now)
    s["last_rx"] = now
    s["count"] += 1


def rate(stream):
    st = streams[stream]["stamps"]
    if len(st) < 2:
        return None
    span = st[-1] - st[0]
    return (len(st) - 1) / span if span > 0 else None


def age(stream):
    last = streams[stream]["last_rx"]
    return None if not last else time.monotonic() - last


def is_live(stream):
    a = age(stream)
    return a is not None and a <= STALE_S


# ---------------- drawing helpers ----------------

def text(img, s, org, scale=0.5, color=WHITE, thick=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def panel(img, x, y, w, h, title):
    cv2.rectangle(img, (x, y), (x + w, y + h), PANEL_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), (58, 58, 58), 1)
    text(img, title, (x + 14, y + 26), 0.6, WHITE, 2)
    return x + 14, y + 50


def centred_bar(img, x, y, w, h, value, full, colour, label):
    """Bar growing left or right from a centre line -- for signed values."""
    cv2.rectangle(img, (x, y), (x + w, y + h), TRACK, -1)
    mid = x + w // 2
    cv2.line(img, (mid, y), (mid, y + h), GREY, 1)
    frac = max(-1.0, min(1.0, value / full)) if full else 0.0
    px = int((w // 2) * frac)
    if px >= 0:
        cv2.rectangle(img, (mid, y), (mid + px, y + h), colour, -1)
    else:
        cv2.rectangle(img, (mid + px, y), (mid, y + h), colour, -1)
    text(img, label, (x + w + 12, y + h - 3), 0.45, GREY)


def level_bar(img, x, y, w, h, value, full, colour):
    """Bar growing from the left -- for unsigned values like a distance."""
    cv2.rectangle(img, (x, y), (x + w, y + h), TRACK, -1)
    frac = max(0.0, min(1.0, value / full)) if full else 0.0
    cv2.rectangle(img, (x, y), (x + int(w * frac), y + h), colour, -1)


def sparkline(img, x, y, w, h, values, colour, lo=None, hi=None):
    """Tiny line plot. None entries break the line, so gaps stay visible as gaps."""
    pts = [v for v in values if v is not None]
    if len(pts) < 2:
        return
    lo = min(pts) if lo is None else lo
    hi = max(pts) if hi is None else hi
    span = max(hi - lo, 1e-6)
    step = w / max(len(values) - 1, 1)
    run = []
    for i, v in enumerate(values):
        if v is None:
            if len(run) > 1:
                cv2.polylines(img, [np.array(run, np.int32)], False, colour, 1, cv2.LINE_AA)
            run = []
            continue
        run.append((int(x + i * step), int(y + h - h * (v - lo) / span)))
    if len(run) > 1:
        cv2.polylines(img, [np.array(run, np.int32)], False, colour, 1, cv2.LINE_AA)


def status_strip(img, y):
    """One chip per stream: rate, age, and whether it is live. The liveness display."""
    chip_w, gap = 300, 12
    for i, name in enumerate(("DRIVE", "ENC", "IMU", "RANGE")):
        x = 30 + i * (chip_w + gap)
        a, r = age(name), rate(name)
        if a is None:
            col, state = DIM, "never seen"
        elif a <= STALE_S:
            col, state = GREEN, f"{r:.0f} Hz" if r else "live"
        else:
            # The stick only transmits while you are driving, so a quiet DRIVE stream is
            # normal rather than a fault -- say so instead of raising an alarm.
            col = AMBER if name == "DRIVE" else RED
            state = f"idle {a:.0f}s" if name == "DRIVE" else f"SILENT {a:.0f}s"
        cv2.rectangle(img, (x, y), (x + chip_w, y + 38), PANEL_BG, -1)
        cv2.rectangle(img, (x, y), (x + 5, y + 38), col, -1)
        text(img, name, (x + 16, y + 25), 0.55, WHITE, 2)
        text(img, state, (x + 110, y + 25), 0.5, col)
        text(img, f"n={streams[name]['count']}", (x + 220, y + 25), 0.42, DIM)


# ---------------- panels ----------------

def draw_drive(img, x, y, w, h):
    px, py = panel(img, x, y, w, h, "DRIVE  (stick commands overheard)")
    if streams["DRIVE"]["count"] == 0:
        text(img, "no drive commands seen yet", (px, py + 30), 0.55, GREY)
        text(img, "power the stick controller and drive", (px, py + 56), 0.45, DIM)
        return
    idle = not is_live("DRIVE")
    for i, (name, value, colour) in enumerate((("left", cmd["left"], C_LEFT),
                                               ("right", cmd["right"], C_RIGHT))):
        by = py + 10 + i * 46
        text(img, name, (px, by + 20), 0.5, colour)
        centred_bar(img, px + 60, by, w - 170, 26,
                    0 if idle else value, PWM_FULL,
                    DIM if idle else colour, f"{value:+4d}")
    if idle:
        text(img, f"stick idle {age('DRIVE'):.0f}s -- bars zeroed, last command shown",
             (px, py + 112), 0.42, AMBER)


def draw_encoders(img, x, y, w, h):
    px, py = panel(img, x, y, w, h, "ENCODERS")
    if streams["ENC"]["count"] == 0:
        text(img, "no ENC packets -- is the encoder board powered?", (px, py + 30), 0.5, GREY)
        return
    for i, (name, dist, avail, colour) in enumerate(
            (("left", enc.dist_l, enc.available[0], C_LEFT),
             ("right", enc.dist_r, enc.available[1], C_RIGHT))):
        by = py + 6 + i * 44
        text(img, name, (px, by + 20), 0.5, colour)
        if not avail:
            # The encoders sit by the wheels and get damaged; a missing one must never
            # look like a wheel sitting still.
            text(img, "NOT RESPONDING", (px + 60, by + 20), 0.5, RED)
        else:
            text(img, f"{dist:+8.2f} in", (px + 60, by + 20), 0.55, WHITE)
            sparkline(img, px + 210, by, w - 250, 26,
                      list(hist["dist_l"] if i == 0 else hist["dist_r"]), colour)
    hdg = enc.heading_deg
    if hdg is None:
        text(img, "heading: needs both wheels", (px, py + 108), 0.45, AMBER)
    else:
        text(img, f"heading {hdg:+7.1f} deg", (px, py + 108), 0.5, WHITE)


def draw_imu(img, x, y, w, h):
    px, py = panel(img, x, y, w, h, "IMU  (GY-521)")
    if streams["IMU"]["count"] == 0:
        text(img, "no IMU packets -- is the IMU board powered?", (px, py + 30), 0.5, GREY)
        return

    # Artificial horizon: tilts with roll, slides with pitch.
    cx, cy, r = px + 80, py + 80, 68
    cv2.circle(img, (cx, cy), r, TRACK, -1)
    off = max(-r, min(r, ori.pitch * r / 60.0))
    a = math.radians(-ori.roll)
    dx, dy = math.cos(a), math.sin(a)
    mx, my = cx + dy * off, cy + dx * off
    cv2.line(img, (int(mx - dx * r), int(my - dy * r)),
             (int(mx + dx * r), int(my + dy * r)), WHITE, 2, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), r, GREY, 1)
    cv2.line(img, (cx - 24, cy), (cx - 7, cy), AMBER, 2)
    cv2.line(img, (cx + 7, cy), (cx + 24, cy), AMBER, 2)
    text(img, "pitch/roll", (cx - 34, cy + r + 18), 0.4, GREY)

    # Compass for yaw. Gyro-integrated, so it drifts -- 'z' re-zeros it.
    cx2 = px + 240
    yaw = ori.yaw - yaw_ref
    cv2.circle(img, (cx2, cy), r, TRACK, -1)
    cv2.circle(img, (cx2, cy), r, GREY, 1)
    ang = math.radians(yaw - 90)
    cv2.line(img, (cx2, cy),
             (int(cx2 + math.cos(ang) * (r - 10)), int(cy + math.sin(ang) * (r - 10))),
             C_THIRD, 3, cv2.LINE_AA)
    text(img, f"yaw {yaw:+.0f}", (cx2 - 30, cy + r + 18), 0.4, GREY)

    tx = px + 340
    text(img, f"pitch {ori.pitch:+7.1f}", (tx, py + 30), 0.5, C_LEFT)
    text(img, f"roll  {ori.roll:+7.1f}", (tx, py + 56), 0.5, C_RIGHT)
    text(img, f"yaw   {yaw:+7.1f}  (drifts)", (tx, py + 82), 0.5, C_THIRD)
    for i, (axis, g) in enumerate(zip("xyz", accel_g)):
        by = py + 106 + i * 24
        text(img, f"a{axis}", (tx, by + 12), 0.42, GREY)
        centred_bar(img, tx + 30, by, 150, 14, g, ACCEL_FULL_G, C_FOURTH, f"{g:+5.2f} g")


def draw_range(img, x, y, w, h):
    px, py = panel(img, x, y, w, h, "RANGE  (VL53L0X)")
    if not tof.channels:
        text(img, "no range packets -- is the ToF board powered?", (px, py + 30), 0.5, GREY)
        return
    for i, key in enumerate(sorted(tof.channels)):
        entry = tof.channels[key]
        by = py + 6 + i * 44
        colour = (C_LEFT, C_RIGHT, C_THIRD, C_FOURTH)[i % 4]
        text(img, TofState.label(*key), (px, by + 20), 0.45, colour)
        mm = entry["mm"]
        if mm is None:
            # Reporting, but nothing within 2 m. Different from a dead sensor.
            text(img, "no target in range", (px + 150, by + 20), 0.45, DIM)
        else:
            level_bar(img, px + 150, by, w - 330, 26, mm, RANGE_MAX_MM, colour)
            text(img, f"{mm:5d} mm  {mm/25.4:5.1f} in", (px + w - 170, by + 20), 0.45, WHITE)
        sparkline(img, px + 150, by + 28, w - 330, 12,
                  list(tof_hist.get(key, ())), colour, lo=0, hi=RANGE_MAX_MM)
    closest = tof.closest_mm()
    text(img, "closest (median-filtered): " +
         ("nothing in range" if closest is None else f"{closest} mm / {closest/25.4:.1f} in"),
         (px, py + h - 70), 0.5, WHITE if closest is not None else GREY)


# ---------------- main ----------------

def drain(reader):
    global accel_g
    while True:
        try:
            raw = reader.q.get_nowait()
        except queue.Empty:
            return
        kind, data = parse_line(raw)
        if kind == "ENC":
            enc.update(*data)
            note("ENC")
            hist["dist_l"].append(enc.dist_l if enc.available[0] else None)
            hist["dist_r"].append(enc.dist_r if enc.available[1] else None)
        elif kind == "IMU":
            agx, agy, agz, *_ = ori.update(*data)
            accel_g = (agx, agy, agz)
            note("IMU")
            hist["yaw"].append(ori.yaw - yaw_ref)
        elif kind == "CMD":
            cmd["left"], cmd["right"] = data
            note("DRIVE")
        elif kind == "SENSOR":
            name, values, ms = data
            if name.startswith("TOF"):
                for key in tof.update(name, values, ms):
                    h = tof_hist.setdefault(key, collections.deque(maxlen=HIST))
                    h.append(tof.channels[key]["mm"])
                note("RANGE")
        elif kind is None and data.strip():
            boot_lines.append(data.strip())


def main():
    global yaw_ref
    ap = argparse.ArgumentParser(description="Live dashboard for all bridge telemetry.")
    ap.add_argument("--port", help="bridge serial port (default: autodetect)")
    args = ap.parse_args()

    port = pick_port(args.port)
    # exclusive=True: refuse to attach to a port something else already owns, rather
    # than resetting the ESP32 and splitting the byte stream with it.
    ser = serial.Serial(port, BAUD_RATE, timeout=0.05, exclusive=True)
    print(f"Reading {port} at {BAUD_RATE}; waiting {RESET_SETTLE_S}s for the ESP32 to boot.")
    time.sleep(RESET_SETTLE_S)

    reader = SerialReader(ser)
    reader.start()
    win = "Llama robot -- live telemetry"
    cv2.namedWindow(win)
    try:
        while True:
            drain(reader)

            img = np.zeros((H, W, 3), np.uint8)
            img[:] = (24, 24, 24)
            text(img, "Llama robot -- live telemetry", (30, 40), 0.75, WHITE, 2)
            text(img, f"{port} @ {BAUD_RATE}   read-only, safe while driving",
                 (420, 40), 0.45, GREY)
            text(img, "q/ESC quit   z zero dist+yaw   r clear plots", (W - 400, 40), 0.45, GREY)

            status_strip(img, 62)

            draw_drive(img, 30, 120, 610, 160)
            draw_encoders(img, 30, 296, 610, 170)
            draw_imu(img, 660, 120, 590, 220)
            draw_range(img, 660, 356, 590, 300)

            for i, line in enumerate(boot_lines):
                text(img, line[:150], (30, H - 70 + i * 15), 0.35, DIM)

            cv2.imshow(win, img)
            k = cv2.waitKey(20) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("z"):
                enc.zero()
                ori.zero_yaw()
                yaw_ref = 0.0
            if k == ord("r"):
                for q in hist.values():
                    q.clear()
                for q in tof_hist.values():
                    q.clear()
    finally:
        reader.stop()
        ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
