#!/usr/bin/env python3
"""Replay a recorded plan run: robot camera | outside camera, above a dashboard.

    mcp_server/.venv/bin/python tools/plan_replay.py recordings/plan_<time>/ PHONE.mp4
        [--offset S] [--out replay.mp4] [--pad 2] [--fps 30] [--sync-only]

Reads what the plan page saved with "record this run" + "with camera video":
  plan_<time>.csv  telemetry (ENC / IMU / TOF / MARK rows, time_s from the run start)
  frames.csv       one row per robot-camera frame: time_s and its index in video.avi
  video.avi        the robot camera, tags outlined, time burned in
  plan.json        the steps and the repeat count

Sync: --offset is where the run's time_s = 0 falls in the outside video, in seconds.
Without it, the offset is found by sliding the robot's wheel speed (|left| + |right|,
from the CSV) along the loudness of the outside video's sound track (the motors) and
taking the best correlation; the motion in its picture is checked as a second opinion.
The two clocks have been seen to agree to 0.01 s over a 4 min run, so one offset is
enough. Use --sync-only to print the offset without rendering.

The dashboard is rebuilt from the CSV in the operator page's style: plan (lap, step,
result), wheel speeds, encoders, IMU (horizon, compass), front ToF, a dead-reckoned
map (encoder distance + IMU yaw; it drifts, it is not ground truth) and a timeline.
The plan's own PWM commands are not in the recording, so speeds are measured, not sent.

Heavy parts run gently: ffmpeg is niced and thread-capped, and encoding goes to the
GPU (h264_nvenc) when there is one.
"""

import argparse
import bisect
import csv
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

NICE = ["nice", "-n", "10"]
THREADS = "4"

# operator page, dark theme
BG, CARD, FG, MUT, LINE = (22, 22, 21), (33, 33, 32), (236, 235, 231), (154, 153, 147), (54, 53, 49)
GOOD, BAD = (88, 165, 104), (226, 80, 75)
BLUE, ORANGE = (58, 134, 212), (212, 144, 42)
STEP_COL = {"forward": (88, 165, 104), "back": (226, 80, 75), "turn": (212, 144, 42),
            "aim": (150, 110, 220), "find": (150, 110, 220), "toward": (58, 134, 212),
            "wait": (110, 110, 105)}

GHOST_MM = (75, 95)      # the faceplate's fake return (~85 mm): grey, not an alarm
TOP_H = 540
ROBOT_W = 720            # 640x480 -> 720x540
PHONE_W = 960            # 16:9 -> 960x540
W = ROBOT_W + PHONE_W    # 1680
DASH_H = 500
H = TOP_H + DASH_H       # 1040


def font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    for d in ("/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/dejavu"):
        p = Path(d) / name
        if p.exists():
            return ImageFont.truetype(str(p), size)
    return ImageFont.load_default()


F_H = font(19, True)
F_V = font(20)
F_S = font(16)
F_BIG = font(26, True)


# ---------------------------------------------------------------- recording

class Run:
    def __init__(self, d):
        d = Path(d)
        self.dir = d
        self.info = json.loads((d / "plan.json").read_text())
        csvs = [p for p in d.glob("plan_*.csv")]
        if not csvs:
            sys.exit("no plan_*.csv in %s" % d)
        t, dl, dr, sl, sr, yaw, pitch, roll, rng = ([] for _ in range(9))
        self.marks = []
        with open(csvs[0]) as f:
            for r in csv.DictReader(f):
                ts = float(r["time_s"])
                if r["source"] == "MARK":
                    self.marks.append((ts, r["note"]))
                    continue
                if r["source"] not in ("ENC", "IMU"):
                    continue
                t.append(ts)
                dl.append(float(r["dist_l_in"]))
                dr.append(float(r["dist_r_in"]))
                sl.append(float(r["speed_l_in_s"]))
                sr.append(float(r["speed_r_in_s"]))
                yaw.append(float(r["yaw_deg"]))
                pitch.append(float(r["pitch_deg"]))
                roll.append(float(r["roll_deg"]))
                rng.append(float(r["range_closest_mm"]))
        A = np.array
        self.t, self.dl, self.dr = A(t), A(dl) - dl[0], A(dr) - dr[0]
        self.sl, self.sr, self.yaw = A(sl), A(sr), A(yaw)
        self.pitch, self.roll, self.rng = A(pitch), A(roll), A(rng)
        self.end = float(self.t[-1])
        self._tl = self.t.tolist()
        # dead-reckoned path: encoder distance along IMU heading (left turn = CCW)
        ds = np.diff((self.dl + self.dr) / 2.0, prepend=0.0)
        h = np.radians(-(self.yaw - self.yaw[0]))
        self.x, self.y = np.cumsum(ds * np.cos(h)), np.cumsum(ds * np.sin(h))
        self.heading = h
        self._steps()
        self._frames()

    def _steps(self):
        self.steps = []          # (t0, t1, lap, n, cmd, text, result)
        open_ = {}
        for ts, note in self.marks:
            m = re.match(r"lap (\d+) step (\d+) start: (.*)", note)
            if m:
                open_ = dict(t0=ts, lap=int(m[1]), n=int(m[2]), text=m[3], cmd=m[3].split()[0])
                continue
            m = re.match(r"lap (\d+) step (\d+) (done|failed): (.*)", note)
            if m and open_:
                self.steps.append((open_["t0"], ts, open_["lap"], open_["n"], open_["cmd"],
                                   open_["text"], ("" if m[3] == "done" else "FAILED: ") + m[4]))
                open_ = {}
        self.step_t0 = [s[0] for s in self.steps]
        self.laps = self.info.get("repeat", 1)
        self.n_steps = len(self.info.get("steps", [])) or max((s[3] for s in self.steps), default=1)

    def _frames(self):
        self.fr_t, self.fr_v = [], []
        p = self.dir / "frames.csv"
        if not p.exists():
            return
        with open(p) as f:
            for r in csv.DictReader(f):
                if r.get("video_frame", "") == "":
                    continue
                self.fr_t.append(float(r["time_s"]))
                self.fr_v.append(int(r["video_frame"]))

    def at(self, ts):
        """Index of the last telemetry row at or before ts."""
        return max(0, min(len(self._tl) - 1, bisect.bisect_right(self._tl, ts) - 1))

    def step_at(self, ts):
        i = bisect.bisect_right(self.step_t0, ts) - 1
        if i < 0:
            return None, None
        s = self.steps[i]
        return s, (ts <= s[1])


# ---------------------------------------------------------------- sync

def remux_fixed(src, tmp):
    """Copy the phone video with sane colour tags (some phones write 'reserved', which
    ffmpeg's scaler refuses). Stream copy: seconds, no re-encode."""
    out = Path(tmp) / "outside.mp4"
    r = subprocess.run(NICE + ["ffmpeg", "-y", "-v", "error", "-i", str(src), "-c", "copy",
                               "-bsf:v", "h264_metadata=colour_primaries=1:transfer_characteristics=1:"
                               "matrix_coefficients=1", str(out)])
    if r.returncode != 0:      # not h264, or nothing wrong with it: use it as is
        return Path(src)
    return out


def probe(path):
    r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                        "stream=width,height:format=duration", "-of", "json", str(path)],
                       capture_output=True, text=True, check=True)
    j = json.loads(r.stdout)
    s = j["streams"][0]
    return s["width"], s["height"], float(j["format"]["duration"])


def find_offset(run, outside, dur, fs=100):
    z = lambda v: (v - v.mean()) / (v.std() + 1e-9)
    grid = np.arange(0, run.end, 1 / fs)
    rob = z(np.interp(grid, run.t, np.abs(run.sl) + np.abs(run.sr)))
    raw = subprocess.run(NICE + ["ffmpeg", "-v", "error", "-threads", THREADS, "-i", str(outside),
                                 "-vn", "-ac", "1", "-ar", "8000", "-f", "s16le", "-"],
                         capture_output=True, check=True).stdout
    a = np.frombuffer(raw, np.int16).astype(np.float32)
    win = 8000 // fs
    aud = np.sqrt((a[: len(a) // win * win].reshape(-1, win) ** 2).mean(1))
    n = len(rob)
    if len(aud) < n:
        sys.exit("the outside video is shorter than the run")
    # normalized cross-correlation by FFT, all lags at once
    m = len(aud)
    L = 1 << int(math.ceil(math.log2(m + n)))
    corr = np.fft.irfft(np.fft.rfft(aud, L) * np.conj(np.fft.rfft(rob, L)), L)[: m - n + 1]
    c1 = np.cumsum(np.r_[0, aud])
    c2 = np.cumsum(np.r_[0, aud ** 2])
    mean = (c1[n:] - c1[:-n]) / n
    var = (c2[n:] - c2[:-n]) / n - mean ** 2
    r = corr / (n * np.sqrt(np.maximum(var, 1e-9)))
    i = int(np.argmax(r))
    far = np.abs(np.arange(len(r)) - i) > 2 * fs
    return i / fs, float(r[i]), float(r[far].max()) if far.any() else float("nan")


# ---------------------------------------------------------------- drawing

def text(d, xy, s, f=F_V, fill=FG, anchor="la"):
    d.text(xy, s, font=f, fill=fill, anchor=anchor)


def cbar(d, x, y, w, h, v, full, col):
    d.rectangle([x, y, x + w, y + h], fill=LINE)
    mid = x + w / 2
    d.line([mid, y, mid, y + h], fill=MUT)
    f = max(-1, min(1, v / full)) if full else 0
    px = w / 2 * f
    if abs(px) >= 1:
        d.rectangle([min(mid, mid + px), y, max(mid, mid + px), y + h], fill=col)


def spark(d, x, y, w, h, ts, vs, t0, t1, lo, hi, col):
    if len(ts) < 2:
        return
    span = max(hi - lo, 1e-6)
    pts = [(x + w * (t - t0) / (t1 - t0), y + h - h * (min(max(v, lo), hi) - lo) / span)
           for t, v in zip(ts, vs) if not math.isnan(v)]
    if len(pts) >= 2:
        d.line(pts, fill=col, width=2)


def card(d, x, y, w, h, title, sub=""):
    d.rounded_rectangle([x, y, x + w, y + h], 10, fill=CARD, outline=LINE)
    text(d, (x + 12, y + 10), title, F_H, MUT)
    if sub:
        text(d, (x + 12 + d.textlength(title, font=F_H) + 12, y + 12), sub, F_S, MUT)


class Dash:
    """The dashboard for one moment. Static parts are drawn once."""

    def __init__(self, run, offset):
        self.run, self.offset = run, offset
        self.map_box = (1272, 12, 396, 392)
        x0, y0, mw, mh = self.map_box
        xs, ys = run.x, run.y
        pad = 8
        sx = (mw - 40) / max(xs.max() - xs.min() + 2 * pad, 1)
        sy = (mh - 60) / max(ys.max() - ys.min() + 2 * pad, 1)
        self.k = min(sx, sy)
        self.cx = (xs.max() + xs.min()) / 2
        self.cy = (ys.max() + ys.min()) / 2
        self.mx, self.my = x0 + mw / 2, y0 + 30 + (mh - 40) / 2
        step = max(1, len(xs) // 3000)
        self.path = [self.m(px, py) for px, py in zip(xs[::step], ys[::step])]
        self._pt = run.t[::step].tolist()
        self.base = self._static()

    def m(self, px, py):     # world inches -> canvas, y up
        return (self.mx + (px - self.cx) * self.k, self.my - (py - self.cy) * self.k)

    def _static(self):
        im = Image.new("RGB", (W, DASH_H), BG)
        d = ImageDraw.Draw(im)
        card(d, 12, 12, 400, 392, "PLAN")
        card(d, 424, 12, 410, 392, "WHEELS", "measured, in/s")
        card(d, 846, 12, 414, 392, "IMU + RANGE")
        x0, y0, mw, mh = self.map_box
        card(d, x0, y0, mw, mh, "MAP", "dead reckoned (drifts)")
        d.line(self.path, fill=LINE, width=2)
        # timeline
        r = self.run
        tx, ty, tw = 90, 420, W - 190
        self.tl = (tx, ty, tw)
        text(d, (12, ty + 2), "timeline", F_S, MUT)
        for s in r.steps:
            a, b = tx + tw * s[0] / r.end, tx + tw * s[1] / r.end
            d.rectangle([a, ty, max(a + 1, b), ty + 22], fill=STEP_COL.get(s[4], MUT))
        for s in r.steps:
            if s[3] == 1:
                a = tx + tw * s[0] / r.end
                d.line([a, ty - 4, a, ty + 26], fill=FG, width=1)
                text(d, (a + 2, ty + 28), "L%d" % s[2], F_S, MUT)
        x = tx
        for cmd in ("back", "turn", "forward", "toward", "find", "aim", "wait"):
            if any(s[4] == cmd for s in r.steps):
                d.rectangle([x, ty + 50, x + 14, ty + 64], fill=STEP_COL[cmd])
                text(d, (x + 20, ty + 48), cmd, F_S, MUT)
                x += 30 + d.textlength(cmd, font=F_S)
        return im

    def draw(self, ts):
        r = self.run
        im = self.base.copy()
        d = ImageDraw.Draw(im)
        i = r.at(ts)
        live = 0 <= ts <= r.end
        # PLAN
        s, running = r.step_at(ts)
        laps = r.laps
        if ts < 0:
            text(d, (24, 50), "starting in %.1f s" % -ts, F_BIG)
        elif s is None:
            text(d, (24, 50), "starting", F_BIG)
        else:
            text(d, (24, 50), "lap %d of %s" % (s[2], laps if laps else "∞"), F_BIG)
            text(d, (24, 92), "step %d of %d" % (s[3], r.n_steps), F_V, MUT)
            col = STEP_COL.get(s[4], FG)
            d.rectangle([24, 130, 34, 158], fill=col)
            words, line, y = s[5].split(), "", 128
            for wd in words:
                if d.textlength(line + " " + wd, font=F_V) > 350:
                    text(d, (44, y), line.strip())
                    line, y = "", y + 26
                line += " " + wd
            text(d, (44, y), line.strip())
            y += 40
            if running:
                text(d, (24, y), "running  %.1f s" % (ts - s[0]), F_V, GOOD)
            else:
                text(d, (24, y), "→ " + s[6][:40], F_V, BAD if s[6].startswith("FAILED") else GOOD)
        text(d, (24, 330), "run  t = %7.2f s" % ts, F_V, MUT)
        text(d, (24, 360), "outside video  %7.2f s" % (ts + self.offset), F_S, MUT)
        # WHEELS
        x0 = 436
        sl, sr = (r.sl[i], r.sr[i]) if live else (0.0, 0.0)
        text(d, (x0, 52), "L", F_V, BLUE)
        cbar(d, x0 + 24, 54, 280, 22, sl, 25, BLUE)
        text(d, (x0 + 314, 52), "%+5.1f" % sl)
        text(d, (x0, 86), "R", F_V, ORANGE)
        cbar(d, x0 + 24, 88, 280, 22, sr, 25, ORANGE)
        text(d, (x0 + 314, 86), "%+5.1f" % sr)
        t0 = max(0.0, ts - 15)
        j0 = r.at(t0)
        j1 = i + 1 if live else (0 if ts < 0 else len(r.t))
        sl_ = slice(j0, j1, max(1, (j1 - j0) // 300))
        d.rectangle([x0, 122, x0 + 386, 212], outline=LINE)
        spark(d, x0, 122, 386, 90, r.t[sl_], r.sl[sl_], t0, t0 + 15, -25, 25, BLUE)
        spark(d, x0, 122, 386, 90, r.t[sl_], r.sr[sl_], t0, t0 + 15, -25, 25, ORANGE)
        text(d, (x0, 216), "last 15 s", F_S, MUT)
        text(d, (x0, 250), "ENCODERS", F_H, MUT)
        dl, dr = (r.dl[i], r.dr[i]) if ts >= 0 else (0.0, 0.0)
        text(d, (x0, 282), "L %8.1f in" % dl)
        text(d, (x0 + 200, 282), "R %8.1f in" % dr)
        text(d, (x0, 312), "since run start", F_S, MUT)
        # IMU + RANGE
        x0 = 858
        yaw, pitch, roll = (r.yaw[i], r.pitch[i], r.roll[i]) if ts >= 0 else (r.yaw[0], r.pitch[0], r.roll[0])
        cx, cy, rr = x0 + 56, 110, 46
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=LINE)
        # horizon: tilt by roll, shift by pitch
        off = max(-rr, min(rr, pitch * 2))
        a = math.radians(roll)
        dx, dy = math.cos(a) * rr, math.sin(a) * rr
        d.line([cx - dx, cy + off - dy, cx + dx, cy + off + dy], fill=GOOD, width=3)
        text(d, (cx, cy + rr + 8), "horizon", F_S, MUT, "ma")
        cx2 = x0 + 176
        d.ellipse([cx2 - rr, cy - rr, cx2 + rr, cy + rr], outline=MUT, width=2)
        hd = math.radians(-(yaw - r.yaw[0]))       # map heading, CCW
        d.line([cx2, cy, cx2 + math.cos(hd) * (rr - 6), cy - math.sin(hd) * (rr - 6)], fill=ORANGE, width=4)
        text(d, (cx2, cy + rr + 8), "heading", F_S, MUT, "ma")
        text(d, (x0 + 250, 70), "yaw   %7.1f°" % yaw, F_S)
        text(d, (x0 + 250, 96), "pitch %7.1f°" % pitch, F_S)
        text(d, (x0 + 250, 122), "roll  %7.1f°" % roll, F_S)
        text(d, (x0 + 250, 148), "turned %+6.0f°" % -(yaw - r.yaw[0]), F_S, MUT)
        text(d, (x0, 200), "FRONT TOF", F_H, MUT)
        mm = r.rng[i] if ts >= 0 else float("nan")
        if math.isnan(mm):
            text(d, (x0, 232), "no reading", F_V, MUT)
        else:
            ghost = GHOST_MM[0] <= mm <= GHOST_MM[1]
            text(d, (x0, 228), "%4.0f mm" % mm, F_BIG, MUT if ghost else BAD if mm < 150 else FG)
            if ghost:
                text(d, (x0, 262), "faceplate?", F_S, MUT)
            d.rectangle([x0 + 150, 236, x0 + 386, 252], fill=LINE)
            d.rectangle([x0 + 150, 236, x0 + 150 + 236 * min(mm, 2000) / 2000, 252], fill=BLUE)
        d.rectangle([x0, 276, x0 + 386, 366], outline=LINE)
        spark(d, x0, 276, 386, 90, r.t[sl_], r.rng[sl_], t0, t0 + 15, 0, 2000, BLUE)
        text(d, (x0, 370), "last 15 s, 0–2000 mm", F_S, MUT)
        # MAP
        n = bisect.bisect_right(self._pt, ts)
        if n >= 2:
            d.line(self.path[max(0, n - 2000):n], fill=FG, width=2)
        if ts >= 0:
            px, py = self.m(r.x[i], r.y[i])
            h = r.heading[i]
            pts = [(px + math.cos(h) * 13, py - math.sin(h) * 13),
                   (px + math.cos(h + 2.5) * 9, py - math.sin(h + 2.5) * 9),
                   (px + math.cos(h - 2.5) * 9, py - math.sin(h - 2.5) * 9)]
            d.polygon(pts, fill=ORANGE)
        # timeline cursor
        tx, ty, tw = self.tl
        cx = tx + tw * min(max(ts, 0), r.end) / r.end
        d.line([cx, ty - 8, cx, ty + 30], fill=FG, width=3)
        return np.asarray(im)[:, :, ::-1]     # RGB -> BGR


# ---------------------------------------------------------------- render

def encoder_args():
    r = subprocess.run(["ffmpeg", "-hide_banner", "-encoders"], capture_output=True, text=True)
    if "h264_nvenc" in r.stdout:
        probe_ = subprocess.run(["ffmpeg", "-v", "error", "-f", "lavfi", "-i", "color=s=256x256:d=0.1",
                                 "-c:v", "h264_nvenc", "-f", "null", "-"], capture_output=True)
        if probe_.returncode == 0:
            return ["-c:v", "h264_nvenc", "-preset", "p5", "-cq", "23"]
    return ["-c:v", "libx264", "-preset", "veryfast", "-crf", "21", "-threads", THREADS]


def still_frame(run, outside, offset, ts, out):
    raw = subprocess.run(NICE + ["ffmpeg", "-v", "error", "-ss", "%.3f" % max(ts + offset, 0),
                                 "-i", str(outside), "-frames:v", "1", "-vf",
                                 "scale=%d:%d:force_original_aspect_ratio=decrease,"
                                 "pad=%d:%d:(ow-iw)/2:(oh-ih)/2" % (PHONE_W, TOP_H, PHONE_W, TOP_H),
                                 "-f", "rawvideo", "-pix_fmt", "bgr24", "-"],
                         capture_output=True, check=True).stdout
    canvas = np.zeros((H, W, 3), np.uint8)
    canvas[:TOP_H, ROBOT_W:] = np.frombuffer(raw, np.uint8).reshape(TOP_H, PHONE_W, 3)
    j = bisect.bisect_right(run.fr_t, ts) - 1
    if j >= 0:
        cap = cv2.VideoCapture(str(run.dir / "video.avi"))
        cap.set(cv2.CAP_PROP_POS_FRAMES, run.fr_v[j])
        ok, img = cap.read()
        if ok:
            canvas[:TOP_H, :ROBOT_W] = cv2.resize(img, (ROBOT_W, TOP_H))
    canvas[TOP_H:] = Dash(run, offset).draw(ts)
    label(canvas, "robot camera", 0)
    label(canvas, "outside (synced %+.2f s)" % offset, ROBOT_W)
    cv2.imwrite(str(out), canvas)


def label(canvas, s, x):
    """White text on a black box in the top-left corner of a view, like the burned-in stamp."""
    (tw, th), base = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(canvas, (x, 0), (x + tw + 16, th + base + 12), (0, 0, 0), -1)
    cv2.putText(canvas, s, (x + 8, th + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)


def render(run, outside, offset, out, pad, fps, still=None):
    if still is not None:
        return still_frame(run, outside, offset, still, out)
    t_start, t_end = -pad, run.end + pad
    dur = t_end - t_start
    ow, oh, odur = probe(outside)
    ss = offset + t_start
    pre = max(0.0, -ss)            # outside video starts after the run start: black first
    ph_cmd = NICE + ["ffmpeg", "-v", "error", "-threads", THREADS, "-ss", "%.3f" % max(ss, 0),
                     "-i", str(outside), "-t", "%.3f" % (dur - pre),
                     "-vf", "fps=%d,scale=%d:%d:force_original_aspect_ratio=decrease,"
                     "pad=%d:%d:(ow-iw)/2:(oh-ih)/2" % (fps, PHONE_W, TOP_H, PHONE_W, TOP_H),
                     "-f", "rawvideo", "-pix_fmt", "bgr24", "-"]
    phone = subprocess.Popen(ph_cmd, stdout=subprocess.PIPE)
    enc_cmd = NICE + ["ffmpeg", "-y", "-v", "error",
                      "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", "%dx%d" % (W, H), "-r", str(fps),
                      "-i", "-",
                      "-ss", "%.3f" % max(ss, 0), "-t", "%.3f" % (dur - pre), "-i", str(outside),
                      "-map", "0:v", "-map", "1:a?"] + (
                     ["-af", "adelay=%d:all=1" % int(pre * 1000)] if pre > 0 else []) + \
        encoder_args() + ["-pix_fmt", "yuv420p", "-c:a", "aac", "-b:a", "128k",
                          "-shortest", "-movflags", "+faststart", str(out)]
    enc = subprocess.Popen(enc_cmd, stdin=subprocess.PIPE)
    cap = cv2.VideoCapture(str(run.dir / "video.avi"))
    cur_idx, cur_img = -1, None
    dash = Dash(run, offset)
    frame_bytes = PHONE_W * TOP_H * 3
    blank_phone = np.zeros((TOP_H, PHONE_W, 3), np.uint8)
    blank_robot = np.zeros((TOP_H, ROBOT_W, 3), np.uint8)
    n_out = int(round(dur * fps))
    canvas = np.zeros((H, W, 3), np.uint8)
    try:
        for k in range(n_out):
            ts = t_start + k / fps
            # outside
            if ts + offset < 0 or k < int(pre * fps):
                ph = blank_phone
            else:
                buf = phone.stdout.read(frame_bytes)
                ph = np.frombuffer(buf, np.uint8).reshape(TOP_H, PHONE_W, 3) if len(buf) == frame_bytes else blank_phone
            # robot camera: newest frame that had arrived by ts
            j = bisect.bisect_right(run.fr_t, ts) - 1
            if j >= 0:
                want = run.fr_v[j]
                while cur_idx < want:
                    ok, img = cap.read()
                    if not ok:
                        break
                    cur_idx += 1
                    if cur_idx == want:
                        cur_img = cv2.resize(img, (ROBOT_W, TOP_H), interpolation=cv2.INTER_LINEAR)
            rb = cur_img if (cur_img is not None and j >= 0) else blank_robot
            canvas[:TOP_H, :ROBOT_W] = rb
            canvas[:TOP_H, ROBOT_W:] = ph
            canvas[TOP_H:] = dash.draw(ts)
            label(canvas, "robot camera", 0)
            label(canvas, "outside (synced %+.2f s)" % offset, ROBOT_W)
            enc.stdin.write(canvas.tobytes())
            if k % (fps * 10) == 0:
                print("  %5.1f / %.1f s" % (ts - t_start, dur), flush=True)
    finally:
        enc.stdin.close()
        enc.wait()
        phone.stdout.close()
        phone.kill()
        cap.release()
    if enc.returncode != 0:
        sys.exit("encoder failed")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("outside_video")
    ap.add_argument("--offset", type=float, help="run t=0 in the outside video, s (default: find it)")
    ap.add_argument("--out", help="default: <run_dir>/replay.mp4")
    ap.add_argument("--pad", type=float, default=2.0, help="seconds before and after the run")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--sync-only", action="store_true")
    ap.add_argument("--still", type=float, help="write one frame at this run time to a PNG and stop")
    a = ap.parse_args()
    run = Run(a.run_dir)
    with tempfile.TemporaryDirectory(prefix="plan_replay_") as tmp:
        outside = remux_fixed(a.outside_video, tmp)
        _, _, odur = probe(outside)
        if a.offset is None:
            off, r, r2 = find_offset(run, outside, odur)
            print("sync: run t=0 is %.2f s into the outside video (r=%.3f, next best %.3f)" % (off, r, r2))
            if r - r2 < 0.1:
                print("  weak match: check it, or pass --offset")
        else:
            off = a.offset
        if a.sync_only:
            return
        if a.still is not None:
            out = Path(a.out) if a.out else run.dir / ("replay_%.1f.png" % a.still)
            render(run, outside, off, out, 0, 1, still=a.still)
            print("wrote", out)
            return
        out = Path(a.out) if a.out else run.dir / "replay.mp4"
        print("rendering %.0f s to %s" % (run.end + 2 * a.pad, out))
        render(run, outside, off, out, a.pad, a.fps)
        print("done:", out)


if __name__ == "__main__":
    main()
