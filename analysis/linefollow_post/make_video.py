"""Compose the post video: course map + what the AI is doing + what the robot sees.

    python3 analysis/linefollow_post/make_video.py                 # -> post_assets/linefollow_post.mp4
    python3 analysis/linefollow_post/make_video.py --preview 40    # one frame at t=40 s -> preview.png

Panels (1920x1080):
  left top     course map, built live from dead reckoning (left encoder + IMU yaw): the tape the
               robot has seen so far, its trail, the camera footprint, live detections in green,
               AI intervention pins
  left bottom  who is driving (script or AI), the verdict text, wheel efforts, the AI log, and
               the report's contact sheets at each stop
  right        the camera with the line segmentation (red = dark mask, green = line centre per
               band, orange = cross strip); during the AI's stops it shows the snapshots
  bottom bar   timeline, green = the script drives, amber = the AI is looking / acting

The story it tells is scripted from what happened on 2026-10-03 (two runs, stall, wrong-way
nudge, correction, real finish); positions and frames are real data.
"""

import argparse
import os
import subprocess
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(__file__))
import pipeline as pl  # noqa: E402
import course_map  # noqa: E402

W, H, FPS = 1920, 1080, 30
OUT = os.path.join(pl.REPO, "post_assets")
SNAP = os.path.join(OUT, "snapshots")

BG = (14, 18, 22)
PANEL = (24, 30, 36)
EDGE = (52, 62, 72)
TXT = (232, 236, 240)
DIM = (140, 150, 160)
GREEN = (88, 204, 126)
RED = (236, 84, 84)
AMBER = (255, 184, 48)
CYAN = (80, 200, 240)
TAPE = (206, 214, 220)

MAP_R = (24, 86, 1320, 726)       # x0, y0, x1, y1
AI_R = (24, 742, 1320, 1050)
CAM_R = (1344, 86, 1896, 1050)
BAR_R = (24, 1060, 1896, 1074)


def font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    for d in ("/usr/share/fonts/truetype/dejavu/", "/usr/share/fonts/dejavu/"):
        if os.path.exists(d + name):
            return ImageFont.truetype(d + name, size)
    return ImageFont.load_default()


F_TITLE, F_CHIP, F_BODY, F_SMALL, F_TINY = font(30, True), font(26, True), font(23), font(19), font(16)
F_BODY_B = font(23, True)


def img(name):
    bgr = cv2.imread(os.path.join(SNAP, name))
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def fit(src, w, h, bg=PANEL):
    """Letterbox `src` into a w x h box."""
    s = min(w / src.shape[1], h / src.shape[0])
    nw, nh = int(src.shape[1] * s), int(src.shape[0] * s)
    out = np.full((h, w, 3), bg, np.uint8)
    out[(h - nh) // 2:(h - nh) // 2 + nh, (w - nw) // 2:(w - nw) // 2 + nw] = cv2.resize(
        src, (nw, nh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_CUBIC)
    return out


def wrap(text, fnt, width):
    lines, cur = [], ""
    for word in text.split(" "):
        t = (cur + " " + word).strip()
        if fnt.getlength(t) <= width:
            cur = t
        else:
            lines.append(cur)
            cur = word
    lines.append(cur)
    return lines


def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


# ------------------------------------------------------------------------------ data

def load():
    runs = course_map.build()
    for r in runs:
        run = pl.load_run(r["name"])
        r["log"] = run["log"]
        r["vt"] = run["vt"].time_s.values
        raw, ann, prev = [], [], None
        for _, _, bgr in pl.read_frames(run):
            res, out = pl.line_vision.analyze(bgr, prev)
            prev = res.get("x_frac")
            raw.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            ann.append(cv2.cvtColor(out, cv2.COLOR_BGR2RGB))
        r["raw"], r["ann"] = raw, ann
        r["dur"] = float(r["vt"][-1])
    return runs


def build_phases(runs):
    r1, r2 = runs
    y1 = float(np.degrees(r1["yaw"][-1]))   # yaw at the stall
    y2 = y1 + 14.4                          # after the 12 deg left nudge (IMU: 14.4)
    y3 = float(np.degrees(r2["yaw"][0]))    # run 2 starts here (after the 22.3 deg right nudge)
    P = []

    def add(kind, dur, **kw):
        P.append(dict(kind=kind, dur=dur, **kw))

    add("preflight", 3.5, chip=("AI PRE-FLIGHT", AMBER), cam="00_start_line_view.jpg",
        cam_label="SNAPSHOT  line_view",
        text="Tape centered, 6 bands, no finish strip in view. Start the script.")
    add("run", r1["dur"], run=0, chip=("SCRIPT DRIVING", GREEN), cam_label="CAMERA  segmentation")
    add("stop", 2.0, run=0, chip=("SCRIPT STOPPED   stale", RED), cam_label="CAMERA  last frame",
        text="The script says: stalled. It can't tell why.", ai_log=None)
    add("inspect", 8.0, run=0, chip=("AI INSPECTING", AMBER), cam_label="SNAPSHOT  look() after the stop", sheets=("10_run1_stale_sheet_annotated.jpg",
                                                                       "11_run1_stale_sheet_raw.jpg"),
        stills=("12_run1_stale_still_annotated.jpg", "13_run1_stale_still_raw.jpg"),
        text="Verdict: a stall, not a lost line. The tape is still in view and the last log rows "
             "show a left spin at 60 with the aim stuck at -12 deg. The spin is too weak to break "
             "the wheels free.", log_at=3.0,
        log=("STALL", "stale: spin too weak, tape still in view"), count=True)
    add("nudge", 3.0, chip=("AI ACTION   nudge LEFT 12", AMBER), cam_label="CAMERA  no new frame",
        yaw=(y1, y2), text="Nudge left 12 deg (IMU measured 14.4).")
    add("check", 4.5, chip=("AI CHECK   line_view", AMBER), cam="20_after_left12_wrong_way.jpg",
        cam_label="SNAPSHOT  line_view", log_at=1.5,
        log=("WRONG WAY", "nudge left: tape moved 1.8 -> 4.2 in right"),
        text="Tape moved from 1.8 in to 4.2 in right of center. That was the wrong way. "
             "The check caught it.")
    add("nudge", 3.0, chip=("AI ACTION   nudge RIGHT 20", AMBER), cam_label="CAMERA  no new frame",
        yaw=(y2, y3), text="Nudge right 20 deg (IMU measured 22.3).")
    add("check", 3.5, chip=("AI CHECK   line_view", AMBER), cam="30_after_right20_centered.jpg",
        cam_label="SNAPSHOT  line_view", log_at=1.0,
        log=("RESTART", "nudge right: centered, 6 bands -> restart"),
        text="Tape centered, 6 bands, no cross strip. Restart the script.")
    add("run", r2["dur"], run=1, chip=("SCRIPT DRIVING   resumed", GREEN), cam_label="CAMERA  segmentation")
    add("stop", 2.0, run=1, chip=("SCRIPT STOPPED   finish_candidate", RED), cam_label="CAMERA  last frame",
        text="The script says: finish strip ahead. That could be paint or glare.")
    add("inspect", 9.0, run=1, chip=("AI INSPECTING", AMBER), cam_label="SNAPSHOT  look() after the stop", sheets=("40_run2_finish_sheet_annotated.jpg",
                                                                       "41_run2_finish_sheet_raw.jpg"),
        stills=("42_run2_finish_still_annotated.jpg", "43_run2_finish_still_raw.jpg"),
        text="Verdict: a real finish. The crossbar spans both sides of the stem and has sharp "
             "edges in the raw frames and the raw still. It is not glare or paint.", log_at=3.0,
        log=("REAL FINISH", "finish_candidate: crossbar both sides, sharp edges"), count=True)
    add("end", 5.0, run=1, chip=("COURSE COMPLETE", GREEN), cam="43_run2_finish_still_raw.jpg",
        cam_label="SNAPSHOT  raw still", text="")
    t0 = 0.0
    for p in P:
        p["t0"] = t0
        t0 += p["dur"]
    return P, t0


# ------------------------------------------------------------------------------ renderer

class Renderer:
    def __init__(self, runs):
        self.runs = runs
        self.phases, self.total = build_phases(runs)
        allx = np.concatenate([r["x"] for r in runs] + [[p[1] for p in r["tape"]] for r in runs])
        ally = np.concatenate([r["y"] for r in runs] + [[p[2] for p in r["tape"]] for r in runs])
        mw, mh = MAP_R[2] - MAP_R[0], MAP_R[3] - MAP_R[1]
        pad = 10.0
        self.xmin, self.xmax = allx.min() - pad, allx.max() + pad
        self.ymin, self.ymax = ally.min() - pad, ally.max() + pad
        self.S = min((mw - 40) / (self.xmax - self.xmin), (mh - 60) / (self.ymax - self.ymin))
        self.ox = (mw - self.S * (self.xmax - self.xmin)) / 2
        self.oy = (mh - self.S * (self.ymax - self.ymin)) / 2
        self.layer = np.zeros((mh, mw, 3), np.uint8)
        self.layer[:] = PANEL
        self._grid()
        self.cursor = [0, 0]
        self.last_pt = None
        self.last_run = None
        self.static = self._static()

    # --- map helpers
    def w2s(self, x, y):
        mh = MAP_R[3] - MAP_R[1]
        return (int(round(self.ox + (x - self.xmin) * self.S)),
                int(round(mh - self.oy - (y - self.ymin) * self.S)))

    def _grid(self):
        for gx in np.arange(np.ceil(self.xmin / 12) * 12, self.xmax, 12):
            a, b = self.w2s(gx, self.ymin), self.w2s(gx, self.ymax)
            cv2.line(self.layer, a, b, (34, 42, 50), 1)
        for gy in np.arange(np.ceil(self.ymin / 12) * 12, self.ymax, 12):
            a, b = self.w2s(self.xmin, gy), self.w2s(self.xmax, gy)
            cv2.line(self.layer, a, b, (34, 42, 50), 1)

    def _static(self):
        c = np.zeros((H, W, 3), np.uint8)
        c[:] = BG
        for r in (MAP_R, AI_R, CAM_R):
            cv2.rectangle(c, (r[0], r[1]), (r[2], r[3]), PANEL, -1)
            cv2.rectangle(c, (r[0], r[1]), (r[2], r[3]), EDGE, 1)
        return c

    def advance_map(self, ri, u):
        """Draw the tape and trail of run ri up to run-time u into the persistent layer."""
        r = self.runs[ri]
        tape = r["tape"]
        while self.cursor[ri] < len(tape) and tape[self.cursor[ri]][0] <= u:
            _, x, y, band = tape[self.cursor[ri]]
            self.cursor[ri] += 1
            if band <= 3:
                cv2.circle(self.layer, self.w2s(x, y), 2, TAPE, -1)
        px, py = np.interp(u, r["t"], r["x"]), np.interp(u, r["t"], r["y"])
        pt = self.w2s(px, py)
        if self.last_run == ri and self.last_pt is not None and pt != self.last_pt:
            cv2.line(self.layer, self.last_pt, pt, GREEN, 3, cv2.LINE_AA)
        self.last_pt, self.last_run = pt, ri

    def pose(self, p, u):
        if p["kind"] in ("run", "stop", "inspect", "end", "preflight"):
            ri = p.get("run", 0)
            r = self.runs[ri]
            uu = min(u, r["t"][-1]) if p["kind"] == "run" else r["t"][-1]
            if p["kind"] == "preflight":
                return r["x"][0], r["y"][0], r["yaw"][0]
            return (float(np.interp(uu, r["t"], r["x"])), float(np.interp(uu, r["t"], r["y"])),
                    float(np.interp(uu, r["t"], r["yaw"])))
        # nudge / check: in-place turns at the stall position, yaw animated or held
        x, y = float(self.runs[0]["x"][-1]), float(self.runs[0]["y"][-1])
        if p["kind"] == "nudge":
            a, b = p["yaw"]
            return x, y, np.radians(a + (b - a) * smooth(u / (p["dur"] * 0.7)))
        # check: hold the yaw of the previous nudge's end
        prev = [q for q in self.phases if q["t0"] < p["t0"] and q["kind"] == "nudge"][-1]
        return x, y, np.radians(prev["yaw"][1])

    def phase_start(self, kind, nth):
        return [q for q in self.phases if q["kind"] == kind][nth]["t0"]

    # --- drawing
    def draw_robot(self, canvas, x, y, yaw, color):
        ox, oy = MAP_R[0], MAP_R[1]
        h = np.array([-np.sin(yaw), np.cos(yaw)])
        r = np.array([np.cos(yaw), np.sin(yaw)])

        def P(fwd, lat):
            q = np.array([x, y]) + fwd * h + lat * r
            sx, sy = self.w2s(*q)
            return (sx + ox, sy + oy)

        cam = np.array([P(4, -6), P(4, 6), P(19, 6.5), P(19, -6.5)], np.int32)
        ov = canvas.copy()
        cv2.fillPoly(ov, [cam], CYAN)
        cv2.addWeighted(ov, 0.22, canvas, 0.78, 0, canvas)
        cv2.polylines(canvas, [cam], True, CYAN, 1, cv2.LINE_AA)
        body = np.array([P(-6, -4.5), P(-6, 4.5), P(5, 4.5), P(5, -4.5)], np.int32)
        cv2.fillPoly(canvas, [body], color)
        cv2.polylines(canvas, [body], True, (255, 255, 255), 2, cv2.LINE_AA)
        nose = np.array([P(5, -3), P(5, 3), P(9, 0)], np.int32)
        cv2.fillPoly(canvas, [nose], (255, 255, 255))

    def pin(self, canvas, x, y, color, label, dx=18, dy=-30):
        sx, sy = self.w2s(x, y)
        sx += MAP_R[0]
        sy += MAP_R[1]
        cv2.circle(canvas, (sx, sy), 9, color, -1, cv2.LINE_AA)
        cv2.circle(canvas, (sx, sy), 9, (255, 255, 255), 2, cv2.LINE_AA)
        return (sx + dx, sy + dy, label, color)

    def render(self, t):
        p = next(q for q in self.phases if q["t0"] <= t < q["t0"] + q["dur"]) if t < self.total \
            else self.phases[-1]
        u = t - p["t0"]
        canvas = self.static.copy()
        labels = []   # (x, y, text, font, color) drawn through PIL at the end

        # --- map
        if p["kind"] == "run":
            self.advance_map(p["run"], min(u, self.runs[p["run"]]["t"][-1]))
        elif p["kind"] != "preflight" and p["kind"] in ("stop", "inspect", "end"):
            self.advance_map(p["run"], self.runs[p["run"]]["t"][-1])
        mp = self.layer.copy()
        canvas[MAP_R[1]:MAP_R[3], MAP_R[0]:MAP_R[2]] = mp
        x, y, yaw = self.pose(p, u)
        # live detections of the current frame, projected with the camera-lag pose
        if p["kind"] == "run":
            r = self.runs[p["run"]]
            fi = min(max(int(np.searchsorted(r["vt"], u, side="right")) - 1, 0), len(r["raw"]) - 1)
            lag = max(u - pl.CAM_LAG_S, 0)
            pose_l = (np.interp(lag, r["t"], r["x"]), np.interp(lag, r["t"], r["y"]),
                      np.interp(lag, r["t"], r["yaw"]))
            for gx, gy, _ in pl.ground_points(r["det"][fi]["res"].get("bands") or [], pose_l):
                sx, sy = self.w2s(gx, gy)
                cv2.circle(canvas, (sx + MAP_R[0], sy + MAP_R[1]), 4, GREEN, -1, cv2.LINE_AA)
        stall = self.phase_start("stop", 0)
        fin = self.phase_start("stop", 1)
        pins = []
        sx0, sy0 = self.runs[0]["x"][0], self.runs[0]["y"][0]
        pins.append(self.pin(canvas, sx0, sy0, GREEN, "START", dx=14, dy=-8))
        if t >= stall:
            pins.append(self.pin(canvas, self.runs[0]["x"][-1], self.runs[0]["y"][-1], AMBER,
                                 "AI: stall, 2 nudges", dx=-250, dy=22))
        if t >= fin:
            pins.append(self.pin(canvas, self.runs[1]["x"][-1], self.runs[1]["y"][-1], AMBER,
                                 "AI: real finish", dx=-170, dy=-84))
        self.draw_robot(canvas, x, y, yaw, AMBER if p["kind"] in ("inspect", "nudge", "check") else GREEN)
        for sx, sy, label, col in pins:
            labels.append((sx, sy, label, F_SMALL, col))
        labels.append((MAP_R[0] + 14, MAP_R[1] + 8, "COURSE MAP   dead reckoned, tape drawn as seen", F_SMALL, DIM))
        labels.append((MAP_R[2] - 150, MAP_R[3] - 28, "grid = 1 ft", F_TINY, DIM))

        # --- camera panel
        cx0, cy0, cx1, cy1 = CAM_R
        cw, ch = cx1 - cx0 - 4, cy1 - cy0 - 44
        if "cam" in p:
            cam = fit(img(p["cam"]), cw, ch)
        elif p["kind"] in ("inspect",):
            a, b = p["stills"]
            cam = fit(img(a if (u % 6.0) < 3.0 else b), cw, ch)
            labels.append((cx0 + 14, cy1 - 34, "annotated still" if (u % 6.0) < 3.0 else "RAW still: no overlay",
                           F_SMALL, AMBER))
        else:
            r = self.runs[p.get("run", 0)]
            if p["kind"] == "run":
                fi = min(max(int(np.searchsorted(r["vt"], u, side="right")) - 1, 0), len(r["raw"]) - 1)
            else:
                fi = len(r["raw"]) - 1
            cam = fit(r["ann"][fi], cw, ch)
            if p["kind"] == "run":
                pip = cv2.resize(r["raw"][fi], (126, 224), interpolation=cv2.INTER_AREA)
                cam[ch - 232:ch - 8, cw - 134:cw - 8] = pip
                cv2.rectangle(cam, (cw - 134, ch - 232), (cw - 8, ch - 8), (255, 255, 255), 1)
            elif p["kind"] in ("stop", "nudge"):
                cam = (cam * 0.55).astype(np.uint8)
        canvas[cy0 + 40:cy0 + 40 + ch, cx0 + 2:cx0 + 2 + cw] = cam
        labels.append((cx0 + 14, cy0 + 8, p.get("cam_label", "CAMERA"), F_SMALL, DIM))

        # --- AI panel
        ax0, ay0, ax1, ay1 = AI_R
        chip, chip_col = p["chip"]
        labels.append(("chip", ax0 + 20, ay0 + 16, chip, chip_col))
        text = p.get("text", "")
        if text:
            n = int(max(0.0, u - 0.3) * 55)
            lines = wrap(text[:n], F_BODY, 640)
            for i, ln in enumerate(lines):
                labels.append((ax0 + 22, ay0 + 76 + i * 32, ln, F_BODY, TXT))
        if p["kind"] == "run":
            r = self.runs[p["run"]]
            row = r["log"].iloc[min(max(int(np.searchsorted(r["log"].time_s.values, u, side="right")) - 1, 0),
                                    len(r["log"]) - 1)]
            spin = float(np.nan_to_num(row.spin)) > 0
            e = row.e_near_in if row.e_near_in == row.e_near_in else float("nan")
            ph = row.phi_deg if row.phi_deg == row.phi_deg else float("nan")
            labels.append((ax0 + 22, ay0 + 84, f"mode  {'SPIN' if spin else 'DRIVE'}", F_BODY_B,
                           CYAN if spin else GREEN))
            labels.append((ax0 + 22, ay0 + 120, f"tape offset {e:+.2f} in    aim {ph:+.1f} deg    bands {int(row.n_bands)}",
                           F_BODY, TXT))
            for k, (name, raw) in enumerate((("L", row.left_raw), ("R", row.right_raw))):
                eff = -float(np.nan_to_num(raw))
                bx, by = ax0 + 70, ay0 + 180 + k * 40
                cv2.rectangle(canvas, (bx, by), (bx + 400, by + 22), (40, 48, 56), -1)
                cv2.line(canvas, (bx + 200, by - 3), (bx + 200, by + 25), DIM, 1)
                w = int(abs(eff) / 100 * 200)
                col = GREEN if eff >= 0 else RED
                cv2.rectangle(canvas, (bx + 200, by) if eff >= 0 else (bx + 200 - w, by),
                              (bx + 200 + w, by + 22) if eff >= 0 else (bx + 200, by + 22), col, -1)
                labels.append((ax0 + 22, by - 2, name, F_BODY_B, TXT))
                labels.append((bx + 410, by - 1, f"{eff:+.0f}", F_SMALL, DIM))
            labels.append((ax0 + 22, ay0 + 262, "wheel effort (up = forward)", F_TINY, DIM))
        if p["kind"] == "end":
            lines = wrap("The script drove 42 s of this course by itself. The AI handled both stops: it "
                         "diagnosed a stall, caught its own wrong-way nudge, and verified the finish. "
                         "Hand-coding those calls kept failing, so the script stays simple.", F_BODY, 640)
            for i, ln in enumerate(lines):
                labels.append((ax0 + 22, ay0 + 80 + i * 32, ln, F_BODY, TXT))

        # sheets (inspect) or the AI log (all other phases)
        rx0 = ax0 + 700
        if p["kind"] == "inspect":
            a, b = p["sheets"]
            use_a = (u % 6.0) < 3.0
            sh = fit(img(a if use_a else b), ax1 - rx0 - 14, 190)
            canvas[ay0 + 50:ay0 + 50 + 190, rx0:rx0 + sh.shape[1]] = sh
            labels.append((rx0, ay0 + 14, "last 6 frames before the stop  " +
                           ("annotated" if use_a else "RAW, no overlay"), F_SMALL, AMBER))
            labels.append((rx0, ay0 + 256, "the AI looks at the pictures first, then the log", F_TINY, DIM))
        else:
            labels.append((rx0, ay0 + 14, "AI LOG", F_SMALL, DIM))
            if t < self.phase_start("inspect", 0) + 3.0:
                labels.append((rx0, ay0 + 52, "no interventions yet: the script is driving", F_TINY, DIM))
            entries = [(q["t0"] + q["log_at"], q["log"]) for q in self.phases if "log" in q and q["t0"] + q["log_at"] <= t]
            for i, (_, (tag, msg)) in enumerate(entries):
                col = RED if tag == "WRONG WAY" else (GREEN if tag in ("REAL FINISH", "RESTART") else AMBER)
                labels.append((rx0, ay0 + 52 + i * 52, tag, F_SMALL, col))
                labels.append((rx0 + 4, ay0 + 52 + i * 52 + 22, msg, F_TINY, TXT))

        # --- title + counters
        n_ai = sum(1 for q in self.phases if q.get("count") and q["t0"] + q["log_at"] <= t)
        labels.append((24, 18, "eLlama line follower   simple script, AI as the judge", F_TITLE, TXT))
        for k, (nm, col) in enumerate((("script drives", GREEN), ("script stopped", RED), ("AI looks / acts", AMBER))):
            lx = 940 + k * 165
            cv2.rectangle(canvas, (lx, 30), (lx + 14, 44), col, -1)
            labels.append((lx + 20, 26, nm, F_TINY, DIM))
        labels.append((W - 420, 24, f"stops the AI handled: {n_ai}", F_CHIP, AMBER if n_ai else DIM))

        # --- timeline bar
        bx0, by0, bx1, by1 = BAR_R
        for q in self.phases:
            xa = int(bx0 + (bx1 - bx0) * q["t0"] / self.total)
            xb = int(bx0 + (bx1 - bx0) * (q["t0"] + q["dur"]) / self.total)
            col = GREEN if q["kind"] == "run" else (RED if q["kind"] == "stop" else AMBER)
            cv2.rectangle(canvas, (xa + 1, by0), (xb - 1, by1), col, -1)
        mx = int(bx0 + (bx1 - bx0) * min(t, self.total) / self.total)
        cv2.rectangle(canvas, (mx - 2, by0 - 6), (mx + 2, by1 + 6), (255, 255, 255), -1)

        pilimg = Image.fromarray(canvas)
        d = ImageDraw.Draw(pilimg)
        for item in labels:
            if item[0] == "chip":
                _, x0, y0, txt, col = item
                w = int(F_CHIP.getlength(txt)) + 36
                d.rounded_rectangle((x0, y0, x0 + w, y0 + 44), 10, fill=col)
                d.text((x0 + 18, y0 + 6), txt, font=F_CHIP, fill=(14, 18, 22))
            else:
                x0, y0, txt, fnt, col = item
                d.text((x0, y0), txt, font=fnt, fill=col)
        return np.asarray(pilimg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preview", type=float, nargs="+", default=None, help="render frames at these times (s)")
    ap.add_argument("--out", default=os.path.join(OUT, "linefollow_post.mp4"))
    args = ap.parse_args()
    rr = Renderer(load())
    print("phases:", [(q["kind"], round(q["t0"], 1)) for q in rr.phases], "total", round(rr.total, 1))
    if args.preview is not None:
        # replay the map layer up to that time so the preview shows the real state
        want = {int(round(v * FPS)): v for v in args.preview}
        for k in range(max(want) + 1):
            f = rr.render(k / FPS)
            if k in want:
                Image.fromarray(f).save(os.path.join(OUT, f"preview_{want[k]:.0f}.png"))
        return
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
           "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "18",
           "-pix_fmt", "yuv420p", "-movflags", "+faststart", args.out]
    ff = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    n = int(rr.total * FPS)
    for k in range(n):
        ff.stdin.write(rr.render(k / FPS).tobytes())
        if k % 150 == 0:
            print(f"{k}/{n}", flush=True)
    ff.stdin.close()
    ff.wait()
    print("wrote", args.out)


if __name__ == "__main__":
    main()
