#!/usr/bin/env python3
"""
eLlama top-down robot simulator.

A minimal OpenCV-only GUI: type drive commands into the MDI (Manual Data
Input) box, one per line, as:

    pwml pwmr time_ms

pwml / pwmr are signed motor PWM values (-255..255, matching the real
firmware's DrivePacket{left, right}), and time_ms is how long that command
runs. Click RUN to simulate the whole command list and watch the robot
drive the resulting path. Click EXPORT CSV to write the current command
list to exported_path.csv (next to this script), ready to hand to
send_csv.py so it can be replayed on the real robot without retyping.

MAX PWM (top-left) is required before RUN or EXPORT CSV will do anything.
It's a hard ceiling -- every command's (pwml, pwmr) pair is scaled down,
preserving their ratio, so neither wheel exceeds it. There is no default
on purpose: start low (40-80) and raise it once you trust the path,
rather than trusting a number picked for you. See simulation/README.md.

Speed model: PWM magnitude -> ft/s, piecewise-linear, calibrated against
real straight-line runs (see the _SPEED_PTS comment below). Pivot turns
use an inflated effective track width (see TURN_SCRUB_FACTOR) to account
for skid-steer wheel scrub, calibrated against a real 90-degree turn.

Run with: python3 sim/simulator.py
"""

import csv
import os

import cv2
import numpy as np

# ----------------------------------------------------------------------
# Robot geometry (from images/chassis_dimensions_isometric.png), inches
# ----------------------------------------------------------------------
BODY_LENGTH_IN = 18.5
BODY_WIDTH_IN = 15.0
WHEELBASE_IN = 14.0        # front-to-rear axle spacing
TRACK_WIDTH_IN = 18.0      # left-to-right wheel centerline spacing
WHEEL_DIAMETER_IN = 10.0
WHEEL_WIDTH_IN = 3.5

IN_PER_FT = 12.0
TRACK_WIDTH_FT = TRACK_WIDTH_IN / IN_PER_FT

# ----------------------------------------------------------------------
# Speed model: PWM magnitude -> ft/s, piecewise-linear through the two
# given data points and the origin. Calibrated 2026-09-19 against real
# straight-line runs (120,120/1000ms->20in, 80,80/1000ms->12.5in,
# 80,80/2000ms->24in), which came in at ~0.335x the originally guessed
# curve -- scaled the same two-breakpoint shape down to match.
# ----------------------------------------------------------------------
_PWM_PTS = [0, 100, 255]
_SPEED_PTS = [0.0, 1.339, 3.349]  # ft/s (since the given durations are 1000ms)

# Pivot/differential turns scrub noticeably more than straight-line rolling
# resistance predicts (4-wheel skid-steer drags all four wheels sideways).
# Calibrated from a real 123,-123/1800ms command turning 90 degrees: the
# effective track width for turn-rate purposes is ~2.5x the physical one.
TURN_SCRUB_FACTOR = 2.5
EFFECTIVE_TRACK_WIDTH_FT = TRACK_WIDTH_FT * TURN_SCRUB_FACTOR


def pwm_to_speed_fps(pwm):
    """Signed PWM (-255..255) -> signed speed in ft/s."""
    pwm = max(-255, min(255, pwm))
    speed = np.interp(abs(pwm), _PWM_PTS, _SPEED_PTS)
    return speed if pwm >= 0 else -speed


# ----------------------------------------------------------------------
# Simulation
# ----------------------------------------------------------------------
SIM_DT = 0.02  # seconds per integration step


def simulate(commands):
    """commands: list of (pwml, pwmr, time_ms). Returns list of (x_ft, y_ft, theta_rad) poses,
    starting with the initial pose, in world coords (x right, y up, theta ccw from +x)."""
    x, y, theta = 0.0, 0.0, np.pi / 2  # start facing "up"
    poses = [(x, y, theta)]
    for pwml, pwmr, time_ms in commands:
        v_l = pwm_to_speed_fps(pwml)
        v_r = pwm_to_speed_fps(pwmr)
        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / EFFECTIVE_TRACK_WIDTH_FT
        steps = max(1, int(round(time_ms / 1000.0 / SIM_DT)))
        for _ in range(steps):
            x += v * np.cos(theta) * SIM_DT
            y += v * np.sin(theta) * SIM_DT
            theta += omega * SIM_DT
            poses.append((x, y, theta))
    return poses


def clamp_commands(commands, max_pwm):
    """Scale each command's (pwml, pwmr) pair down, preserving their ratio (so
    turn shape/curvature is unaffected), so neither wheel exceeds max_pwm."""
    out = []
    for pwml, pwmr, time_ms in commands:
        mag = max(abs(pwml), abs(pwmr))
        if mag > max_pwm and mag > 0:
            scale = max_pwm / mag
            pwml = int(round(pwml * scale))
            pwmr = int(round(pwmr * scale))
        out.append((pwml, pwmr, time_ms))
    return out


# ----------------------------------------------------------------------
# UI layout
# ----------------------------------------------------------------------
WIN_NAME = "eLlama Simulator"
VIEW_W, VIEW_H = 800, 760
PANEL_W = 380
WIN_W, WIN_H = VIEW_W + PANEL_W, VIEW_H

# Required safety ceiling: RUN and EXPORT CSV refuse to act until this is
# set, and then scale every command's PWM pair down (preserving the L/R
# ratio, so turn shape is unaffected) so neither wheel exceeds it. There is
# deliberately no default -- see simulation/README.md on why.
MAX_PWM_LABEL_POS = (VIEW_W + 15, 78)
MAX_PWM_RECT = (VIEW_W + 15, 84, VIEW_W + 120, 114)

TEXTBOX_RECT = (VIEW_W + 15, 120, WIN_W - 15, 520)         # x0,y0,x1,y1
RUN_BUTTON_RECT = (VIEW_W + 15, 535, VIEW_W + 185, 580)
CLEAR_BUTTON_RECT = (VIEW_W + 195, 535, VIEW_W + 365, 580)
RESET_VIEW_RECT = (VIEW_W + 15, 595, VIEW_W + 365, 640)
EXPORT_BUTTON_RECT = (VIEW_W + 15, 650, VIEW_W + 365, 695)

# Where EXPORT CSV writes, next to this script regardless of cwd, so it's
# always where send_csv.py's default lookup (same directory) expects it.
EXPORT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "exported_path.csv")

BG = (40, 40, 40)
PANEL_BG = (55, 55, 55)
TEXTBOX_BG = (25, 25, 25)
TEXT_COLOR = (230, 230, 230)
ACCENT = (60, 180, 255)
GRID_COLOR = (70, 70, 70)


class Simulator:
    def __init__(self):
        self.lines = [""]          # MDI text buffer, list of lines
        self.focused = False
        self.max_pwm_str = ""      # required; no default, see MAX_PWM_RECT
        self.max_pwm_focused = False
        self.status = "Set MAX PWM, click the MDI box, type commands, then RUN."
        self.poses = simulate([])  # trajectory currently shown (starts at origin)
        self.anim_index = 0
        self.animating = False
        self.px_per_ft = 30.0
        self.origin_px = (VIEW_W // 2, VIEW_H // 2)

        cv2.namedWindow(WIN_NAME)
        cv2.setMouseCallback(WIN_NAME, self.on_mouse)

    # -------------------- input handling --------------------
    def on_mouse(self, event, mx, my, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        self.max_pwm_focused = False
        if _in_rect(mx, my, MAX_PWM_RECT):
            self.focused = False
            self.max_pwm_focused = True
        elif _in_rect(mx, my, TEXTBOX_RECT):
            self.focused = True
        elif _in_rect(mx, my, RUN_BUTTON_RECT):
            self.focused = False
            self.run_commands()
        elif _in_rect(mx, my, CLEAR_BUTTON_RECT):
            self.focused = False
            self.lines = [""]
            self.status = "Commands cleared."
            self.poses = simulate([])
            self.anim_index = 0
            self.animating = False
        elif _in_rect(mx, my, RESET_VIEW_RECT):
            self.focused = False
            self.px_per_ft = 30.0
            self.origin_px = (VIEW_W // 2, VIEW_H // 2)
            self.status = "View reset."
        elif _in_rect(mx, my, EXPORT_BUTTON_RECT):
            self.focused = False
            self.export_csv()
        else:
            self.focused = False

    def handle_key(self, key):
        if key == -1:
            return
        if self.max_pwm_focused:
            k = key & 0xFF
            if k in (8, 127):       # Backspace
                self.max_pwm_str = self.max_pwm_str[:-1]
            elif 48 <= k <= 57 and len(self.max_pwm_str) < 3:  # digits 0-9
                self.max_pwm_str += chr(k)
            return
        if not self.focused:
            return
        if key in (13, 10):        # Enter
            self.lines.append("")
        elif key in (8, 127):      # Backspace
            if self.lines[-1]:
                self.lines[-1] = self.lines[-1][:-1]
            elif len(self.lines) > 1:
                self.lines.pop()
        elif 32 <= key <= 126:     # printable ASCII
            self.lines[-1] += chr(key)

    # -------------------- MAX PWM safety ceiling --------------------
    def _get_max_pwm(self):
        """Returns (max_pwm, err). max_pwm is None on err -- blank or out of 1..255."""
        s = self.max_pwm_str.strip()
        if not s:
            return None, "Set MAX PWM first -- start low, e.g. 40-80, and raise it once you trust the path."
        val = int(s)
        if not (1 <= val <= 255):
            return None, "MAX PWM must be 1-255."
        return val, None

    # -------------------- command parsing / simulation --------------------
    def _parse_commands(self):
        commands = []
        for lineno, raw in enumerate(self.lines, start=1):
            line = raw.strip().replace(",", " ")
            if not line:
                continue
            parts = line.split()
            if len(parts) != 3:
                return None, f"Line {lineno}: expected 'pwml pwmr time_ms', got '{raw}'"
            try:
                pwml, pwmr, time_ms = int(parts[0]), int(parts[1]), int(parts[2])
            except ValueError:
                return None, f"Line {lineno}: values must be integers ('{raw}')"
            commands.append((pwml, pwmr, time_ms))
        if not commands:
            return None, "No commands entered."
        return commands, None

    def run_commands(self):
        commands, err = self._parse_commands()
        if err:
            self.status = err
            return
        max_pwm, err = self._get_max_pwm()
        if err:
            self.status = err
            return
        commands = clamp_commands(commands, max_pwm)

        self.poses = simulate(commands)
        self._fit_view()
        self.anim_index = 0
        self.animating = True
        self.status = f"Running {len(commands)} command(s), capped at {max_pwm} PWM..."

    def export_csv(self):
        commands, err = self._parse_commands()
        if err:
            self.status = err
            return
        max_pwm, err = self._get_max_pwm()
        if err:
            self.status = err
            return
        commands = clamp_commands(commands, max_pwm)
        with open(EXPORT_PATH, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["pwml", "pwmr", "time_ms"])
            writer.writerows(commands)
        self.status = f"Exported {len(commands)} command(s), capped at {max_pwm} PWM, to {EXPORT_PATH}"

    def _fit_view(self):
        xs = [p[0] for p in self.poses]
        ys = [p[1] for p in self.poses]
        span_x = max(xs) - min(xs)
        span_y = max(ys) - min(ys)
        span = max(span_x, span_y, TRACK_WIDTH_FT * 3)
        margin = 0.15
        self.px_per_ft = min(VIEW_W, VIEW_H) * (1 - 2 * margin) / span
        self.px_per_ft = min(self.px_per_ft, 60.0)
        cx = (max(xs) + min(xs)) / 2.0
        cy = (max(ys) + min(ys)) / 2.0
        self.origin_px = (
            VIEW_W // 2 - int(cx * self.px_per_ft),
            VIEW_H // 2 + int(cy * self.px_per_ft),
        )

    def world_to_px(self, x_ft, y_ft):
        ox, oy = self.origin_px
        return int(ox + x_ft * self.px_per_ft), int(oy - y_ft * self.px_per_ft)

    # -------------------- drawing --------------------
    def draw(self):
        frame = np.full((WIN_H, WIN_W, 3), BG, dtype=np.uint8)
        self._draw_view(frame)
        self._draw_panel(frame)
        return frame

    def _draw_view(self, frame):
        view = frame[0:VIEW_H, 0:VIEW_W]
        step_px = max(1, int(self.px_per_ft))
        ox, oy = self.origin_px
        for gx in range(ox % step_px, VIEW_W, step_px):
            cv2.line(view, (gx, 0), (gx, VIEW_H), GRID_COLOR, 1)
        for gy in range(oy % step_px, VIEW_H, step_px):
            cv2.line(view, (0, gy), (VIEW_W, gy), GRID_COLOR, 1)
        cv2.line(view, (0, oy), (VIEW_W, oy), (90, 90, 60), 1)
        cv2.line(view, (ox, 0), (ox, VIEW_H), (90, 90, 60), 1)

        # path trace
        pts = [self.world_to_px(x, y) for x, y, _ in self.poses[: self.anim_index + 1]]
        for i in range(1, len(pts)):
            cv2.line(view, pts[i - 1], pts[i], ACCENT, 2)

        pose = self.poses[min(self.anim_index, len(self.poses) - 1)]
        draw_robot(view, pose, self.px_per_ft, self.world_to_px)

        if self.animating:
            self.anim_index += 2
            if self.anim_index >= len(self.poses) - 1:
                self.anim_index = len(self.poses) - 1
                self.animating = False
                self.status = "Done."

    def _draw_panel(self, frame):
        x0 = VIEW_W
        frame[:, x0:WIN_W] = PANEL_BG
        cv2.putText(frame, "MDI Command Input", (x0 + 15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR, 2)
        cv2.putText(frame, "format: pwml pwmr time_ms", (x0 + 15, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

        lx, ly = MAX_PWM_LABEL_POS
        cv2.putText(frame, "MAX PWM (required, start low):", (lx, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        mx0, my0, mx1, my1 = MAX_PWM_RECT
        if self.max_pwm_focused:
            mborder = ACCENT
        elif not self.max_pwm_str.strip():
            mborder = (0, 140, 255)  # unset -- flagged in orange, RUN/EXPORT are blocked
        else:
            mborder = (100, 100, 100)
        cv2.rectangle(frame, (mx0, my0), (mx1, my1), TEXTBOX_BG, -1)
        cv2.rectangle(frame, (mx0, my0), (mx1, my1), mborder, 2 if self.max_pwm_focused else 1)
        mtext = self.max_pwm_str + ("_" if self.max_pwm_focused else "")
        cv2.putText(frame, mtext, (mx0 + 8, my1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)

        tb = TEXTBOX_RECT
        border = ACCENT if self.focused else (100, 100, 100)
        cv2.rectangle(frame, (tb[0], tb[1]), (tb[2], tb[3]), TEXTBOX_BG, -1)
        cv2.rectangle(frame, (tb[0], tb[1]), (tb[2], tb[3]), border, 2)
        line_h = 22
        max_lines = (tb[3] - tb[1] - 10) // line_h
        visible = self.lines[-max_lines:] if len(self.lines) > max_lines else self.lines
        for i, line in enumerate(visible):
            ty = tb[1] + 20 + i * line_h
            cursor = "_" if (self.focused and i == len(visible) - 1) else ""
            cv2.putText(frame, line + cursor, (tb[0] + 8, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)

        _draw_button(frame, RUN_BUTTON_RECT, "RUN", (60, 160, 60))
        _draw_button(frame, CLEAR_BUTTON_RECT, "CLEAR", (140, 60, 60))
        _draw_button(frame, RESET_VIEW_RECT, "RESET VIEW", (70, 70, 110))
        _draw_button(frame, EXPORT_BUTTON_RECT, "EXPORT CSV", (90, 90, 140))

        cv2.putText(frame, "Status:", (x0 + 15, 720),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
        cv2.putText(frame, self.status[:44], (x0 + 15, 740),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1)

    def loop(self):
        while True:
            frame = self.draw()
            cv2.imshow(WIN_NAME, frame)
            key = cv2.waitKey(15) & 0xFF
            if key == 27:  # ESC quits
                break
            if cv2.getWindowProperty(WIN_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            self.handle_key(key)
        cv2.destroyAllWindows()


def _in_rect(mx, my, rect):
    x0, y0, x1, y1 = rect
    return x0 <= mx <= x1 and y0 <= my <= y1


def _draw_button(frame, rect, label, color):
    x0, y0, x1, y1 = rect
    cv2.rectangle(frame, (x0, y0), (x1, y1), color, -1)
    cv2.rectangle(frame, (x0, y0), (x1, y1), (220, 220, 220), 1)
    size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
    tx = x0 + (x1 - x0 - size[0]) // 2
    ty = y0 + (y1 - y0 + size[1]) // 2
    cv2.putText(frame, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)


def _rot(pts, theta):
    """Rotate an (N,2) array of local points (x forward, y left) by theta (world ccw from +x)."""
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T


def draw_robot(view, pose, px_per_ft, world_to_px):
    x, y, theta = pose
    body_l = BODY_LENGTH_IN / IN_PER_FT
    body_w = BODY_WIDTH_IN / IN_PER_FT
    wheel_d = WHEEL_DIAMETER_IN / IN_PER_FT
    wheel_w = WHEEL_WIDTH_IN / IN_PER_FT
    half_wb = (WHEELBASE_IN / IN_PER_FT) / 2.0
    half_tw = (TRACK_WIDTH_FT) / 2.0

    def poly_px(local_pts):
        world = _rot(np.array(local_pts), theta) + np.array([x, y])
        return np.array([world_to_px(wx, wy) for wx, wy in world], dtype=np.int32)

    # four wheels: local (forward, left) offsets at the corners
    wheel_centers = [
        (half_wb, half_tw), (half_wb, -half_tw),
        (-half_wb, half_tw), (-half_wb, -half_tw),
    ]
    for cf, cl in wheel_centers:
        rect_local = [
            (cf + wheel_d / 2, cl + wheel_w / 2), (cf + wheel_d / 2, cl - wheel_w / 2),
            (cf - wheel_d / 2, cl - wheel_w / 2), (cf - wheel_d / 2, cl + wheel_w / 2),
        ]
        cv2.fillPoly(view, [poly_px(rect_local)], (25, 25, 25))

    # body
    body_local = [
        (body_l / 2, body_w / 2), (body_l / 2, -body_w / 2),
        (-body_l / 2, -body_w / 2), (-body_l / 2, body_w / 2),
    ]
    cv2.fillPoly(view, [poly_px(body_local)], (215, 215, 215))
    cv2.polylines(view, [poly_px(body_local)], True, (120, 120, 120), 2)

    # front sensor pair (matches the two red modules in the reference photo)
    sensor_y = body_w * 0.18
    for sign in (1, -1):
        sensor_local = [
            (body_l / 2 - 0.05, sign * sensor_y + 0.06), (body_l / 2 - 0.05, sign * sensor_y - 0.06),
            (body_l / 2 - 0.2, sign * sensor_y - 0.06), (body_l / 2 - 0.2, sign * sensor_y + 0.06),
        ]
        cv2.fillPoly(view, [poly_px(sensor_local)], (30, 30, 220))

    # heading arrow
    tip = world_to_px(*( _rot(np.array([[body_l / 2 + 0.3, 0.0]]), theta)[0] + np.array([x, y])))
    base = world_to_px(x, y)
    cv2.arrowedLine(view, base, tip, ACCENT, 2, tipLength=0.3)


if __name__ == "__main__":
    Simulator().loop()
