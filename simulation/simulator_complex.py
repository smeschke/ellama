#!/usr/bin/env python3
"""
eLlama top-down robot simulator.

A minimal OpenCV-only GUI: enter drive commands into the MDI (Manual Data
Input) grid, one command per row, as three cells:

    pwml | pwmr | time_ms

pwml / pwmr are signed motor PWM values (-255..255, matching the real
firmware's DrivePacket{left, right}), and time_ms is how long that command
runs. Click a cell to focus it, type digits (and a leading '-' for negative
PWM), and use Tab / Shift+Tab to move to the next / previous cell. Click RUN
to simulate the whole command list and watch the robot drive the resulting
path. Click EXPORT CSV to write the current grid to exported_path.csv
(next to this script), ready to hand to send_csv.py so it can be replayed
on the real robot without retyping.

Speed model: PWM magnitude -> ft/s, piecewise-linear, calibrated against
real straight-line runs (see the _SPEED_PTS comment below). Pivot turns
use an inflated effective track width (see TURN_SCRUB_FACTOR) to account
for skid-steer wheel scrub, calibrated against a real 90-degree turn.

Run with: python3 sim/simulator_complex.py
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
# Used for all turn-rate kinematics (simulate() and the waypoint/arc
# planner below); TRACK_WIDTH_FT itself stays physical for drawing.
TURN_SCRUB_FACTOR = 2.5
EFFECTIVE_TRACK_WIDTH_FT = TRACK_WIDTH_FT * TURN_SCRUB_FACTOR


def pwm_to_speed_fps(pwm):
    """Signed PWM (-255..255) -> signed speed in ft/s."""
    pwm = max(-255, min(255, pwm))
    speed = np.interp(abs(pwm), _PWM_PTS, _SPEED_PTS)
    return speed if pwm >= 0 else -speed


def speed_to_pwm(speed):
    """Signed speed in ft/s -> signed PWM (-255..255). Inverse of pwm_to_speed_fps
    (valid since the speed curve is monotonic, so swapping the interp tables works)."""
    max_speed = _SPEED_PTS[-1]
    speed = max(-max_speed, min(max_speed, speed))
    pwm = np.interp(abs(speed), _SPEED_PTS, _PWM_PTS)
    return pwm if speed >= 0 else -pwm


# ----------------------------------------------------------------------
# Simulation
# ----------------------------------------------------------------------
SIM_DT = 0.02  # seconds per integration step
INITIAL_POSE = (0.0, 0.0, np.pi / 2)  # matches simulate()'s starting pose

# ----------------------------------------------------------------------
# Click-to-waypoint planning. Two modes:
#  - "turn_drive": turn in place, then drive straight, both at fixed PWM.
#  - "arc": a single constant-PWM command tracing the circular arc that is
#    tangent to the current heading and passes through the clicked point
#    (the same "pure pursuit" geometry real path-followers use).
# ----------------------------------------------------------------------
WAYPOINT_TURN_PWM = 120
WAYPOINT_DRIVE_PWM = 150
WAYPOINT_TURN_OMEGA = 2.0 * pwm_to_speed_fps(WAYPOINT_TURN_PWM) / EFFECTIVE_TRACK_WIDTH_FT  # rad/s
WAYPOINT_DRIVE_SPEED = pwm_to_speed_fps(WAYPOINT_DRIVE_PWM)  # ft/s
MIN_TURN_RAD = np.radians(2.0)  # skip generating a turn command below this

ARC_NOMINAL_SPEED = WAYPOINT_DRIVE_SPEED  # ft/s cruising speed an arc aims for
MAX_SPEED_FPS = _SPEED_PTS[-1]            # fastest a wheel can go (PWM 255)
MIN_ARC_LATERAL_FT = 0.05                 # |y_l| below this is treated as "straight ahead"
MIN_ARC_FORWARD_FT = 0.05                 # x_l below this means "point is behind us"


def simulate(commands):
    """commands: list of (pwml, pwmr, time_ms). Returns list of (x_ft, y_ft, theta_rad) poses,
    starting with the initial pose, in world coords (x right, y up, theta ccw from +x)."""
    x, y, theta = INITIAL_POSE
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


# ----------------------------------------------------------------------
# UI layout (sized for a 4K/hi-DPI display)
# ----------------------------------------------------------------------
WIN_NAME = "eLlama Simulator"
VIEW_W, VIEW_H = 2400, 1900
PANEL_W = 1000
WIN_W, WIN_H = VIEW_W + PANEL_W, VIEW_H

# MDI command grid: one row per command, 3 numeric columns.
GRID_ROWS = 40
GRID_COLS = 3
COL_HEADERS = ["PWM L", "PWM R", "TIME (ms)"]

ROW_NUM_W = 50
CELL_H = 30
GRID_X0 = VIEW_W + 20
GRID_X1 = WIN_W - 20
GRID_Y0 = 145
GRID_Y1 = GRID_Y0 + GRID_ROWS * CELL_H
DATA_X0 = GRID_X0 + ROW_NUM_W
COL_W = (GRID_X1 - DATA_X0) // GRID_COLS

RUN_BUTTON_RECT = (GRID_X0, GRID_Y1 + 25, GRID_X0 + 170, GRID_Y1 + 75)
CLEAR_BUTTON_RECT = (GRID_X0 + 190, GRID_Y1 + 25, GRID_X0 + 360, GRID_Y1 + 75)
RESET_VIEW_RECT = (GRID_X0, GRID_Y1 + 95, GRID_X0 + 170, GRID_Y1 + 145)
CLEAR_POINTS_RECT = (GRID_X0 + 190, GRID_Y1 + 95, GRID_X0 + 360, GRID_Y1 + 145)
MODE_TOGGLE_RECT = (GRID_X0, GRID_Y1 + 165, GRID_X0 + 360, GRID_Y1 + 215)
EXPORT_BUTTON_RECT = (GRID_X0, GRID_Y1 + 235, GRID_X0 + 360, GRID_Y1 + 285)

# Where EXPORT CSV writes, next to this script regardless of cwd, so it's
# always where send_csv.py's default lookup (same directory) expects it.
EXPORT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "exported_path.csv")

# Zoom buttons, overlaid in the top-right corner of the map view.
ZOOM_BTN = 50
ZOOM_IN_RECT = (VIEW_W - ZOOM_BTN - 20, 20, VIEW_W - 20, 20 + ZOOM_BTN)
ZOOM_OUT_RECT = (VIEW_W - ZOOM_BTN - 20, 30 + ZOOM_BTN, VIEW_W - 20, 30 + 2 * ZOOM_BTN)
ZOOM_FACTOR = 1.25
MIN_PX_PER_FT = 4.0
MAX_PX_PER_FT = 300.0
DEFAULT_PX_PER_FT = 40.0

BG = (40, 40, 40)
PANEL_BG = (55, 55, 55)
TEXTBOX_BG = (25, 25, 25)
TEXT_COLOR = (230, 230, 230)
ACCENT = (60, 180, 255)
GRID_COLOR = (70, 70, 70)
WAYPOINT_COLOR = (0, 200, 255)

# Raw (unmasked) key codes from cv2.waitKeyEx() on Linux/GTK; Shift+Tab's
# code is backend-dependent, so it may not register on every OpenCV build.
KEY_TAB = 9
KEY_SHIFT_TAB = 65056  # ISO_Left_Tab


class Simulator:
    def __init__(self):
        self.grid = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
        self.cur_row, self.cur_col = 0, 0
        self.focused = False
        self.status = "Click a grid cell, type commands, then RUN."
        self.poses = simulate([])  # trajectory currently shown (starts at origin)
        self.anim_index = 0
        self.animating = False
        self.px_per_ft = DEFAULT_PX_PER_FT
        self.origin_px = (VIEW_W // 2, VIEW_H // 2)

        # click-to-waypoint planning state
        self.waypoints = []            # [(x_ft, y_ft), ...] clicked so far
        self.waypoint_row_groups = []  # grid row indices used by each waypoint
        self.plan_pose = INITIAL_POSE  # pose the next waypoint is planned from
        self.point_mode = "arc"        # "arc" or "turn_drive"

        cv2.namedWindow(WIN_NAME)
        cv2.setMouseCallback(WIN_NAME, self.on_mouse)

    # -------------------- input handling --------------------
    def _cell_at(self, mx, my):
        if not (DATA_X0 <= mx <= GRID_X1 and GRID_Y0 <= my <= GRID_Y1):
            return None, None
        col = min(GRID_COLS - 1, int((mx - DATA_X0) // COL_W))
        row = min(GRID_ROWS - 1, int((my - GRID_Y0) // CELL_H))
        return row, col

    def on_mouse(self, event, mx, my, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        row, col = self._cell_at(mx, my)
        if row is not None:
            self.focused = True
            self.cur_row, self.cur_col = row, col
        elif _in_rect(mx, my, RUN_BUTTON_RECT):
            self.focused = False
            self.run_commands()
        elif _in_rect(mx, my, CLEAR_BUTTON_RECT):
            self.focused = False
            self._clear_all()
        elif _in_rect(mx, my, RESET_VIEW_RECT):
            self.focused = False
            self._reset_view()
        elif _in_rect(mx, my, CLEAR_POINTS_RECT):
            self.focused = False
            self._clear_points()
        elif _in_rect(mx, my, MODE_TOGGLE_RECT):
            self.focused = False
            self.point_mode = "turn_drive" if self.point_mode == "arc" else "arc"
            self.status = f"Click mode: {self.point_mode.replace('_', '+')}"
        elif _in_rect(mx, my, EXPORT_BUTTON_RECT):
            self.focused = False
            self.export_csv()
        elif _in_rect(mx, my, ZOOM_IN_RECT):
            self.focused = False
            self.zoom(ZOOM_FACTOR)
        elif _in_rect(mx, my, ZOOM_OUT_RECT):
            self.focused = False
            self.zoom(1.0 / ZOOM_FACTOR)
        elif 0 <= mx < VIEW_W and 0 <= my < VIEW_H:
            self.focused = False
            self._collect_point(mx, my)
        else:
            self.focused = False

    # -------------------- view controls --------------------
    def zoom(self, factor):
        """Zoom the map view, keeping the point at the view's center fixed."""
        cx_px, cy_px = VIEW_W // 2, VIEW_H // 2
        ox, oy = self.origin_px
        wx = (cx_px - ox) / self.px_per_ft
        wy = (oy - cy_px) / self.px_per_ft
        self.px_per_ft = max(MIN_PX_PER_FT, min(MAX_PX_PER_FT, self.px_per_ft * factor))
        self.origin_px = (
            int(cx_px - wx * self.px_per_ft),
            int(cy_px + wy * self.px_per_ft),
        )
        self.status = f"Zoom: {self.px_per_ft:.0f} px/ft"

    def _reset_view(self):
        self.px_per_ft = DEFAULT_PX_PER_FT
        self.origin_px = (VIEW_W // 2, VIEW_H // 2)
        self.status = "View reset."

    def _clear_all(self):
        self.grid = [["" for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
        self.cur_row, self.cur_col = 0, 0
        self.waypoints = []
        self.waypoint_row_groups = []
        self.plan_pose = INITIAL_POSE
        self.status = "Commands cleared."
        self.poses = simulate([])
        self.anim_index = 0
        self.animating = False

    def _clear_points(self):
        for rows in self.waypoint_row_groups:
            for i in rows:
                self.grid[i] = ["", "", ""]
        self.waypoints = []
        self.waypoint_row_groups = []
        self.plan_pose = INITIAL_POSE
        self.status = "Waypoints cleared."

    # -------------------- click-to-waypoint planning --------------------
    def _append_row(self, pwml, pwmr, time_ms):
        """Write into the first empty grid row. Returns the row index, or None if full."""
        for i, row in enumerate(self.grid):
            if all(c == "" for c in row):
                row[0] = str(int(round(pwml)))
                row[1] = str(int(round(pwmr)))
                row[2] = str(int(round(time_ms)))
                return i
        return None

    def _plan_turn_drive(self, ptheta, dx, dy, dist):
        """Turn in place to face the point, then drive straight to it (2 commands)."""
        target_heading = float(np.arctan2(dy, dx))
        dtheta = float(np.arctan2(np.sin(target_heading - ptheta), np.cos(target_heading - ptheta)))

        commands = []
        if abs(dtheta) > MIN_TURN_RAD:
            turn_ms = int(round(abs(dtheta) / WAYPOINT_TURN_OMEGA * 1000.0))
            sign = 1.0 if dtheta > 0 else -1.0
            commands.append((-sign * WAYPOINT_TURN_PWM, sign * WAYPOINT_TURN_PWM, turn_ms))

        drive_ms = int(round(dist / WAYPOINT_DRIVE_SPEED * 1000.0))
        commands.append((WAYPOINT_DRIVE_PWM, WAYPOINT_DRIVE_PWM, drive_ms))
        return commands, target_heading

    def _plan_arc(self, ptheta, dx, dy, dist):
        """A single constant-PWM command tracing the arc, tangent to the current heading,
        that passes through the point (pure-pursuit circle-through-goal geometry)."""
        x_l = dx * np.cos(ptheta) + dy * np.sin(ptheta)   # forward
        y_l = -dx * np.sin(ptheta) + dy * np.cos(ptheta)  # left

        if x_l <= MIN_ARC_FORWARD_FT:
            self.status = "Point is behind the current heading; arc mode needs a point roughly ahead."
            return None

        if abs(y_l) < MIN_ARC_LATERAL_FT:
            time_ms = int(round(dist / ARC_NOMINAL_SPEED * 1000.0))
            pwm = speed_to_pwm(ARC_NOMINAL_SPEED)
            return [(pwm, pwm, time_ms)], ptheta

        # Tangent-chord geometry: circle tangent to the heading at the start,
        # passing through (x_l, y_l). alpha is the chord angle; the arc sweeps 2*alpha.
        alpha = float(np.arctan2(y_l, x_l))
        phi = 2.0 * alpha
        radius = (dist * dist) / (2.0 * y_l)
        omega = ARC_NOMINAL_SPEED / radius
        v_l = ARC_NOMINAL_SPEED - omega * EFFECTIVE_TRACK_WIDTH_FT / 2.0
        v_r = ARC_NOMINAL_SPEED + omega * EFFECTIVE_TRACK_WIDTH_FT / 2.0

        fastest = max(abs(v_l), abs(v_r))
        if fastest > MAX_SPEED_FPS:
            scale = MAX_SPEED_FPS / fastest
            v_l *= scale
            v_r *= scale

        v_avg = (v_l + v_r) / 2.0
        arc_len = abs(radius * phi)
        time_ms = int(round(arc_len / abs(v_avg) * 1000.0))
        if time_ms <= 0:
            self.status = "Arc command would be instantaneous; ignored."
            return None

        pwml = speed_to_pwm(v_l)
        pwmr = speed_to_pwm(v_r)
        return [(pwml, pwmr, time_ms)], ptheta + phi

    def _collect_point(self, mx, my):
        ox, oy = self.origin_px
        tx = (mx - ox) / self.px_per_ft
        ty = (oy - my) / self.px_per_ft

        px, py, ptheta = self.plan_pose
        dx, dy = tx - px, ty - py
        dist = float(np.hypot(dx, dy))
        if dist < 0.05:
            self.status = "Point too close to the previous one; ignored."
            return

        if self.point_mode == "arc":
            result = self._plan_arc(ptheta, dx, dy, dist)
        else:
            result = self._plan_turn_drive(ptheta, dx, dy, dist)
        if result is None:
            return  # helper already set self.status

        commands, final_heading = result
        rows_used = []
        for pwml, pwmr, time_ms in commands:
            i = self._append_row(pwml, pwmr, time_ms)
            if i is None:
                break
            rows_used.append(i)

        if not rows_used:
            self.status = "MDI grid is full; couldn't add point."
            return

        self.waypoint_row_groups.append(rows_used)
        self.waypoints.append((tx, ty))
        self.plan_pose = (tx, ty, final_heading)
        mode_label = "Arc" if self.point_mode == "arc" else "Point"
        self.status = f"{mode_label} {len(self.waypoints)}: {len(rows_used)} command(s) added."

    def _move_cell(self, delta):
        idx = self.cur_row * GRID_COLS + self.cur_col
        idx = (idx + delta) % (GRID_ROWS * GRID_COLS)
        self.cur_row, self.cur_col = divmod(idx, GRID_COLS)

    def handle_key(self, key):
        if key == -1:
            return
        if not self.focused:
            return
        if key == KEY_TAB:
            self._move_cell(1)
            return
        if key == KEY_SHIFT_TAB:
            self._move_cell(-1)
            return
        k = key & 0xFF
        cell = self.grid[self.cur_row][self.cur_col]
        if k in (13, 10):           # Enter -> next cell
            self._move_cell(1)
        elif k in (8, 127):         # Backspace
            self.grid[self.cur_row][self.cur_col] = cell[:-1]
        elif k == ord('-'):
            if not cell:
                self.grid[self.cur_row][self.cur_col] = '-'
        elif 48 <= k <= 57:         # digits 0-9
            self.grid[self.cur_row][self.cur_col] = cell + chr(k)

    # -------------------- command parsing / simulation --------------------
    def _parse_commands(self):
        commands = []
        for rownum, row in enumerate(self.grid, start=1):
            cells = [c.strip() for c in row]
            if all(c == "" for c in cells):
                continue
            if any(c == "" for c in cells):
                return None, f"Row {rownum}: fill in all three cells (pwml, pwmr, time_ms)"
            try:
                pwml, pwmr, time_ms = int(cells[0]), int(cells[1]), int(cells[2])
            except ValueError:
                return None, f"Row {rownum}: values must be integers"
            commands.append((pwml, pwmr, time_ms))
        if not commands:
            return None, "No commands entered."
        return commands, None

    def run_commands(self):
        commands, err = self._parse_commands()
        if err:
            self.status = err
            return

        self.poses = simulate(commands)
        self._fit_view()
        self.anim_index = 0
        self.animating = True
        self.status = f"Running {len(commands)} command(s)..."

    def export_csv(self):
        commands, err = self._parse_commands()
        if err:
            self.status = err
            return
        with open(EXPORT_PATH, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["pwml", "pwmr", "time_ms"])
            writer.writerows(commands)
        self.status = f"Exported {len(commands)} command(s) to {EXPORT_PATH}"

    def _fit_view(self):
        xs = [p[0] for p in self.poses]
        ys = [p[1] for p in self.poses]
        span_x = max(xs) - min(xs)
        span_y = max(ys) - min(ys)
        span = max(span_x, span_y, TRACK_WIDTH_FT * 3)
        margin = 0.15
        self.px_per_ft = min(VIEW_W, VIEW_H) * (1 - 2 * margin) / span
        self.px_per_ft = min(self.px_per_ft, 90.0)
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
        self._draw_zoom_buttons(frame)
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

        # planned waypoints (clicked, not yet necessarily run)
        if self.waypoints:
            prev = self.world_to_px(INITIAL_POSE[0], INITIAL_POSE[1])
            for wx, wy in self.waypoints:
                pt = self.world_to_px(wx, wy)
                cv2.line(view, prev, pt, WAYPOINT_COLOR, 1)
                cv2.circle(view, pt, 5, WAYPOINT_COLOR, -1)
                prev = pt

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

    def _draw_zoom_buttons(self, frame):
        _draw_button(frame, ZOOM_IN_RECT, "+", (70, 70, 110))
        _draw_button(frame, ZOOM_OUT_RECT, "-", (70, 70, 110))

    def _draw_panel(self, frame):
        x0 = VIEW_W
        frame[:, x0:WIN_W] = PANEL_BG
        cv2.putText(frame, "MDI Command Grid", (x0 + 20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, TEXT_COLOR, 2)
        cv2.putText(frame, "click a cell, type digits, Tab / Shift+Tab to move",
                    (x0 + 20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1)
        cv2.putText(frame, "click the map to add a waypoint; +/- (top-right) to zoom",
                    (x0 + 20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
        cv2.putText(frame, "MODE button below switches how a click becomes a command",
                    (x0 + 20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)

        for c, label in enumerate(COL_HEADERS):
            cx = DATA_X0 + c * COL_W + COL_W // 2
            size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0]
            cv2.putText(frame, label, (cx - size[0] // 2, GRID_Y0 - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)

        for r in range(GRID_ROWS):
            ry0 = GRID_Y0 + r * CELL_H
            ry1 = ry0 + CELL_H
            cv2.putText(frame, str(r + 1), (GRID_X0 + 5, ry1 - 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (140, 140, 140), 1)
            for c in range(GRID_COLS):
                cx0 = DATA_X0 + c * COL_W
                cx1 = cx0 + COL_W
                is_cur = self.focused and r == self.cur_row and c == self.cur_col
                border = ACCENT if is_cur else (90, 90, 90)
                cv2.rectangle(frame, (cx0, ry0), (cx1, ry1), TEXTBOX_BG, -1)
                cv2.rectangle(frame, (cx0, ry0), (cx1, ry1), border, 2 if is_cur else 1)
                text = self.grid[r][c] + ("_" if is_cur else "")
                cv2.putText(frame, text, (cx0 + 8, ry1 - 9),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)

        _draw_button(frame, RUN_BUTTON_RECT, "RUN", (60, 160, 60))
        _draw_button(frame, CLEAR_BUTTON_RECT, "CLEAR", (140, 60, 60))
        _draw_button(frame, RESET_VIEW_RECT, "RESET VIEW", (70, 70, 110))
        _draw_button(frame, CLEAR_POINTS_RECT, "CLEAR PTS", (140, 100, 40))
        mode_label = f"MODE: {'ARC' if self.point_mode == 'arc' else 'TURN+DRIVE'}"
        mode_color = (150, 90, 20) if self.point_mode == "arc" else (90, 90, 150)
        _draw_button(frame, MODE_TOGGLE_RECT, mode_label, mode_color)
        _draw_button(frame, EXPORT_BUTTON_RECT, "EXPORT CSV", (90, 90, 140))

        status_y = EXPORT_BUTTON_RECT[3]
        cv2.putText(frame, "Status:", (x0 + 20, status_y + 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1)
        cv2.putText(frame, self.status[:70], (x0 + 20, status_y + 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)

    def loop(self):
        while True:
            frame = self.draw()
            cv2.imshow(WIN_NAME, frame)
            key = cv2.waitKeyEx(15)  # unmasked, so Shift+Tab is distinguishable from Tab
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
