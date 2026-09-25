#!/usr/bin/env python3
"""
Live read-only viewer + calibrator for the two wheel encoders and the IMU.
Never sends anything to the robot -- drive with the stick, watch the numbers,
and use them to calibrate.

Setup: computer_bridge.ino on the ESP32 plugged into this computer (it is
listen-only until sent a drive command, which this script never does), with
robot_encoders_as5600 and a robot_imu_* sketch running on the robot.

The window shows a rolling history of per-wheel distance and speed, IMU
pitch/roll/yaw (plus the encoder-derived heading, dashed), and accel. The big
readout at the top is distance and heading SINCE THE LAST ZERO.

Drive-a-foot workflow:
  1. Put the robot at a mark and press  z  in the plot window (or type z in
     this terminal). Distance and heading reset to 0.
  2. Drive it a known distance with the stick, e.g. one foot.
  3. Read the distance off the screen. If it's wrong, type the distance you
     actually measured (inches) in this terminal:   d 12
     That rescales the wheel diameter in calibration/config.json so the screen
     matches, and applies it right away. Repeat z / drive / check until it
     reads right.
  4. Turn calibration: press z, spin the robot in place through a measured
     angle (e.g. mark a 90 or 180 degree turn), then type the real angle:  t 90
     That rescales track_width_in. Do the distance calibration first -- the
     turn estimate depends on it.

Terminal commands:  z  |  d <inches>  |  t <degrees>  |  show  |  help
Close the window or Ctrl+C to quit. Nothing is recorded.

Limits: distance calibration assumes the robot drove straight (both wheels
scaled together); the IMU yaw shown is gyro-integrated and drifts slowly.

Usage:
    python3 calibration/live_view.py
    python3 calibration/live_view.py --port /dev/ttyUSB0 --window 30

Requires: matplotlib
"""

import argparse
import collections
import queue
import sys
import threading
import time

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from bridge import (BAUD_RATE, RESET_SETTLE_S, SerialReader, EncoderState,
                    OrientationState, load_config, save_config, parse_line, pick_port)

INK_PRIMARY = "#0b0b0b"
GRIDLINE = "#e1e0d9"
SURFACE = "#fcfcfb"
CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]

MIN_CAL_DISTANCE_IN = 2.0   # refuse to calibrate off a tiny move
MIN_CAL_TURN_DEG = 10.0


def main():
    parser = argparse.ArgumentParser(description="Live read-only encoder + IMU viewer and calibrator.")
    parser.add_argument("--port", help="Serial port for the bridge ESP32 (default: auto-detect)")
    parser.add_argument("--window", type=float, default=20.0, help="Seconds of history to show (default: 20)")
    args = parser.parse_args()

    cfg = load_config()
    port = pick_port(args.port)
    print(f"Connecting to {port}...")
    ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    reader = SerialReader(ser)
    reader.start()
    time.sleep(RESET_SETTLE_S)
    print("Connected. Commands: z | d <inches> | t <degrees> | show | help.  Close window or Ctrl+C to quit.")

    enc = EncoderState(cfg)
    ori = OrientationState()
    t0 = time.monotonic()

    def buf():
        return collections.deque()

    t_enc, dist_l, dist_r, speed_l, speed_r, enc_hdg = (buf() for _ in range(6))
    t_imu, pitch, roll, yaw, acc_x, acc_y, acc_z = (buf() for _ in range(7))
    all_bufs = [t_enc, dist_l, dist_r, speed_l, speed_r, enc_hdg,
                t_imu, pitch, roll, yaw, acc_x, acc_y, acc_z]
    last_seen = {"enc": None, "imu": None}

    # Terminal commands are read on a thread so typing never freezes the plot.
    cmd_q = queue.Queue()

    def stdin_loop():
        for line in sys.stdin:
            cmd_q.put(line.strip())

    threading.Thread(target=stdin_loop, daemon=True).start()

    fig, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True, facecolor=SURFACE)
    ax_d, ax_s, ax_o, ax_a = axes
    (l_dist_l,) = ax_d.plot([], [], color=CAT[0], linewidth=2, label="left")
    (l_dist_r,) = ax_d.plot([], [], color=CAT[1], linewidth=2, label="right")
    (l_speed_l,) = ax_s.plot([], [], color=CAT[0], linewidth=1.5, label="left")
    (l_speed_r,) = ax_s.plot([], [], color=CAT[1], linewidth=1.5, label="right")
    (l_pitch,) = ax_o.plot([], [], color=CAT[0], linewidth=2, label="pitch")
    (l_roll,) = ax_o.plot([], [], color=CAT[1], linewidth=2, label="roll")
    (l_yaw,) = ax_o.plot([], [], color=CAT[2], linewidth=2, label="yaw (imu, drifts)")
    (l_ehdg,) = ax_o.plot([], [], color=CAT[3], linewidth=2, linestyle="--", label="heading (encoders)")
    (l_ax,) = ax_a.plot([], [], color=CAT[0], linewidth=1.5, label="x")
    (l_ay,) = ax_a.plot([], [], color=CAT[1], linewidth=1.5, label="y")
    (l_az,) = ax_a.plot([], [], color=CAT[2], linewidth=1.5, label="z")

    ax_d.set_ylabel("distance (in)")
    ax_s.set_ylabel("speed (in/s)")
    ax_o.set_ylabel("degrees")
    ax_a.set_ylabel("accel (g)")
    ax_a.set_xlabel("time (s)")
    for ax in axes:
        ax.legend(frameon=False, loc="upper left")
        ax.set_facecolor(SURFACE)
        ax.grid(color=GRIDLINE, linewidth=1)
        ax.tick_params(colors=INK_PRIMARY)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    readout = fig.text(0.5, 0.985, "waiting for data...", ha="center", va="top",
                       fontsize=12, family="monospace", color=INK_PRIMARY)

    def do_zero():
        enc.zero()
        ori.zero_yaw()
        for b in all_bufs:
            b.clear()
        print("-> zeroed distance, heading and yaw")

    def clear_history():
        for b in all_bufs:
            b.clear()

    def do_calibrate_distance(measured_in):
        travelled = abs(enc.dist_avg)
        if travelled < MIN_CAL_DISTANCE_IN:
            print(f"-> only {travelled:.2f} in since zero; drive further (>= {MIN_CAL_DISTANCE_IN:g} in) before calibrating")
            return
        if measured_in <= 0:
            print("-> measured distance must be positive")
            return
        old = cfg["wheel_diameter_in"]
        cfg["wheel_diameter_in"] = old * measured_in / travelled
        save_config(cfg)
        clear_history()
        print(f"-> screen said {travelled:.2f} in, you measured {measured_in:g} in: "
              f"wheel_diameter_in {old:.3f} -> {cfg['wheel_diameter_in']:.3f} (saved). "
              f"Press z and drive again to confirm.")

    def do_calibrate_turn(actual_deg):
        turned = abs(enc.heading_deg)
        if turned < MIN_CAL_TURN_DEG:
            print(f"-> encoders only show {turned:.1f} deg since zero; spin further (>= {MIN_CAL_TURN_DEG:g} deg)")
            return
        if actual_deg <= 0:
            print("-> actual angle must be positive")
            return
        old = cfg["track_width_in"]
        cfg["track_width_in"] = old * turned / actual_deg
        save_config(cfg)
        clear_history()
        print(f"-> encoders said {turned:.1f} deg (imu yaw {abs(ori.yaw):.1f}), actual {actual_deg:g}: "
              f"track_width_in {old:.2f} -> {cfg['track_width_in']:.2f} (saved). "
              f"Press z and spin again to confirm.")

    def handle_command(line):
        parts = line.split()
        if not parts:
            return
        cmd = parts[0].lower()
        try:
            if cmd == "z":
                do_zero()
            elif cmd == "d" and len(parts) == 2:
                do_calibrate_distance(float(parts[1]))
            elif cmd == "t" and len(parts) == 2:
                do_calibrate_turn(float(parts[1]))
            elif cmd == "show":
                print(f"-> config: {cfg}")
            else:
                print("commands: z | d <measured inches> | t <actual degrees> | show")
        except ValueError:
            print("-> couldn't read that number")

    def on_key(event):
        if event.key == "z":
            do_zero()

    fig.canvas.mpl_connect("key_press_event", on_key)

    def drain():
        while True:
            try:
                raw = reader.q.get_nowait()
            except queue.Empty:
                return
            kind, data = parse_line(raw)
            t = time.monotonic() - t0
            if kind == "ENC":
                sl, sr = enc.update(*data)
                t_enc.append(t); dist_l.append(enc.dist_l); dist_r.append(enc.dist_r)
                speed_l.append(sl if sl is not None else 0.0)
                speed_r.append(sr if sr is not None else 0.0)
                enc_hdg.append(enc.heading_deg)
                last_seen["enc"] = t
            elif kind == "IMU":
                agx, agy, agz, *_ = ori.update(*data)
                t_imu.append(t)
                pitch.append(ori.pitch); roll.append(ori.roll); yaw.append(ori.yaw)
                acc_x.append(agx); acc_y.append(agy); acc_z.append(agz)
                last_seen["imu"] = t

    def trim(now, *groups):
        for tq, *qs in groups:
            while tq and tq[0] < now - args.window:
                tq.popleft()
                for q in qs:
                    q.popleft()

    def update(_frame):
        while True:
            try:
                handle_command(cmd_q.get_nowait())
            except queue.Empty:
                break
        drain()
        now = time.monotonic() - t0
        trim(now, (t_enc, dist_l, dist_r, speed_l, speed_r, enc_hdg),
             (t_imu, pitch, roll, yaw, acc_x, acc_y, acc_z))

        l_dist_l.set_data(t_enc, dist_l)
        l_dist_r.set_data(t_enc, dist_r)
        l_speed_l.set_data(t_enc, speed_l)
        l_speed_r.set_data(t_enc, speed_r)
        l_pitch.set_data(t_imu, pitch)
        l_roll.set_data(t_imu, roll)
        l_yaw.set_data(t_imu, yaw)
        l_ehdg.set_data(t_enc, enc_hdg)
        l_ax.set_data(t_imu, acc_x)
        l_ay.set_data(t_imu, acc_y)
        l_az.set_data(t_imu, acc_z)

        ax_d.set_xlim(max(0, now - args.window), max(args.window, now))
        for ax in axes:
            ax.relim()
            ax.autoscale_view(scalex=False)

        def age(k):
            return "no data" if last_seen[k] is None else f"{now - last_seen[k]:.1f}s ago"

        if last_seen["enc"] is not None:
            d = enc.dist_avg
            line1 = (f"SINCE ZERO  {d:+8.2f} in  ({d / 12:+.3f} ft)   "
                     f"L {enc.dist_l:+.2f}  R {enc.dist_r:+.2f} in")
            line2 = (f"heading enc {enc.heading_deg:+7.1f} deg   imu yaw {ori.yaw:+7.1f} deg   "
                     f"pitch {ori.pitch:+.1f} roll {ori.roll:+.1f}")
        else:
            line1 = "SINCE ZERO  (no encoder data yet)"
            line2 = f"imu yaw {ori.yaw:+.1f} deg"
        readout.set_text(f"{line1}\n{line2}\n"
                         f"enc {age('enc')} | imu {age('imu')} | z = zero, type d/t in terminal to calibrate")
        return []

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        ser.close()
    return anim


if __name__ == "__main__":
    main()
