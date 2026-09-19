#!/usr/bin/env python3
"""
robot_command_client.py -- runs on YOUR laptop, not the Pi. Connects to
robot_command_server.py over the network and drives it with the keyboard.
No video here -- watch the robot however you're streaming it separately
(phone, Zoom, a second script, whatever) and just use this terminal to
steer.

Keys (hold to keep moving, terminal must have focus):
  w        forward (hold to keep driving)
  s        reverse
  a / d    spin left / right in place (holding ramps up intensity)
  up/down  increase / decrease speed
  space    stop
  q        quit

  Only one of w/s/a/d is active at a time -- there's no mixing, you're
  either driving straight or spinning in place.

  python3 robot_command_client.py --host <pi-ip-or-hostname>
"""

import argparse
import curses
import socket
import threading
import time

DEFAULT_CONTROL_PORT = 8091

TICK_S = 0.08           # how often we re-send the held command
HOLD_TIMEOUT_S = 0.25   # if no repeat keypress arrives within this, treat key as released
SPIN_RAMP_S = 0.8       # time held to ramp turn from STEER_MIN to full intensity
STEER_MIN = 0.5         # even a quick tap of a/d gives at least this much turn

SPEED_MIN = 2
SPEED_MAX = 128         # half of the ESP32's 255 ceiling -- top speed was too hot
SPEED_STEP = 2
SPEED_DEFAULT = 64

class ControlLink:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.status = "idle"
        self.moving = False
        self.running = True
        self.last_status_at = time.time()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        f = self.sock.makefile("r")
        try:
            for line in f:
                line = line.strip()
                if not line.startswith("STATUS "):
                    continue
                payload = line[len("STATUS "):]
                moving, _, text = payload.partition("|")
                self.moving = moving == "1"
                self.status = text
                self.last_status_at = time.time()
        except OSError:
            pass
        self.running = False

    def send(self, line):
        try:
            self.sock.sendall((line + "\n").encode())
            return True
        except OSError:
            return False

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def bar(value, lo, hi, width=20):
    frac = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    filled = int(round(frac * width))
    return "[" + "#" * filled + "-" * (width - filled) + "]"


def run(stdscr, control):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(int(TICK_S * 1000))
    stdscr.keypad(True)

    held = {"key": None, "last_seen": 0.0, "held_since": 0.0}
    speed = SPEED_DEFAULT
    last_cmd_sent = "-"
    last_cmd_at = 0.0

    def note(key):
        now = time.time()
        if held["key"] != key:
            held["key"] = key
            held["held_since"] = now
        held["last_seen"] = now

    while True:
        ch = stdscr.getch()
        now = time.time()

        if ch != -1:
            if ch == curses.KEY_UP:
                speed = min(SPEED_MAX, speed + SPEED_STEP)
            elif ch == curses.KEY_DOWN:
                speed = max(SPEED_MIN, speed - SPEED_STEP)
            else:
                c = chr(ch) if 0 <= ch < 256 else ""
                if c == "q":
                    control.send("STOP")
                    return
                if c == " ":
                    control.send("STOP")
                    last_cmd_sent = "STOP"
                    last_cmd_at = now
                    held["key"] = None
                elif c in ("w", "s", "a", "d"):
                    note(c)

        # treat key as released if we haven't seen a repeat recently
        if held["key"] and (now - held["last_seen"]) > HOLD_TIMEOUT_S:
            held["key"] = None

        key = held["key"]
        if key == "w":
            cmd = f"FWD {speed}"
            control.send(cmd)
            last_cmd_sent, last_cmd_at = cmd, now
        elif key == "s":
            cmd = f"REV {speed}"
            control.send(cmd)
            last_cmd_sent, last_cmd_at = cmd, now
        elif key in ("a", "d"):
            held_dur = now - held["held_since"]
            frac = min(1.0, held_dur / SPIN_RAMP_S)
            mag = STEER_MIN + (1.0 - STEER_MIN) * frac
            nx = mag if key == "a" else -mag
            cmd = f"SPIN {nx:.2f} {speed}"
            control.send(cmd)
            last_cmd_sent, last_cmd_at = cmd, now

        stale_status = control.running and (now - control.last_status_at) > 1.0

        max_y, max_x = stdscr.getmaxyx()

        def put(r, text):
            if r >= max_y:
                return
            try:
                stdscr.addstr(r, 0, text[:max(0, max_x - 1)])
            except curses.error:
                pass

        stdscr.erase()
        put(0, "robot_command_client -- w/s drive, a/d spin, "
               "up/down speed, space stop, q quit")
        put(1, f"server   : {control.host}:{control.port}")

        row = 3
        put(row, f"key held : {key or '-'}")
        row += 1
        put(row, f"speed    : {speed:3d}/{SPEED_MAX}  {bar(speed, SPEED_MIN, SPEED_MAX)}")
        row += 2

        conn_state = "connected" if control.running else "DISCONNECTED"
        put(row, f"link     : {conn_state}" +
            ("  (status feed stale)" if stale_status else ""))
        row += 1
        put(row, f"robot    : {control.status}")
        row += 1
        moving_str = "MOVING" if control.moving else "stopped"
        put(row, f"moving   : {moving_str}")
        row += 2

        age = now - last_cmd_at if last_cmd_at else None
        age_str = f"{age:4.1f}s ago" if age is not None else "-"
        put(row, f"last cmd : {last_cmd_sent}  ({age_str})")
        row += 2

        if not control.running:
            put(row, "connection lost, press q to quit")
            row += 1

        stdscr.refresh()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="Pi hostname or IP")
    ap.add_argument("--control-port", type=int, default=DEFAULT_CONTROL_PORT)
    args = ap.parse_args()

    print(f"connecting to {args.host}:{args.control_port} ...")
    control = ControlLink(args.host, args.control_port)
    try:
        curses.wrapper(run, control)
    finally:
        control.send("STOP")
        control.close()


if __name__ == "__main__":
    main()
