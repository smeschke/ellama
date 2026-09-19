#!/usr/bin/env python3
"""
robot_command_server.py -- runs on the Pi. Receives drive commands
("FWD", "REV", "SPIN <nx>", "STOP") over a plain TCP socket and forwards
motion to the ESP-NOW sender sketch over serial at 20 Hz.

No camera/video code here at all -- video is your problem now (phone,
Zoom, a second stream, whatever). This just drives the motors.

Motion logic (Link, Runner, spin_plan) is unchanged from llama_teleop.py --
it still talks "<left> <right>\n" to the ESP32 sender over serial.

  python3 robot_command_server.py --port /dev/ttyUSB0
  python3 robot_command_server.py --dry-run
"""

import argparse
import glob
import select
import socket
import threading
import time

try:
    import serial
except ImportError:
    serial = None

DEFAULT_CONTROL_PORT = 8091

SPIN_PWM_CENTER = 150
SPIN_PWM_EDGE = 255
SPIN_TIME_CENTER = 0.0375
SPIN_TIME_EDGE = 0.125
CENTER_BAND = 1.0 / 3.0
PWM_SCALE = 1.0

MAX_PWM = 255              # hard ceiling -- client speed tops out here
FORWARD_MS = 125           # each FWD command drives for this long
REVERSE_MS = 150           # each REV command drives for this long

SILENCE_TIMEOUT_S = 1.0    # no command received for this long -> PWM 0,0

KEEPALIVE_S = 0.15
BOOT_WAIT_S = 2.0

STATUS_PUSH_S = 0.1


# ---------------------------------------------------------------- plan

def pwm(value):
    return int(round(value * PWM_SCALE))


def spin_plan(nx):
    """nx in [-1, 1]: sign is direction, magnitude is how hard/long to spin."""
    nx = max(-1.0, min(1.0, nx))
    off = abs(nx)
    t = round(SPIN_TIME_CENTER + (SPIN_TIME_EDGE - SPIN_TIME_CENTER) * off, 2)
    ramp = 0.0 if off <= CENTER_BAND else (off - CENTER_BAND) / (1.0 - CENTER_BAND)
    p = pwm(SPIN_PWM_CENTER + (SPIN_PWM_EDGE - SPIN_PWM_CENTER) * ramp)
    if nx >= 0:
        return [(p, -p, t, "spin right")]
    return [(-p, p, t, "spin left")]


def print_plan(segs, label):
    print()
    print("-" * 44)
    print(f"{label}    [{time.strftime('%H:%M:%S')}]")
    if not segs:
        print("  (no motion)")
    for i, (l, r, t, seg_label) in enumerate(segs, 1):
        print(f"  {i}. LPWM {l:4d}  RPWM {r:4d}  {t:4.2f} s   ({seg_label})")
    print("-" * 44)


# ---------------------------------------------------------------- link

class Link:
    """Serial connection to the ESP-NOW sender sketch."""

    def __init__(self, port, baud=115200, dry_run=False):
        self.dry_run = dry_run or serial is None
        self.ser = None
        self.lock = threading.Lock()
        if self.dry_run:
            if serial is None and not dry_run:
                print("pyserial not installed -- running dry. "
                      "pip3 install pyserial")
            print("[dry run] commands will only be printed")
            return
        if port == "auto":
            found = (sorted(glob.glob("/dev/ttyUSB*"))
                     + sorted(glob.glob("/dev/ttyACM*")))
            if not found:
                raise SystemExit("no /dev/ttyUSB* or /dev/ttyACM* found; "
                                 "pass --port or use --dry-run")
            port = found[0]
        self.ser = serial.Serial(port, baud, timeout=0.1)
        print(f"serial: {port} @ {baud}, waiting {BOOT_WAIT_S}s for ESP32 boot")
        time.sleep(BOOT_WAIT_S)
        self.ser.reset_input_buffer()

    def send(self, left, right):
        left = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))
        line = f"{left} {right}\n"
        if self.dry_run:
            print(f"  >> {line.strip()}")
            return
        with self.lock:
            self.ser.write(line.encode())
            self.ser.flush()

    def stop(self):
        self.send(0, 0)

    def close(self):
        try:
            self.stop()
            time.sleep(0.05)
        finally:
            if self.ser:
                self.ser.close()


# ------------------------------------------------------------- runner

class Runner:
    def __init__(self, link):
        self.link = link
        self.abort = threading.Event()
        self.thread = None
        self.status = "idle"

    @property
    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def run(self, segs):
        self.cancel()
        self.abort.clear()
        self.thread = threading.Thread(target=self._worker, args=(segs,),
                                       daemon=True)
        self.thread.start()

    def run_one(self, l, r, seconds, label):
        self.run([(l, r, seconds, label)])

    def cancel(self):
        if self.busy:
            self.abort.set()
            self.thread.join(timeout=1.0)
        self.link.stop()
        self.status = "idle"

    def _worker(self, segs):
        try:
            for l, r, t, label in segs:
                self.status = f"{label} L{l} R{r} {t:.2f}s"
                end = time.time() + t
                while time.time() < end:
                    if self.abort.is_set():
                        return
                    self.link.send(l, r)
                    time.sleep(min(KEEPALIVE_S, max(0.0, end - time.time())))
        finally:
            self.link.stop()
            self.status = "idle"


# ------------------------------------------------------------ networking

def send_all(conn, data):
    conn.sendall(data)


def control_client_loop(conn, runner, state):
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def status_pusher(stop_evt):
        try:
            while not stop_evt.is_set():
                moving = 1 if runner.busy else 0
                line = f"STATUS {moving}|{runner.status}\n"
                send_all(conn, line.encode())
                time.sleep(STATUS_PUSH_S)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    stop_evt = threading.Event()
    pusher = threading.Thread(target=status_pusher, args=(stop_evt,), daemon=True)
    pusher.start()

    def handle(line):
        state["last_cmd"] = time.time()
        parts = line.split()
        if not parts:
            return
        cmd = parts[0]
        if cmd == "FWD":
            speed = float(parts[1]) if len(parts) >= 2 else MAX_PWM
            speed = max(0.0, min(MAX_PWM, speed))
            p = int(round(speed))
            label = f"forward speed={speed:.0f}"
            print_plan([(-p, -p, FORWARD_MS / 1000.0, label)], label)
            runner.run_one(-p, -p, FORWARD_MS / 1000.0, label)
        elif cmd == "REV":
            speed = float(parts[1]) if len(parts) >= 2 else MAX_PWM
            speed = max(0.0, min(MAX_PWM, speed))
            rev_pwm = int(round(speed))
            label = f"reverse speed={speed:.0f}"
            print_plan([(rev_pwm, rev_pwm, REVERSE_MS / 1000.0, label)], label)
            runner.run_one(rev_pwm, rev_pwm, REVERSE_MS / 1000.0, label)
        elif cmd == "SPIN" and len(parts) >= 2:
            nx = float(parts[1])
            speed = float(parts[2]) if len(parts) >= 3 else MAX_PWM
            speed = max(0.0, min(MAX_PWM, speed))
            scale = min(1.0, (speed * 2) / MAX_PWM)  # turning pwm runs 2x speed's
            segs = spin_plan(nx)
            segs = [(int(round(l * scale)), int(round(r * scale)), t, lbl)
                    for l, r, t, lbl in segs]
            print_plan(segs, f"spin nx={nx:+.2f} speed={speed:.0f}")
            runner.run(segs)
        elif cmd == "STOP":
            runner.cancel()
            print("STOP")

    buf = b""
    try:
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk

            # drain whatever else is already sitting in the socket, non-blocking,
            # so a burst of queued commands collapses to just the newest one
            while True:
                ready, _, _ = select.select([conn], [], [], 0)
                if not ready:
                    break
                more = conn.recv(4096)
                if not more:
                    break
                buf += more

            *complete, buf = buf.split(b"\n")
            lines = [ln.decode().strip() for ln in complete if ln.strip()]
            if not lines:
                continue
            for stale in lines[:-1]:
                state["last_cmd"] = time.time()  # still counts as a live client
            handle(lines[-1])
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
    finally:
        stop_evt.set()
        runner.cancel()  # safety: control link dropped -> stop the robot
        conn.close()


def silence_watchdog(link, state):
    """If no command has been received for SILENCE_TIMEOUT_S, force PWM 0,0."""
    while True:
        if time.time() - state["last_cmd"] > SILENCE_TIMEOUT_S:
            link.stop()
        time.sleep(KEEPALIVE_S)


def accept_loop(sock, handler, *args):
    while True:
        conn, addr = sock.accept()
        print(f"client connected from {addr[0]}:{addr[1]} ({sock.getsockname()[1]})")
        threading.Thread(target=handler, args=(conn,) + args, daemon=True).start()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto",
                    help="serial port of the ESP32 sender, or 'auto'")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--control-port", type=int, default=DEFAULT_CONTROL_PORT)
    args = ap.parse_args()

    link = Link(args.port, args.baud, args.dry_run)
    runner = Runner(link)
    state = {"last_cmd": time.time()}

    control_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    control_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    control_sock.bind((args.host, args.control_port))
    control_sock.listen(1)

    print(f"control listening on {args.host}:{args.control_port}")

    threading.Thread(target=silence_watchdog, args=(link, state), daemon=True).start()

    try:
        accept_loop(control_sock, control_client_loop, runner, state)
    except KeyboardInterrupt:
        pass
    finally:
        runner.cancel()
        link.close()


if __name__ == "__main__":
    main()
