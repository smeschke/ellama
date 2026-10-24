#!/usr/bin/env python3
"""
lidar_client.py -- runs on YOUR laptop, not the Pi. Connects to
lidar_server.py over the network and displays each completed 360 deg LIDAR
sweep locally. Press 'q' or Escape to quit.

  python3 lidar_client.py --host <pi-ip-or-hostname>
"""

import argparse
import socket
import struct
import threading
import time

import cv2
import numpy as np

DEFAULT_LIDAR_PORT = 8092

MAX_RANGE_M = 4.0
MARKER_RADIUS = 2
WINDOW_WIDTH = 1024
WINDOW_HEIGHT = 768

# angle (deg), distance (m), confidence -- matches the server's encode dtype
POINT_DTYPE = np.dtype([("angle", "<f4"), ("dist", "<f4"), ("conf", "u1")])


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


class SweepReceiver:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.lock = threading.Lock()
        self.sweep = None
        self.seq = 0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        try:
            while self.running:
                header = recv_exact(self.sock, 4)
                if header is None:
                    break
                (length,) = struct.unpack(">I", header)
                data = recv_exact(self.sock, length)
                if data is None:
                    break
                arr = np.frombuffer(data, dtype=POINT_DTYPE)
                with self.lock:
                    self.sweep = arr
                    self.seq += 1
        except OSError:
            pass
        self.running = False

    def latest(self):
        with self.lock:
            return self.sweep, self.seq

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def confidence_to_color(conf):
    """Map confidence to grayscale BGR."""
    v = int(conf)
    return (v, v, v)


def draw_sweep(sweep, sweep_count, cx, cy, scale, ring_radii):
    frame = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)

    for r in ring_radii:
        if r < min(cx, cy):
            cv2.circle(frame, (cx, cy), r, (40, 40, 40), 1)

    for angle, dist, conf in sweep:
        if dist > MAX_RANGE_M:
            continue
        rad = np.radians(angle)
        px = int(cx - dist * np.sin(rad) * scale)
        py = int(cy + dist * np.cos(rad) * scale)
        if 0 <= px < WINDOW_WIDTH and 0 <= py < WINDOW_HEIGHT:
            cv2.circle(frame, (px, py), MARKER_RADIUS, confidence_to_color(conf), -1)

    cv2.circle(frame, (cx, cy), 4, (100, 100, 100), -1)
    cv2.line(frame, (cx - 30, cy), (cx + 30, cy), (80, 80, 80), 1)
    cv2.line(frame, (cx, cy - 30), (cx, cy + 30), (80, 80, 80), 1)

    cv2.putText(frame, f"Sweep #{sweep_count} ({len(sweep)} pts)",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="Pi hostname or IP")
    ap.add_argument("--lidar-port", type=int, default=DEFAULT_LIDAR_PORT)
    args = ap.parse_args()

    print(f"connecting to {args.host}:{args.lidar_port} (lidar)")
    receiver = SweepReceiver(args.host, args.lidar_port)

    win = "LD06 LIDAR - Complete Sweeps"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, WINDOW_WIDTH, WINDOW_HEIGHT)
    print("q = quit")

    cx = WINDOW_WIDTH // 2
    cy = WINDOW_HEIGHT // 2
    scale = min(cx, cy) / MAX_RANGE_M
    ring_radii = [int(scale * d) for d in [1.0, 2.0, 3.0, 4.0]]

    last_seq = -1

    try:
        while True:
            if not receiver.running:
                print("connection lost")
                break

            sweep, seq = receiver.latest()
            if sweep is None or seq == last_seq:
                time.sleep(0.005)
                k = cv2.waitKey(1) & 0xFF
                if k in (ord('q'), 27):
                    break
                continue

            last_seq = seq
            print(f"Sweep #{seq}: {len(sweep)} points")

            frame = draw_sweep(sweep, seq, cx, cy, scale, ring_radii)
            cv2.imshow(win, frame)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
    finally:
        receiver.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
