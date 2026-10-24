#!/usr/bin/env python3
"""
lidar_server.py -- runs on the Pi. Reads LD06 LIDAR packets from serial,
accumulates one complete 360 deg sweep at a time, and streams each finished
sweep to a client over plain TCP.

  python3 lidar_server.py
"""

import argparse
import socket
import struct
import threading
import time

import numpy as np
import serial

SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 230400
CONFIDENCE_MIN = 20

PACKET_LENGTH = 47
MEASUREMENT_COUNT = 12
MSG_FORMAT = "<xBHH" + "HB" * MEASUREMENT_COUNT + "HHB"

DEFAULT_LIDAR_PORT = 8092

# angle (deg), distance (m), confidence -- matches the client's unpack dtype
POINT_DTYPE = np.dtype([("angle", "<f4"), ("dist", "<f4"), ("conf", "u1")])


# ------------------------------------------------------------ LD06 parsing

def parse_packet(data):
    """Parse LD06 packet -> list of (angle_deg, distance_m, confidence)."""
    try:
        unpacked = struct.unpack(MSG_FORMAT, data)
    except struct.error:
        return []

    start_angle = unpacked[2] / 100.0
    stop_angle = unpacked[-3] / 100.0
    if stop_angle < start_angle:
        stop_angle += 360.0

    step = (stop_angle - start_angle) / (MEASUREMENT_COUNT - 1)
    raw = unpacked[3:-3]

    points = []
    for i in range(MEASUREMENT_COUNT):
        distance_mm = raw[i * 2]
        confidence = raw[i * 2 + 1]
        if distance_mm == 0 or confidence < CONFIDENCE_MIN:
            continue
        points.append((start_angle + step * i, distance_mm / 1000.0, confidence))
    return points


def read_packet(ser):
    """Read one valid packet from serial."""
    while True:
        b1 = ser.read(1)
        if not b1:
            continue
        if b1 == b'\x54':
            b2 = ser.read(1)
            if b2 == b'\x2C':
                rest = ser.read(PACKET_LENGTH - 2)
                if len(rest) == PACKET_LENGTH - 2:
                    return b'\x54\x2C' + rest


# ------------------------------------------------------------ networking

class SweepSource:
    """Continuously reads the LIDAR over serial, publishing one complete
    360 deg sweep at a time into a single shared slot."""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.lock = threading.Lock()
        self.sweep = None
        self.seq = 0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        current = []
        last_angle = None
        while self.running:
            packet = read_packet(self.ser)
            for angle_deg, dist_m, conf in parse_packet(packet):
                boundary = (last_angle is not None
                            and last_angle > 180 and angle_deg < 180)
                last_angle = angle_deg

                if boundary:
                    if current:
                        with self.lock:
                            self.sweep = current
                            self.seq += 1
                    current = [(angle_deg, dist_m, conf)]
                else:
                    current.append((angle_deg, dist_m, conf))

    def latest(self):
        with self.lock:
            return self.sweep, self.seq

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.ser.close()


def encode_sweep(points):
    arr = np.empty(len(points), dtype=POINT_DTYPE)
    arr["angle"] = [p[0] for p in points]
    arr["dist"] = [p[1] for p in points]
    arr["conf"] = [p[2] for p in points]
    return arr.tobytes()


def sweep_client_loop(conn, source):
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    last_seq = -1
    try:
        while True:
            sweep, seq = source.latest()
            if sweep is None or seq == last_seq:
                time.sleep(0.01)
                continue
            last_seq = seq
            data = encode_sweep(sweep)
            conn.sendall(struct.pack(">I", len(data)) + data)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
    finally:
        conn.close()


def accept_loop(sock, handler, *args):
    while True:
        conn, addr = sock.accept()
        print(f"client connected from {addr[0]}:{addr[1]} ({sock.getsockname()[1]})")
        threading.Thread(target=handler, args=(conn,) + args, daemon=True).start()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial-port", default=SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=BAUD_RATE)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--lidar-port", type=int, default=DEFAULT_LIDAR_PORT)
    args = ap.parse_args()

    print(f"opening {args.serial_port} at {args.baud} baud...")
    try:
        source = SweepSource(args.serial_port, args.baud)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        print("Try: sudo python3 lidar_server.py")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.lidar_port))
    sock.listen(1)

    print(f"lidar   listening on {args.host}:{args.lidar_port}")

    try:
        accept_loop(sock, sweep_client_loop, source)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()


if __name__ == "__main__":
    main()
