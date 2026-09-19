#!/usr/bin/env python3
"""
llama_teleop_server_video_only_single.py -- runs on the Pi. Captures the
camera and streams JPEG frames to a client over plain TCP. Video only -- no
control link, no serial/motion logic.

  python3 llama_teleop_server_video_only_single.py
"""

import argparse
import socket
import struct
import threading
import time

import cv2

CAP_W, CAP_H = 640, 480

DEFAULT_VIDEO_PORT = 8090

JPEG_QUALITY = 80


# ------------------------------------------------------------ networking

class FrameSource:
    """Continuously grabs frames from the camera into a single shared slot."""

    def __init__(self, cam_index):
        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit(f"could not open camera {cam_index}")
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self.lock:
                self.frame = frame
                self.seq += 1

    def latest(self):
        with self.lock:
            return self.frame, self.seq

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


def send_all(conn, data):
    conn.sendall(data)


def video_client_loop(conn, source, quality):
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    last_seq = -1
    try:
        while True:
            frame, seq = source.latest()
            if frame is None or seq == last_seq:
                time.sleep(0.005)
                continue
            last_seq = seq
            ok, buf = cv2.imencode(".jpg", frame,
                                    [cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                continue
            data = buf.tobytes()
            send_all(conn, struct.pack(">I", len(data)) + data)
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
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--video-port", type=int, default=DEFAULT_VIDEO_PORT)
    ap.add_argument("--jpeg-quality", type=int, default=JPEG_QUALITY)
    args = ap.parse_args()

    source = FrameSource(args.cam)

    video_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    video_sock.bind((args.host, args.video_port))
    video_sock.listen(1)

    print(f"video   listening on {args.host}:{args.video_port}")

    try:
        accept_loop(video_sock, video_client_loop, source, args.jpeg_quality)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()


if __name__ == "__main__":
    main()
