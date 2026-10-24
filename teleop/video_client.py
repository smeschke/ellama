#!/usr/bin/env python3
"""
llama_teleop_client_video_only.py -- runs on YOUR laptop, not the Pi.
Connects to llama_teleop_server_video_only.py over the network and displays
the live feed locally. Video only -- no control link, no click/button
commands.

  python3 llama_teleop_client_video_only.py --host <pi-ip-or-hostname>
"""

import argparse
import socket
import struct
import threading
import time

import cv2
import numpy as np

DEFAULT_VIDEO_PORT = 8090


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


class VideoReceiver:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.lock = threading.Lock()
        self.frame = None
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
                frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8),
                                      cv2.IMREAD_COLOR)
                if frame is not None:
                    with self.lock:
                        self.frame = frame
        except OSError:
            pass
        self.running = False

    def latest(self):
        with self.lock:
            return self.frame

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="Pi hostname or IP")
    ap.add_argument("--video-port", type=int, default=DEFAULT_VIDEO_PORT)
    args = ap.parse_args()

    print(f"connecting to {args.host}:{args.video_port} (video)")
    video = VideoReceiver(args.host, args.video_port)

    win = "llama_teleop (video only)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    print("q = quit")

    try:
        while True:
            if not video.running:
                print("connection lost")
                break

            frame = video.latest()
            if frame is None:
                time.sleep(0.01)
                continue

            cv2.imshow(win, frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
    finally:
        video.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
