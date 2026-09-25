#!/usr/bin/env python3
"""
Send a CSV of drive commands to the ESP32 running
firmware/computer_bridge/computer_bridge.ino, over serial. Same wire protocol as
sim_sender.py's SEND button ("<left> <right>\\n" held for time_ms, then
"stop" at the end), just scriptable so a path doesn't need retyping into
the MDI box.

CSV format: one command per row, "pwml,pwmr,time_ms" (a header row is
fine -- any row that doesn't parse as three ints is skipped).

Usage:
    python3 sim/send_csv.py [path/to/commands.csv] [--port /dev/ttyUSB0]

    python3 sim/send_csv.py                     # sends square_12in.csv
    python3 sim/send_csv.py my_path.csv
    python3 sim/send_csv.py my_path.csv --port /dev/ttyUSB0

Safety: always sends "stop" when the script finishes, is interrupted
(Ctrl+C), or errors out, since computer_bridge.ino repeats the last command
forever until told otherwise.
"""

import argparse
import csv
import re
import sys
import time

import serial
import serial.tools.list_ports as list_ports

BAUD_RATE = 115200
RESET_SETTLE_S = 2.0  # opening the port resets the ESP32; wait for it to boot + join ESP-NOW

_IGNORE_DEVICE_RE = re.compile(r"^/dev/ttyS\d+$")   # Linux's built-in 16550 headers, never an ESP32
_IGNORE_DESC_RE = re.compile(r"bluetooth", re.I)     # Bluetooth SPP ports


def list_candidate_ports():
    names = []
    for p in list_ports.comports():
        if _IGNORE_DEVICE_RE.match(p.device):
            continue
        if _IGNORE_DESC_RE.search(p.description or ""):
            continue
        names.append(p.device)
    return sorted(names)


def pick_port(explicit):
    if explicit:
        return explicit
    candidates = list_candidate_ports()
    if not candidates:
        sys.exit("No serial ports found. Plug in the ESP32 and try again, or pass --port explicitly.")
    if len(candidates) > 1:
        print("Multiple candidate ports found, using the first. Pass --port to pick a different one:")
        for c in candidates:
            print(" ", c)
    return candidates[0]


def load_commands(csv_path):
    commands = []
    with open(csv_path, newline="") as f:
        for row in csv.reader(f):
            if len(row) < 3:
                continue
            try:
                pwml, pwmr, time_ms = int(row[0]), int(row[1]), int(row[2])
            except ValueError:
                continue  # header row, blank line, comment, etc.
            commands.append((pwml, pwmr, time_ms))
    if not commands:
        sys.exit(f"No commands found in {csv_path}")
    return commands


def main():
    parser = argparse.ArgumentParser(description="Send a CSV drive path to the robot over serial.")
    parser.add_argument("csv_path", nargs="?", default="square_12in.csv",
                         help="CSV file of pwml,pwmr,time_ms rows (default: square_12in.csv)")
    parser.add_argument("--port", help="Serial port (default: auto-detect)")
    args = parser.parse_args()

    commands = load_commands(args.csv_path)
    port = pick_port(args.port)

    print(f"Connecting to {port}...")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    try:
        time.sleep(RESET_SETTLE_S)
        print("Connected. Sending path...")
        for i, (pwml, pwmr, time_ms) in enumerate(commands, start=1):
            print(f"[{i}/{len(commands)}] {pwml} {pwmr}  for {time_ms} ms")
            ser.write(f"{pwml} {pwmr}\n".encode("ascii"))
            time.sleep(time_ms / 1000.0)
        print("Path complete.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            ser.write(b"stop\n")
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        print("Robot stopped.")


if __name__ == "__main__":
    main()
