#!/usr/bin/env python3
"""Drive the robot a target distance at a constant PWM, pause, then return to start via
encoder feedback, over serial to computer_bridge.ino.

Usage: python3 toandfro.py <distance_inches> <pwm>
  distance_inches  how far to travel before turning around (magnitude; uses the
                    empirical COUNTS_PER_INCH calibration below, not a gear calc)
  pwm              constant drive PWM for the outbound leg (sign sets direction);
                    the return leg automatically uses the opposite sign

Outbound leg: drives at <pwm> until measured travel (from encoder counts, not time)
reaches distance_inches, then stops and pauses 1s. Return leg: drives at -<pwm> until
the encoder crosses back through its starting count, then stops. Overrunning a bit on
either end is expected -- this is a feedback loop over an ESP-NOW link with real
latency, not a precision positioning script.

COUNTS_PER_INCH = 468 is an empirical fit from 4 hand-measured pulses on the 10" wheel
(measured 455-493 counts/inch, ~468 average). It does NOT match the 95:15 gear-ratio
math (~826 counts/inch predicted) -- that mismatch is still unresolved, so treat
distances from this script as approximate.
"""
import sys
import time
import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200
BOOT_WAIT_S = 2.5
MAX_PWM_CEILING = 125
COUNTS_PER_INCH = 468
MAX_LEG_DURATION_S = 6.0  # hard cutoff per leg if target/crossing is never reached


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)

    distance_inches = float(sys.argv[1])
    pwm = int(sys.argv[2])

    if abs(pwm) > MAX_PWM_CEILING:
        print(f"pwm {pwm} exceeds MAX_PWM_CEILING ({MAX_PWM_CEILING}) -- refusing. "
              f"Raise the constant in this script if you deliberately want more.")
        sys.exit(1)
    if distance_inches <= 0:
        print("distance_inches must be positive")
        sys.exit(1)

    target_counts = distance_inches * COUNTS_PER_INCH

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    try:
        time.sleep(BOOT_WAIT_S)
        ser.reset_input_buffer()

        start_count = read_encoder_now(ser)
        if start_count is None:
            print("no encoder telemetry seen -- aborting before sending any drive command")
            return
        print(f"--- start count={start_count}, target={target_counts:.0f} counts "
              f"({distance_inches}in @ {COUNTS_PER_INCH}/in) ---")

        # Outbound leg
        print(f"--- outbound: {pwm} {pwm} ---")
        send(ser, pwm)
        end_count = start_count
        t_end = time.time() + MAX_LEG_DURATION_S
        while time.time() < t_end:
            c = drain_one(ser)
            if c is not None:
                end_count = c
                if abs(end_count - start_count) >= target_counts:
                    break
        else:
            print("--- outbound leg hit MAX_LEG_DURATION_S without reaching target ---")
        send(ser, "stop")
        print(f"--- outbound stopped at count={end_count} "
              f"({abs(end_count-start_count)/COUNTS_PER_INCH:.2f}in) ---")

        time.sleep(1.0)
        drain_all(ser)

        # Return leg -- back toward the ORIGINAL start_count, not leg-1's end point
        return_pwm = -pwm
        print(f"--- return: {return_pwm} {return_pwm} ---")
        send(ser, return_pwm)
        outbound_sign = sign(end_count - start_count)
        cur = end_count
        t_end = time.time() + MAX_LEG_DURATION_S
        while time.time() < t_end:
            c = drain_one(ser)
            if c is not None:
                cur = c
                if sign(cur - start_count) != outbound_sign or cur == start_count:
                    break
        else:
            print("--- return leg hit MAX_LEG_DURATION_S without crossing start ---")
        send(ser, "stop")
        print(f"--- return stopped at count={cur} "
              f"(net offset {(cur-start_count)/COUNTS_PER_INCH:.2f}in from start) ---")

    finally:
        # Guaranteed even if something above throws -- a prior version of this test
        # script died on a corrupted serial line and left the last drive command
        # running unattended. Never again.
        try:
            ser.write(b"stop\n")
            ser.flush()
        except Exception:
            pass
        time.sleep(0.2)
        drain_all(ser)
        ser.close()
        print("--- done, stopped ---")


def sign(x):
    return (x > 0) - (x < 0)


def send(ser, pwm_or_cmd):
    try:
        cmd = pwm_or_cmd if isinstance(pwm_or_cmd, str) else f"{pwm_or_cmd} {pwm_or_cmd}"
        ser.write((cmd + "\n").encode())
    except Exception as e:
        print(f"[warn] send failed: {e}")


def read_encoder_now(ser, timeout_s=2.0):
    """Drain until we see one valid ENC line, return its count."""
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        c = drain_one(ser)
        if c is not None:
            return c
    return None


def drain_one(ser):
    """Read one line; return its encoder count if it's a valid ENC line, else None."""
    try:
        raw = ser.readline()
    except Exception:
        return None
    if not raw:
        return None
    line = raw.decode(errors="replace").rstrip()
    if not line:
        return None
    if line.startswith("ENC "):
        parts = line.split()  # "ENC <left> <right> <ms>"; uses the left count
        if len(parts) == 4:
            try:
                return int(parts[1])
            except ValueError:
                return None  # corrupted line -- skip, never let this kill the run
    else:
        print(f"[log] {line}")
    return None


def drain_all(ser):
    try:
        while ser.in_waiting:
            drain_one(ser)
    except Exception:
        pass


if __name__ == "__main__":
    main()
