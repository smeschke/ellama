#!/usr/bin/env python3
"""Sweep drive_distance() across PWM values, forward then back, logging steady-state-ish
speed (actual distance / elapsed time) to a CSV. Same idea as characterize_turns.py, for
ROBOT_CHARACTERIZATION.md section 1 (PWM to steady speed) instead of section 4.

Read DRIVING_POLICY.md and mcp_server/README.md before running this -- it sends real
nonzero-PWM drive commands in a loop. --clearance-forward-in / --clearance-reverse-in
must match the room you actually measured; the script refuses to run a distance that
doesn't leave a safety margin against them (drive_distance already overshoots its
target by 20-40% at speed, so the margin matters, not just the nominal distance).

Usage:
    python3 calibration/characterize_speed.py --robot-id unit1 \\
        --clearance-forward-in 48 --clearance-reverse-in 24
    python3 calibration/characterize_speed.py --dry-run
"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mcp_server"))

DEFAULT_PWMS = [40, 50, 60, 80, 100, 125]
PAUSE_BETWEEN_S = 1.0
OVERSHOOT_SAFETY_FACTOR = 2.0  # require clearance >= distance * this, not just >= distance
NET_DRIFT_WARN_IN = 6.0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pwms", type=int, nargs="+", default=DEFAULT_PWMS)
    p.add_argument("--distance-forward-in", type=float, default=12.0)
    p.add_argument("--distance-reverse-in", type=float, default=12.0)
    p.add_argument("--clearance-forward-in", type=float, required=True,
                   help="measured clear space in front of the robot right now")
    p.add_argument("--clearance-reverse-in", type=float, required=True,
                   help="measured clear space behind the robot right now")
    p.add_argument("--robot-id", default="unnamed")
    p.add_argument("--out", help="output CSV path (default calibration/speed_characterization_<robot-id>_<timestamp>.csv)")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    needed_fwd = args.distance_forward_in * OVERSHOOT_SAFETY_FACTOR
    needed_rev = args.distance_reverse_in * OVERSHOOT_SAFETY_FACTOR
    if needed_fwd > args.clearance_forward_in:
        sys.exit(f"refusing: forward distance {args.distance_forward_in}in needs "
                  f"{needed_fwd}in of margin ({OVERSHOOT_SAFETY_FACTOR}x, drive_distance "
                  f"overshoots at speed) but only {args.clearance_forward_in}in was given")
    if needed_rev > args.clearance_reverse_in:
        sys.exit(f"refusing: reverse distance {args.distance_reverse_in}in needs "
                  f"{needed_rev}in of margin ({OVERSHOOT_SAFETY_FACTOR}x) but only "
                  f"{args.clearance_reverse_in}in was given")

    print(f"Planned {len(args.pwms)} pwm steps, forward {args.distance_forward_in}in / "
          f"reverse {args.distance_reverse_in}in each, {PAUSE_BETWEEN_S}s pause between legs:")
    for pwm in args.pwms:
        print(f"  pwm={pwm:3d}")
    print(f"Clearance: {args.clearance_forward_in}in forward, {args.clearance_reverse_in}in "
          f"reverse (>= {OVERSHOOT_SAFETY_FACTOR}x the planned distances, checked above)")

    if args.dry_run:
        print("\n--dry-run: not connecting to hardware, not sending anything.")
        return

    confirm = input(
        "\nAbout to run this on real hardware -- confirm the clearances above are still "
        "accurate right now and the robot is on the ground. Type 'yes' to proceed: "
    )
    if confirm.strip().lower() != "yes":
        print("Not confirmed -- exiting without sending anything.")
        return

    import robot_server as rs  # noqa: E402

    if rs.link is None:
        print(f"Not connected: {rs._connect_error}")
        sys.exit(1)

    for _ in range(50):  # wait for telemetry to actually be flowing before the first test
        snap = rs.link.telemetry_snapshot()
        if snap["telemetry_live"]:
            break
        time.sleep(0.1)

    rs.zero_odometry()

    out_path = Path(args.out) if args.out else Path(__file__).with_name(
        f"speed_characterization_{args.robot_id}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    )

    rows = []
    for i, pwm in enumerate(args.pwms, 1):
        net = rs.read_telemetry()["dist_avg_in"]
        if abs(net) > NET_DRIFT_WARN_IN:
            print(f"\n!! net position drifted to {net:.1f}in before pwm={pwm} -- pausing "
                  "the sweep here rather than compounding it further.")
            break

        print(f"\n[{i}/{len(args.pwms)}] pwm={pwm}: forward {args.distance_forward_in}in")
        fwd = rs.drive_distance(args.distance_forward_in, "forward", pwm, True)
        speed_fwd = (fwd["actual_inches_traveled"] / fwd["elapsed_s"]) if fwd.get("elapsed_s") else None
        print(f"  -> ok={fwd.get('ok')} actual={fwd.get('actual_inches_traveled')}in "
              f"elapsed_s={fwd.get('elapsed_s')} speed={round(speed_fwd, 2) if speed_fwd else None}in/s "
              f"hit_timeout={fwd.get('hit_timeout')}")
        rows.append(dict(robot_id=args.robot_id, pwm=pwm, direction="forward",
                          requested_in=args.distance_forward_in,
                          actual_in=fwd.get("actual_inches_traveled"),
                          elapsed_s=fwd.get("elapsed_s"),
                          speed_in_s=round(speed_fwd, 3) if speed_fwd else None,
                          hit_timeout=fwd.get("hit_timeout"), ok=fwd.get("ok")))
        time.sleep(PAUSE_BETWEEN_S)

        print(f"[{i}/{len(args.pwms)}] pwm={pwm}: reverse {args.distance_reverse_in}in")
        rev = rs.drive_distance(args.distance_reverse_in, "reverse", pwm, True)
        speed_rev = (rev["actual_inches_traveled"] / rev["elapsed_s"]) if rev.get("elapsed_s") else None
        print(f"  -> ok={rev.get('ok')} actual={rev.get('actual_inches_traveled')}in "
              f"elapsed_s={rev.get('elapsed_s')} speed={round(speed_rev, 2) if speed_rev else None}in/s "
              f"hit_timeout={rev.get('hit_timeout')}")
        rows.append(dict(robot_id=args.robot_id, pwm=pwm, direction="reverse",
                          requested_in=args.distance_reverse_in,
                          actual_in=rev.get("actual_inches_traveled"),
                          elapsed_s=rev.get("elapsed_s"),
                          speed_in_s=round(speed_rev, 3) if speed_rev else None,
                          hit_timeout=rev.get("hit_timeout"), ok=rev.get("ok")))
        time.sleep(PAUSE_BETWEEN_S)

    if not rows:
        print("No rows collected.")
        return

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nSaved {len(rows)} rows to {out_path}")

    print("\nSpeed by PWM (in/s):")
    by_pwm = {}
    for r in rows:
        by_pwm.setdefault(r["pwm"], {})[r["direction"]] = r
    for pwm in sorted(by_pwm):
        f_, b_ = by_pwm[pwm].get("forward"), by_pwm[pwm].get("reverse")
        fs = f"fwd={f_['speed_in_s']}" if f_ and f_["speed_in_s"] else "fwd timed out"
        bs = f"rev={b_['speed_in_s']}" if b_ and b_["speed_in_s"] else "rev timed out"
        print(f"  pwm={pwm:3d}  {fs}  {bs}")

    final_net = rs.read_telemetry()["dist_avg_in"]
    print(f"\nFinal net position: {final_net:.1f}in from start")


if __name__ == "__main__":
    main()
