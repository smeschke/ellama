#!/usr/bin/env python3
"""Sweep turn_degrees() across PWM values and both directions, logging how each turn
actually behaved (IMU-based actual degrees, encoder estimate for comparison, time taken,
turn rate) to a CSV. Meant to be re-run per physical robot, since hand-assembled units
can turn more easily one way than the other (uneven wheel alignment) -- pass --robot-id
to tag which unit a run belongs to.

Talks directly to mcp_server/robot_server.py's functions (not through the MCP protocol)
-- this is a local characterization tool, same as everything else in calibration/.

Read DRIVING_POLICY.md and mcp_server/README.md before running this. It sends real
nonzero-PWM drive commands in a loop -- confirm the robot is on a clear floor with
several feet of room before starting, same as any other test in this repo. There's a
--dry-run flag that prints the planned test matrix without ever calling confirmed_safe.

Usage:
    python3 calibration/characterize_turns.py --robot-id unit1
    python3 calibration/characterize_turns.py --pwms 40 60 80 100 125 --degrees 90
    python3 calibration/characterize_turns.py --dry-run
"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mcp_server"))

DEFAULT_PWMS = [40, 50, 60, 80, 100, 125]
PAUSE_BETWEEN_S = 1.5


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pwms", type=int, nargs="+", default=DEFAULT_PWMS,
                   help=f"PWM values to test, both directions each (default {DEFAULT_PWMS})")
    p.add_argument("--degrees", type=float, default=90.0, help="target turn per test (default 90)")
    p.add_argument("--robot-id", default="unnamed", help="tag for this robot/run, goes in the CSV filename and rows")
    p.add_argument("--out", help="output CSV path (default calibration/turn_characterization_<robot-id>_<timestamp>.csv)")
    p.add_argument("--dry-run", action="store_true", help="print the planned test matrix and exit, no hardware contact")
    args = p.parse_args()

    matrix = [(pwm, direction) for pwm in args.pwms for direction in ("left", "right")]

    print(f"Planned {len(matrix)} tests, {args.degrees} deg each, {PAUSE_BETWEEN_S}s pause between:")
    for pwm, direction in matrix:
        print(f"  pwm={pwm:3d}  direction={direction}")

    if args.dry_run:
        print("\n--dry-run: not connecting to hardware, not sending anything.")
        return

    confirm = input(
        "\nAbout to run this on real hardware -- confirm the robot is on a clear floor "
        "with several feet of room, wheels touching the ground. Type 'yes' to proceed: "
    )
    if confirm.strip().lower() != "yes":
        print("Not confirmed -- exiting without sending anything.")
        return

    import robot_server as rs  # noqa: E402  (import after the confirm prompt, on purpose)

    if rs.link is None:
        print(f"Not connected: {rs._connect_error}")
        sys.exit(1)

    # Connecting resets the ESP32 and starts the telemetry pump thread, but the first
    # ENC/IMU lines take a moment to actually arrive -- without this, the very first test
    # can get refused as "no live telemetry" purely from that startup race, wasting a data
    # point. Wait for both to actually be live (or give up after a few seconds).
    for _ in range(50):
        snap = rs.link.telemetry_snapshot()
        if snap["telemetry_live"] and snap["imu_live"]:
            break
        time.sleep(0.1)

    out_path = Path(args.out) if args.out else Path(__file__).with_name(
        f"turn_characterization_{args.robot_id}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    )

    rows = []
    for i, (pwm, direction) in enumerate(matrix, 1):
        print(f"\n[{i}/{len(matrix)}] turn_degrees({args.degrees}, '{direction}', {pwm}, True)")
        result = rs.turn_degrees(args.degrees, direction, pwm, True)
        print(f"  -> ok={result.get('ok')} actual={result.get('actual_degrees_turned')} "
              f"encoder_est={result.get('encoder_degrees_estimate')} "
              f"elapsed_s={result.get('elapsed_s')} hit_timeout={result.get('hit_timeout')}")
        rate = None
        if result.get("ok") and result.get("elapsed_s"):
            rate = abs(result["actual_degrees_turned"]) / result["elapsed_s"]
        rows.append(dict(
            robot_id=args.robot_id,
            pwm=pwm,
            direction=direction,
            requested_degrees=args.degrees,
            actual_degrees_turned=result.get("actual_degrees_turned"),
            encoder_degrees_estimate=result.get("encoder_degrees_estimate"),
            elapsed_s=result.get("elapsed_s"),
            turn_rate_deg_s=round(rate, 2) if rate else None,
            hit_timeout=result.get("hit_timeout"),
            ok=result.get("ok"),
        ))
        time.sleep(PAUSE_BETWEEN_S)

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nSaved {len(rows)} rows to {out_path}")

    print("\nLeft vs right, by PWM:")
    by_pwm = {}
    for r in rows:
        by_pwm.setdefault(r["pwm"], {})[r["direction"]] = r
    for pwm in sorted(by_pwm):
        l, rr = by_pwm[pwm].get("left"), by_pwm[pwm].get("right")
        lr = f"L rate={l['turn_rate_deg_s']}" if l and l["turn_rate_deg_s"] else "L timed out"
        rrs = f"R rate={rr['turn_rate_deg_s']}" if rr and rr["turn_rate_deg_s"] else "R timed out"
        print(f"  pwm={pwm:3d}  {lr}  {rrs}")


if __name__ == "__main__":
    main()
