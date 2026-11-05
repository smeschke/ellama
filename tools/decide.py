#!/usr/bin/env python3
"""One decision call about the move that just happened. For the shell, between moves.

The questions, criteria and state wording all live in decision_client.py, which
explore_bump_turn.py also uses -- llama-router keys a calibration on the model plus the
exact question text, so a second copy of these strings would mean a calibration that
applies to one caller and not the other.

  tools/decide.py --cmd "drive forward at pwm 60" --expected 13.3 --enc 13.37 \
                  --elapsed 2.25 --range-before 1747 --range-after 1041

Omit --range-before/--range-after when the range sensor cannot measure advance for this
move (out of range, or a spin): the state then says so, which is worth more than the
number. Feeding a stale one turned a healthy move into "blocked" at 0.59.
"""
import argparse, json, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decision_client as dc

ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--model", default=dc.DEFAULT_MODEL)
ap.add_argument("--cmd", required=True, help='e.g. "drive forward at pwm 60"')
ap.add_argument("--elapsed", type=float, required=True)
ap.add_argument("--expected", type=float, help="expected inches")
ap.add_argument("--enc", type=float, help="encoder inches travelled")
ap.add_argument("--enc-left", type=float)
ap.add_argument("--enc-right", type=float)
ap.add_argument("--range-before", type=float)
ap.add_argument("--range-after", type=float)
ap.add_argument("--spin", action="store_true",
                help="this was a spin in place: the IMU is the instrument, not the encoders")
ap.add_argument("--expected-deg", type=float)
ap.add_argument("--imu-deg", type=float)
ap.add_argument("--note", default="")
ap.add_argument("--show-request", action="store_true",
                help="print the exact JSON sent to /v1/systemone before sending it")
ap.add_argument("--dry-run", action="store_true",
                help="print the request and do not send it (implies --show-request)")
a = ap.parse_args()

state = dc.build_state(
    cmd=a.cmd, elapsed_s=a.elapsed, expected_in=a.expected, enc_in=a.enc,
    enc_left_in=a.enc_left, enc_right_in=a.enc_right,
    range_before_mm=a.range_before, range_after_mm=a.range_after,
    range_valid=a.range_before is not None and a.range_after is not None,
    kind="spin" if a.spin else "straight",
    expected_deg=a.expected_deg, imu_deg=a.imu_deg, extra=a.note or None)

if a.show_request or a.dry_run:
    qs = dc.questions_for("Left wheel encoder" in state,
                          "spin" if "do not measure rotation" in state else "straight")
    print("POST " + dc.URL + "/v1/systemone")
    print(json.dumps({"model": a.model, "state": state, "questions": qs}, indent=2))
    print()
    if a.dry_run:
        sys.exit(0)

try:
    answers, rtt = dc.ask(state, model=a.model)
except RuntimeError as e:
    sys.exit(str(e))
v = dc.verdict(answers)

print(f"state: {state}\n")
print(f"  progress        {v['progress']:.3f}")
if v["wheels"] is not None:
    print(f"  both wheels     {v['wheels']:.3f}")
print(f"  safe to repeat  {v['retry']:.3f}")
print(f"  cause           {v['cause']}  (confidence {v['confidence']:.2f})")
print(f"    {json.dumps({k: round(x, 3) for k, x in v['probabilities'].items()})}")
print(f"\n  round trip {rtt*1000:.0f} ms")
if v["cause"] == "unmeasured":
    print("\n  A channel has gone bad. Look at the robot rather than driving it further.")
