#!/usr/bin/env python3
"""Score a decision question against a real run recorded by the MCP server.

decision_test.py replays the per-run summaries in calibration/. This replays an actual
recording (recordings/<name>/<name>.csv, 50 Hz) as the states a reflex layer would see
during the move, and asks the question at each tick.

The point is the window the summaries cannot show: the ~0.195 s PWM ramp at the start of
every move, where the command is live and the encoders still read zero. A stall detector
that fires there is useless, and nothing in calibration/*.csv contains that state.

Encoder distance is taken as a magnitude. This firmware reports forward travel as
negative, and "travelled -4.08 in" is a different question from "travelled 4.08 in".

  tools/decision_replay.py recordings/decision_test_A/decision_test_A.csv \
      --pwm 60 --direction forward --elapsed 2.22
"""

import argparse, csv, json, os, statistics, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decision_test as dt

RAMP_S = 0.195   # ROBOT_CHARACTERIZATION.md: motion starts this long after the command


def load(path):
    out = []
    for r in csv.DictReader(open(path)):
        try:
            t = float(r["time_s"]); d = float(r["dist_avg_in"])
        except (TypeError, ValueError):
            continue
        out.append((t, abs(d)))
    return out


def at(samples, t):
    """Distance at time t: the last sample at or before it."""
    best = 0.0
    for ts, d in samples:
        if ts <= t:
            best = d
        else:
            break
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--model", default="unsloth/Qwen3.8-27B-NVFP4")
    ap.add_argument("--question", choices=sorted(dt.QUESTIONS), default="turning")
    ap.add_argument("--pwm", type=int, required=True)
    ap.add_argument("--direction", default="forward")
    ap.add_argument("--elapsed", type=float, required=True,
                    help="elapsed_s the drive tool reported, i.e. how long the command ran")
    ap.add_argument("--url", default=dt.DEFAULT_URL)
    ap.add_argument("--out")
    args = ap.parse_args()

    key = os.environ.get("LLAMA_ROUTER_KEY")
    if not key:
        sys.exit("set LLAMA_ROUTER_KEY")
    dt.QUESTION = {"type": "noul", "instructions": dt.QUESTIONS[args.question]}

    samples = load(args.csv)
    moved = [t for t, d in samples if d > 0.05]
    if not moved:
        sys.exit("no motion in this recording")
    t_cmd = moved[0] - RAMP_S          # when the command went out
    print(f"{len(samples)} samples; motion first seen at t={moved[0]:.3f}s, so the command "
          f"went out at about t={t_cmd:.3f}s")
    print(f'model {args.model}   question "{dt.QUESTIONS[args.question]}"')
    print(f"command: drive {args.direction} at pwm {args.pwm} for {args.elapsed:.2f}s\n")

    # Fine through the ramp, 5 Hz after it -- the flip point is what we are looking for.
    ticks = [round(x * 0.05, 3) for x in range(0, 11)]
    ticks += [round(0.5 + x * 0.2, 3) for x in range(1, int((args.elapsed - 0.5) / 0.2) + 1)]

    print(f"{'since cmd':>10} {'enc in':>8} {'p':>7} {'says':>9}  {'rtt':>6}")
    recs = []
    for dt_s in ticks:
        d = at(samples, t_cmd + dt_s)
        state = (f"Command: drive {args.direction} at pwm {args.pwm}, issued {dt_s:.2f} s "
                 f"ago. Encoder distance since the command: {d:.2f} in.")
        payload, rtt, err = dt.ask(args.url, key, args.model, state, False, 60)
        if err:
            print(f"{dt_s:9.2f}s {d:8.2f}  FAILED  {err[:60]}")
            continue
        p = dt.probability(payload)
        verdict = "turning" if p >= 0.5 else "STUCK"
        print(f"{dt_s:9.2f}s {d:8.2f} {p:7.3f} {verdict:>9}  {rtt:5.2f}s")
        recs.append(dict(since_cmd_s=dt_s, enc_in=round(d, 3), p=round(p, 4),
                         rtt_s=round(rtt, 3), state=state))

    if args.out and recs:
        with open(args.out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(recs[0])); w.writeheader(); w.writerows(recs)
        print(f"\n-> {args.out}")

    flips = [r for r in recs if r["p"] >= 0.5]
    if flips:
        print(f"\nfirst tick called 'turning': {flips[0]['since_cmd_s']:.2f}s after the "
              f"command, at {flips[0]['enc_in']:.2f} in travelled")
    stuck = [r for r in recs if r["p"] < 0.5]
    print(f"{len(stuck)}/{len(recs)} ticks would have called a healthy move stuck"
          + (f" (up to {max(r['since_cmd_s'] for r in stuck):.2f}s in)" if stuck else ""))
    print(f"round trip median {statistics.median(r['rtt_s'] for r in recs):.3f}s")


if __name__ == "__main__":
    main()
