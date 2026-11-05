#!/usr/bin/env python3
"""Score a decision question over a labelled grid of physically-grounded robot states.

The live runs gave one blocked example and one healthy move. That is not enough to compare
models or wordings, and putting the robot through 40 stalls to get there would be slow and
hard on the motors. So this builds the grid from the measured numbers in
ROBOT_CHARACTERIZATION.md and labels each state by an explicit rule:

  stalled  wheels essentially not turning
  blocked  wheels turning, robot not advancing (advance < 25% of wheel rotation)
  slow     advancing, but under 70% of the distance that PWM should have produced
  fine     advancing at close to the commanded rate

The question is whether a model can reproduce that rule from one sentence of criteria,
across several channels, without the rule being coded into the controller. Both live runs
are included as anchors, so a model that does well here is not just fitting the synthetic
cases.

  tools/decision_grid.py --models unsloth/Qwen3.8-27B-NVFP4,chat --out /tmp/grid.csv
"""

import argparse, csv, json, os, statistics, sys, time, urllib.error, urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decision_test as dt

# in/s at each PWM, straight line, from ROBOT_CHARACTERIZATION.md
SPEED = {50: 3.9, 60: 5.9, 80: 9.3, 100: 11.9, 125: 14.3}

CRITERIA = {
    "stalled": "the motors are loaded but the wheels are not turning",
    "blocked": "the wheels are turning but the robot is not advancing",
    "slow":    "the robot is advancing, but well short of the commanded speed",
    "fine":    "the robot is moving normally, close to the commanded speed",
}


def label(pwm, secs, enc, adv):
    expected = SPEED[pwm] * secs
    if enc < 0.1:
        return "stalled"
    if adv < 0.25 * enc:
        return "blocked"
    return "fine" if adv >= 0.7 * expected else "slow"


def grid():
    """(pwm, seconds, encoder inches, advanced inches, note)."""
    cases = []
    for pwm in (60, 80, 125):
        for secs in (0.5, 1.5, 3.0):
            exp = SPEED[pwm] * secs
            cases += [
                (pwm, secs, 0.0, 0.0, "dead stall"),
                (pwm, secs, 0.04, 0.0, "creeping, effectively stalled"),
                (pwm, secs, exp * 0.3, exp * 0.02, "wheels turning, wall in front"),
                (pwm, secs, exp, exp * 0.1, "full wheel rotation, no advance"),
                (pwm, secs, exp * 0.4, exp * 0.38, "advancing at 40% of commanded"),
                (pwm, secs, exp, exp * 0.95, "healthy"),
                (pwm, secs, exp * 1.15, exp * 1.1, "healthy, slightly over"),
            ]
    # The two real runs, as anchors.
    cases += [(60, 1.2, 1.92, 0.20, "LIVE run B: against the wall"),
              (60, 2.0, 13.14, 12.60, "LIVE run A: healthy (front_left cone)")]
    return cases


def state(pwm, secs, enc, adv):
    return (f"Command: drive forward at pwm {pwm}, issued {secs:.2f} s ago. "
            f"Expected {SPEED[pwm] * secs:.1f} in by now. "
            f"Wheel encoders report {enc:.2f} in of rotation. "
            f"Forward range sensor shows the robot advanced {adv:.2f} in.")


def ask(url, key, model, st, timeout=90):
    questions = {
        "progress": {"type": "noul", "instructions": "The robot is making progress"},
        "cause": {"type": "choice", "instructions": "What is happening",
                  "criteria": CRITERIA},
    }
    body = json.dumps({"model": model, "state": st, "questions": questions}).encode()
    req = urllib.request.Request(f"{url}/v1/systemone", body,
                                 {"Content-Type": "application/json", "API-KEY": key})
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r), time.monotonic() - t0, None
    except urllib.error.HTTPError as e:
        return None, time.monotonic() - t0, f"HTTP {e.code}"
    except Exception as e:
        return None, time.monotonic() - t0, type(e).__name__


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models", required=True, help="comma separated")
    ap.add_argument("--url", default=dt.DEFAULT_URL)
    ap.add_argument("--out")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    key = os.environ.get("LLAMA_ROUTER_KEY") or sys.exit("set LLAMA_ROUTER_KEY")

    cases = grid()
    print(f"{len(cases)} labelled states; truth distribution: "
          + ", ".join(f"{k}={sum(1 for c in cases if label(*c[:4]) == k)}"
                      for k in CRITERIA))

    recs = []
    for model in args.models.split(","):
        model = model.strip()
        right = fails = 0
        conf_wrong, conf_right, lats = [], [], []
        confusion = {}
        for pwm, secs, enc, adv, note in cases:
            truth = label(pwm, secs, enc, adv)
            payload, rtt, err = ask(args.url, key, model, state(pwm, secs, enc, adv))
            lats.append(rtt)
            if err:
                fails += 1
                continue
            a = payload["answers"]
            got = a["cause"]["choice"]
            conf = a["cause"]["confidence"]
            prog = a["progress"]["noul"]
            ok = got == truth
            right += ok
            (conf_right if ok else conf_wrong).append(conf)
            confusion[(truth, got)] = confusion.get((truth, got), 0) + 1
            rec = dict(model=model, pwm=pwm, secs=secs, enc=round(enc, 2),
                       adv=round(adv, 2), truth=truth, got=got,
                       confidence=round(conf, 3), p_progress=round(prog, 4),
                       rtt_s=round(rtt, 3), note=note)
            # Every option's probability, not just the argmax: fitting per-option offsets
            # needs the full distribution.
            for k, v in (a["cause"].get("probabilities") or {}).items():
                rec[f"p_{k}"] = round(float(v), 6)
            recs.append(rec)
            if args.verbose and not ok:
                print(f"    {truth:>7} -> {got:<7} conf {conf:.2f}  pwm{pwm} {secs}s "
                      f"enc {enc:.2f} adv {adv:.2f}  ({note})")
        n = len(cases) - fails
        print(f"\n{model}")
        if not n:
            print(f"  all {fails} calls failed")
            continue
        print(f"  cause accuracy {right}/{n} = {right / n:.3f}"
              + (f"   ({fails} calls failed)" if fails else ""))
        if conf_wrong:
            print(f"  mean confidence when right {statistics.fmean(conf_right):.3f}, "
                  f"when WRONG {statistics.fmean(conf_wrong):.3f}"
                  + ("   <-- no louder when wrong, so confidence is usable as a gate"
                     if statistics.fmean(conf_wrong) < statistics.fmean(conf_right) - 0.05
                     else "   <-- just as confident when wrong: do not gate on confidence"))
        print(f"  latency median {statistics.median(lats):.2f}s")
        rows = [r for r in recs if r["model"] == model]
        pos = [r["p_progress"] for r in rows if r["truth"] == "fine"]
        neg = [r["p_progress"] for r in rows if r["truth"] in ("stalled", "blocked")]
        if pos and neg:
            a = dt.auc([(p, True) for p in pos] + [(p, False) for p in neg])
            print(f"  p(progress): fine mean {statistics.fmean(pos):.3f} vs "
                  f"stalled/blocked mean {statistics.fmean(neg):.3f}, AUC {a:.3f}")
        worst = sorted(((c, t, g) for (t, g), c in confusion.items() if t != g), reverse=True)
        if worst:
            print("  top confusions: "
                  + ", ".join(f"{t}->{g} x{c}" for c, t, g in worst[:4]))

    if args.out and recs:
        with open(args.out, "w", newline="") as fh:
            cols = list(recs[0]) + [f"p_{k}" for k in CRITERIA if f"p_{k}" not in recs[0]]
            w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
            w.writeheader(); w.writerows(recs)
        print(f"\n{len(recs)} rows -> {args.out}")


if __name__ == "__main__":
    main()
