#!/usr/bin/env python3
"""Fit per-option offsets for a `choice` decision question.

decision_calibrate.py does the noul case (one probability, Platt scaling). A choice
question needs one additive log offset per option, which is what llama-router's
`offsets` body takes:

    calibrated(k) proportional to p(k) * exp(offset_k)

That is exactly the right shape for the failure the grid shows: every model's errors
collapse into one option, so that option needs pushing down and the others up. Unlike the
noul case there is no temperature to express, so the direct `offsets` body -- the one an
lr_ API key can at least be pasted into by an admin -- is enough here.

  tools/decision_calibrate_choice.py /tmp/grid2.csv
"""

import argparse, csv, json, math, random, statistics, sys

OPTIONS = ["stalled", "blocked", "slow", "fine"]
EPS = 1e-9


def predict(row, off):
    w = {k: row[k] * math.exp(off.get(k, 0.0)) for k in OPTIONS}
    z = sum(w.values()) or 1.0
    return {k: v / z for k, v in w.items()}


def choose(row, off):
    return max(predict(row, off).items(), key=lambda kv: kv[1])[0]


def nll(rows, off):
    return -statistics.fmean(
        math.log(max(predict(r, off)[r["truth"]], EPS)) for r in rows)


def fit(rows, rounds=60):
    """Coordinate ascent on the offsets, `stalled` pinned at 0 as the reference
    (the probabilities are normalised, so only differences matter)."""
    off = {k: 0.0 for k in OPTIONS}
    step = 1.0
    for _ in range(rounds):
        improved = False
        for k in OPTIONS[1:]:
            base = nll(rows, off)
            for delta in (step, -step):
                trial = dict(off); trial[k] = off[k] + delta
                if nll(rows, trial) < base - 1e-9:
                    off, base, improved = trial, nll(rows, trial), True
        if not improved:
            step /= 2
            if step < 1e-3:
                break
    return off


def acc(rows, off):
    return sum(1 for r in rows if choose(r, off) == r["truth"]) / len(rows)


def folds(rows, k, seed=0):
    by = {}
    for r in rows:
        by.setdefault(r["truth"], []).append(r)
    out = [[] for _ in range(k)]
    rnd = random.Random(seed)
    for cls in sorted(by):
        xs = by[cls][:]
        rnd.shuffle(xs)
        for i, x in enumerate(xs):
            out[i % k].append(x)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="a decision_grid.py --out file")
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--emit", help="write the dashboard bodies here")
    args = ap.parse_args()

    raw = list(csv.DictReader(open(args.csv)))
    bodies = {}
    for model in dict.fromkeys(r["model"] for r in raw):
        rows = []
        for r in raw:
            if r["model"] != model:
                continue
            try:
                row = {k: float(r[f"p_{k}"]) for k in OPTIONS}
            except (KeyError, ValueError):
                continue
            row["truth"] = r["truth"]
            rows.append(row)
        if len(rows) < 20:
            print(f"\n{model}: only {len(rows)} usable rows, skipping")
            continue

        print(f"\n{model}  ({len(rows)} states)")
        print(f"  before            acc {acc(rows, {}):.3f}")
        full = fit(rows)
        print(f"  fitted in-sample  acc {acc(rows, full):.3f}")
        pre, post = [], []
        fs = folds(rows, args.folds)
        for i, fold in enumerate(fs):
            train = [x for j, f in enumerate(fs) if j != i for x in f]
            o = fit(train)
            pre.append(acc(fold, {})); post.append(acc(fold, o))
        print(f"  HELD OUT, {args.folds}-fold   acc {statistics.fmean(pre):.3f} "
              f"-> {statistics.fmean(post):.3f}")
        print("  offsets: " + "  ".join(f"{k} {full[k]:+.2f}" for k in OPTIONS))
        # What the calibration actually did, per option.
        for k in OPTIONS:
            was = sum(1 for r in rows if choose(r, {}) == k)
            now = sum(1 for r in rows if choose(r, full) == k)
            tru = sum(1 for r in rows if r["truth"] == k)
            print(f"    {k:>8}: chosen {was:>3} -> {now:>3}   (truth {tru})")
        bodies[model] = {
            "model": model,
            "question": {"type": "choice", "instructions": "What is happening",
                         "criteria": {
                             "stalled": "the motors are loaded but the wheels are not turning",
                             "blocked": "the wheels are turning but the robot is not advancing",
                             "slow": "the robot is advancing, but well short of the commanded speed",
                             "fine": "the robot is moving normally, close to the commanded speed"}},
            "offsets": {k: round(full[k], 4) for k in OPTIONS},
        }

    if args.emit:
        with open(args.emit, "w") as fh:
            json.dump(bodies, fh, indent=2)
        print(f"\n-> {args.emit}  (POST each to /api/decisions/calibrations)")


if __name__ == "__main__":
    main()
