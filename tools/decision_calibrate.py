#!/usr/bin/env python3
"""Fit a calibration for a decision question from the probabilities already collected.

llama-router's own fit lives behind /api/decisions/calibrations/fit, which answers
"Not authenticated" to an lr_ API key -- it wants a dashboard session. So this does the
same job on the CSV that decision_test.py --out writes, reports an honest held-out score,
and prints request bodies to paste into the dashboard.

A noul calibration is Platt scaling on the log-odds:

    calibrated_logit = a * logit(p) + b

`b` alone is the additive log offset per answer the README's `offsets` form takes:
offsets {"true": b/2, "false": -b/2} shifts the log-odds by b. `a` is the temperature,
which that form does not accept, so the pasteable body uses the offset-only fit and the
full fit is reported for comparison.

  tools/decision_calibrate.py /tmp/q27_turning_snapshot.csv \
      --model unsloth/Qwen3.8-27B-NVFP4 --question turning
"""

import argparse, csv, json, math, random, statistics, sys

sys.path.insert(0, __import__("os").path.dirname(__file__))
from decision_test import QUESTIONS, auc, best_threshold   # one source for the wording

EPS = 1e-6


def logit(p):
    p = min(max(p, EPS), 1 - EPS)
    return math.log(p / (1 - p))


def sigmoid(z):
    return 1 / (1 + math.exp(-z)) if z > -700 else 0.0


def nll(pairs, a, b):
    """Negative log-likelihood of the labels under a*logit(p)+b."""
    tot = 0.0
    for p, y in pairs:
        q = sigmoid(a * logit(p) + b)
        tot -= math.log(max(q if y else 1 - q, EPS))
    return tot / len(pairs)


def fit(pairs, fit_a=True):
    """Coordinate descent on a grid then a local refine. The data here is tens of points,
    so a cheap deterministic search beats a gradient implementation nobody will check."""
    best = (float("inf"), 1.0, 0.0)
    a_grid = [0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0] if fit_a else [1.0]
    for a in a_grid:
        for b in [x * 0.25 for x in range(-24, 25)]:
            v = nll(pairs, a, b)
            if v < best[0]:
                best = (v, a, b)
    _, a, b = best
    step = 0.125
    for _ in range(40):                      # refine
        cands = [(nll(pairs, aa, bb), aa, bb)
                 for aa in ([a] if not fit_a else [a - step, a, a + step]) if aa > 0.01
                 for bb in [b - step, b, b + step]]
        v, a, b = min(cands)
        step /= 1.5
    return a, b


def acc(pairs, a=1.0, b=0.0, cut=0.5):
    return sum(1 for p, y in pairs
               if (sigmoid(a * logit(p) + b) >= cut) == y) / len(pairs)


def folds(pairs, k, seed=0):
    """Stratified k-fold, so a fold cannot end up with no stalls in it."""
    pos = [x for x in pairs if x[1]]
    neg = [x for x in pairs if not x[1]]
    rnd = random.Random(seed)
    rnd.shuffle(pos); rnd.shuffle(neg)
    out = [[] for _ in range(k)]
    for i, x in enumerate(pos + neg):
        out[i % k].append(x)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="a decision_test.py --out file")
    ap.add_argument("--model", required=True)
    ap.add_argument("--question", choices=sorted(QUESTIONS), required=True)
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--emit", help="write the dashboard request bodies here as JSON")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv)))
    pairs = [(float(r["p"]), r["label"].strip().lower() == "true") for r in rows]
    n_pos = sum(1 for _, y in pairs if y)
    if not n_pos or n_pos == len(pairs):
        sys.exit("need both classes")

    print(f"{len(pairs)} runs from {args.csv}: {n_pos} true, {len(pairs) - n_pos} false")
    print(f'model {args.model}   question "{QUESTIONS[args.question]}"\n')

    a0 = auc(pairs)
    print(f"uncalibrated   AUC {a0:.3f}   acc@0.5 {acc(pairs):.3f}   "
          f"Brier {statistics.fmean((p - (1.0 if y else 0.0))**2 for p, y in pairs):.3f}")
    bacc, bcut = best_threshold(pairs)
    print(f"               best acc {bacc:.3f} at threshold {bcut:.3f}")
    print("  AUC is unchanged by any calibration: Platt scaling is monotonic, so it moves\n"
          "  the threshold to 0.5, it does not reorder anything.\n")

    # Fit on everything, for the numbers to paste.
    a_off, b_off = fit(pairs, fit_a=False)
    a_full, b_full = fit(pairs, fit_a=True)
    print(f"offset only (a=1)   b={b_off:+.3f}   in-sample acc@0.5 {acc(pairs, 1.0, b_off):.3f}"
          f"   Brier {statistics.fmean((sigmoid(logit(p)+b_off)-(1.0 if y else 0.0))**2 for p,y in pairs):.3f}")
    print(f"with temperature    a={a_full:.3f} b={b_full:+.3f}   "
          f"in-sample acc@0.5 {acc(pairs, a_full, b_full):.3f}"
          f"   Brier {statistics.fmean((sigmoid(a_full*logit(p)+b_full)-(1.0 if y else 0.0))**2 for p,y in pairs):.3f}")

    # Held-out: fit on k-1 folds, score the one left out. In-sample numbers on 45 points
    # mean little; this is the one worth quoting.
    print(f"\nheld-out, {args.folds}-fold stratified:")
    for name, fa in (("offset only", False), ("with temperature", True)):
        pre, post = [], []
        for i, fold in enumerate(folds(pairs, args.folds)):
            train = [x for j, f in enumerate(folds(pairs, args.folds)) if j != i for x in f]
            a, b = fit(train, fit_a=fa)
            pre.append(acc(fold))
            post.append(acc(fold, a, b))
        print(f"  {name:17} acc@0.5  before {statistics.fmean(pre):.3f}  "
              f"after {statistics.fmean(post):.3f}")

    question = {"type": "noul", "instructions": QUESTIONS[args.question]}
    direct = {"model": args.model, "question": question,
              "offsets": {"true": round(b_off / 2, 4), "false": round(-b_off / 2, 4)}}
    from_probs = {"model": args.model, "question": question,
                  "examples": [{"probabilities": {"true": round(p, 6), "false": round(1 - p, 6)},
                                "answer": "true" if y else "false"} for p, y in pairs]}
    print("\nPaste into the dashboard (POST /api/decisions/calibrations), offsets given:")
    print(json.dumps(direct, indent=2))
    print(f"\n...or let the router fit them itself from the same {len(pairs)} labelled "
          f"probabilities (same endpoint, `examples` instead of `offsets`)"
          + (f" -> {args.emit}" if args.emit else ": rerun with --emit PATH"))
    if args.emit:
        with open(args.emit, "w") as fh:
            json.dump({"direct": direct, "from_probabilities": from_probs}, fh, indent=2)


if __name__ == "__main__":
    main()
