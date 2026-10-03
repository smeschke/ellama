"""Find a dark tape line in a downward-facing camera image. Pure OpenCV/numpy -- no robot,
serial or MCP dependencies, so it can be tuned offline on saved photos:

    python3 mcp_server/line_vision.py photo.jpg [out.png]

The image is split into horizontal bands. In each band the dark pixels are found and the
widest run of them is taken as the line. Because the tape is a known width (3/4"), each
band's pixel width gives that band's inches-per-pixel directly, so offsets come out in
inches without a separate camera calibration (perspective makes bands near the top of the
frame smaller, and that is handled per band). Assumes the camera is centered on the robot
and pointing straight down/forward; image-center x is "straight ahead".
"""

import cv2
import numpy as np

TAPE_WIDTH_IN = 0.75
N_BANDS = 8
# Fraction of the image height, top and bottom, that is ignored. The bottom often shows the
# robot's own bumper or wheels; the top is far away and low-resolution.
ROI_TOP = 0.05
ROI_BOTTOM = 0.56  # new webcam mount sees the robot front + shadow below ~0.60
# A run narrower than this (fraction of image width) is noise, wider is a shadow or the
# edge of the sheet, not tape.
MIN_RUN_FRAC = 0.01
MAX_RUN_FRAC = 0.25
BLACKHAT_KERNEL_FRAC = 0.20  # must be wider than the tape in the image
MIN_PEAK = 25  # weakest tape-vs-surroundings contrast (0-255) we accept as "a line"
OPEN_WIDTH_FRAC = 0.01  # box width (fraction of image width) the tape must fill to survive
MAX_BAND_STEP_FRAC = 0.15  # max sideways jump of the line between neighboring bands
CROSS_MIN_FRAC = 0.40  # a run this wide (fraction of image width) is a cross strip, not the line
CROSS_MIN_ROWS_FRAC = 0.012  # ...and must stay that wide for this much of the image height
CROSS_MAX_Y_FRAC = 0.56  # only looked for above this (fraction of height from the top)
REL_THRESH = 0.35  # mask threshold as a fraction of the strongest response


def _dark_mask(gray):
    """Black-hat filter: keeps dark features NARROWER than the kernel (the tape) and drops
    broad dark areas (shadows, vignetting, a dim floor) and slow lighting gradients. The
    kernel is a fraction of image width, wider than the tape. Threshold is relative to the
    strongest response so a glossy tape with a glare stripe down the middle still comes out
    whole, with a floor so a frame with no tape in it produces an empty mask."""
    h, w = gray.shape
    g = cv2.GaussianBlur(gray, (0, 0), max(1.0, w / 500))
    k = int(BLACKHAT_KERNEL_FRAC * w) | 1
    bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT,
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    peak = float(np.percentile(bh, 99.5))
    if peak < MIN_PEAK:
        return np.zeros_like(gray), None
    thr = max(MIN_PEAK * 0.6, REL_THRESH * peak)
    mask = (bh > thr).astype(np.uint8) * 255
    m = max(1, int(0.015 * w))  # the filter misbehaves at the very border
    mask[:, :m] = 0
    mask[:, -m:] = 0
    kk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kk)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kk)
    # Scratches and glare streaks on the floor come out thin and ragged; the tape is a solid
    # strip at least this wide. Opening with a box that wide removes them, keeps the tape.
    ow = max(3, int(OPEN_WIDTH_FRAC * w))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (ow, ow)))
    return mask, float(thr)


def _runs(row_any):
    """(start, end) of each run of True in a 1-D bool array."""
    d = np.diff(np.concatenate(([0], row_any.astype(np.int8), [0])))
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0]))


def analyze(bgr, prev_x_frac=None):
    """Returns (result dict, annotated BGR image). `prev_x_frac` (line x at the bottom band
    last time, as a fraction of width) breaks ties when several dark runs are plausible."""
    h, w = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    mask, thresh = _dark_mask(gray)
    y0, y1 = int(h * ROI_TOP), int(h * ROI_BOTTOM)
    edges = np.linspace(y1, y0, N_BANDS + 1).astype(int)  # bottom (near) -> top (far)

    # Glossy tape can have a glare stripe down its middle that the filter drops; bridge gaps
    # up to about one tape width sideways so the tape reads as one run again.
    mask_runs = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                 cv2.getStructuringElement(cv2.MORPH_RECT, (max(3, int(0.07 * w)), 1)))

    bands = []
    last_w = None
    last_x = None if prev_x_frac is None else prev_x_frac * w
    for i in range(N_BANDS):
        ya, yb = edges[i + 1], edges[i]
        col = (mask_runs[ya:yb] > 0).mean(axis=0) > 0.5  # column dark for most of the band
        cands = [(s, e) for s, e in _runs(col) if MIN_RUN_FRAC * w <= e - s <= MAX_RUN_FRAC * w]
        if not cands:
            continue
        ref = last_x if last_x is not None else w / 2
        # Nearest to the band below, and (once we have one) similar in width to it -- tape
        # width changes slowly up the image, scratches and glare streaks don't match it.
        def cost(r):
            c = abs((r[0] + r[1]) / 2 - ref)
            if last_w:
                c += 2.0 * abs((r[1] - r[0]) - last_w)
            return c
        s, e = min(cands, key=cost)
        cx, wid = float((s + e) / 2), int(e - s)
        last_x, last_w = cx, wid
        in_per_px = TAPE_WIDTH_IN / wid
        bands.append(dict(band=i, y=int((ya + yb) / 2), x_px=round(cx, 1), width_px=wid,
                          lateral_in=round(float((cx - w / 2) * in_per_px), 2)))

    if len(bands) >= 3:
        # Perspective makes width vary with distance, but not wildly between neighbors; a
        # band far off the typical width has swallowed noise (glare, scratches, shadow).
        med = float(np.median([b["width_px"] for b in bands]))
        bands = [b for b in bands if abs(b["width_px"] - med) <= 0.35 * med]

    # The line can't jump sideways between neighboring bands: a dot that does is the shadow or
    # glare near the frame edge (the nearest band picks up the dark strip along the bottom).
    # Peel such bands off the ends, nearest first.
    max_step = MAX_BAND_STEP_FRAC * w
    while len(bands) >= 3 and abs(bands[0]["x_px"] - bands[1]["x_px"]) > max_step:
        bands.pop(0)
    while len(bands) >= 3 and abs(bands[-1]["x_px"] - bands[-2]["x_px"]) > max_step:
        bands.pop()

    # Finish strip: tape laid ACROSS the line is a dark run far wider than the line itself.
    # Only looked for in the upper part of the image -- the bottom holds the shadow/bumper
    # strip, which is wide and dark too.
    ycut = int(h * CROSS_MAX_Y_FRAC)
    row_dark = (mask_runs[y0:ycut] > 0).sum(axis=1)
    wide = row_dark > CROSS_MIN_FRAC * w
    cross_y = None
    if wide.any():
        # nearest (lowest) block of >= CROSS_MIN_ROWS_FRAC*h consecutive wide rows
        need = max(2, int(CROSS_MIN_ROWS_FRAC * h))
        run = 0
        for i in range(len(wide) - 1, -1, -1):
            run = run + 1 if wide[i] else 0
            if run >= need:
                cross_y = y0 + i + run // 2
                break

    out = bgr.copy()
    cv2.rectangle(out, (0, y0), (w - 1, y1), (255, 200, 0), 1)
    cv2.line(out, (w // 2, y0), (w // 2, y1), (255, 200, 0), 1)
    tint = np.zeros_like(out)
    tint[mask > 0] = (0, 0, 255)
    out = cv2.addWeighted(out, 1.0, tint, 0.35, 0)
    for b in bands:
        cv2.circle(out, (int(b["x_px"]), b["y"]), 5, (0, 255, 0), -1)
        cv2.putText(out, f'{b["lateral_in"]:+.1f}', (int(b["x_px"]) + 8, b["y"]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

    res = dict(found=len(bands) >= 2, n_bands=len(bands), threshold=thresh, bands=bands,
               width=w, height=h, cross_strip=cross_y is not None,
               cross_y_frac=None if cross_y is None else round(cross_y / h, 3))
    if cross_y is not None:
        cv2.line(out, (0, cross_y), (w - 1, cross_y), (0, 165, 255), 2)
    if bands:
        near = bands[0]
        res["offset_in"] = near["lateral_in"]  # + = line is right of the robot's center
        res["x_frac"] = round(float(near["x_px"]) / w, 3)
    if len(bands) >= 2:
        # Heading error: slope of x vs y across the bands, i.e. how far the line leans
        # from vertical in the image. + = line leans right going away from the robot.
        ys = np.array([b["y"] for b in bands], float)
        xs = np.array([b["x_px"] for b in bands], float)
        slope = np.polyfit(-ys, xs, 1)[0]  # px right per px forward
        res["lean_deg"] = round(float(np.degrees(np.arctan(slope))), 1)
        far = bands[-1]
        res["lookahead_offset_in"] = far["lateral_in"]
    return res, out


if __name__ == "__main__":
    import json
    import sys

    img = cv2.imread(sys.argv[1])
    r, o = analyze(img)
    print(json.dumps(r, indent=1))
    cv2.imwrite(sys.argv[2] if len(sys.argv) > 2 else "line_view.png", o)
