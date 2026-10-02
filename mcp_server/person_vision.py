"""Person finder for "turn to me": a camera frame in, where the person is (as a bearing) out.
Pure logic -- no serial, camera or MCP -- so it can be tried on saved frames.

The phone is mounted portrait and tilted up, so often only legs/torso are in frame. Uses a
YOLO pose model: every keypoint it can see (ankles, knees, hips, shoulders...) votes for the
person's horizontal center, falling back to the box center. Only the horizontal position
matters, so tilt doesn't.

Bearing: + = person is to the RIGHT of where the robot is pointing (same sign as the line
follower's phi). HFOV_DEG is a GUESS (typical phone main camera, portrait) -- calibrate by
spinning a known IMU angle and watching how far a fixed object moves in the frame.
"""

import math
import pathlib

HFOV_DEG = 51.0          # horizontal field of view of the portrait stream. UNCALIBRATED.
MODEL_PATH = pathlib.Path(__file__).parent / "models" / "yolo11n-pose.pt"
MIN_PERSON_CONF = 0.35   # box confidence to count as a person at all
MIN_KP_CONF = 0.4        # keypoint confidence to let it vote for the center
MIN_BOX_AREA_FRAC = 0.02 # ignore specks (a poster, a far-off person) below this share of the frame

_model = None


def _get_model():
    global _model
    if _model is None:
        from ultralytics import YOLO
        _model = YOLO(str(MODEL_PATH))
    return _model


def bearing_deg(x_frac, hfov_deg=HFOV_DEG):
    """Angle from the camera axis to a point at x_frac (0 = left edge, 1 = right edge)."""
    f = 0.5 / math.tan(math.radians(hfov_deg) / 2)      # focal length in image-widths
    return math.degrees(math.atan((x_frac - 0.5) / f))


def analyze(bgr, draw=True):
    """Returns (result, annotated). result: n_people, found (exactly one), and for each
    person (largest first) `x_frac`, `bearing_deg`, `box` (fractions), `conf`, `n_kp`
    (keypoints that voted), `source` ('keypoints' or 'box'). Top-level `bearing_deg` /
    `x_frac` are those of the largest person, or None if nobody is seen."""
    import cv2
    h, w = bgr.shape[:2]
    r = _get_model().predict(bgr, verbose=False, conf=MIN_PERSON_CONF, classes=[0])[0]
    people = []
    if r.boxes is not None and len(r.boxes):
        xyxy = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        kps = r.keypoints.data.cpu().numpy() if r.keypoints is not None else None
        for i, (b, c) in enumerate(zip(xyxy, confs)):
            area = (b[2] - b[0]) * (b[3] - b[1]) / (w * h)
            if area < MIN_BOX_AREA_FRAC:
                continue
            xs = [] if kps is None else [float(k[0]) for k in kps[i] if k[2] >= MIN_KP_CONF]
            if len(xs) >= 2:
                cx, src = sum(xs) / len(xs), "keypoints"
            else:
                cx, src = float((b[0] + b[2]) / 2), "box"
            people.append(dict(x_frac=round(cx / w, 4), bearing_deg=round(bearing_deg(cx / w), 1),
                               box=[round(float(v) / s, 3) for v, s in zip(b, (w, h, w, h))],
                               conf=round(float(c), 2), n_kp=len(xs), source=src, area=area))
    people.sort(key=lambda p: -p.pop("area"))
    res = dict(n_people=len(people), found=len(people) == 1, people=people,
               x_frac=people[0]["x_frac"] if people else None,
               bearing_deg=people[0]["bearing_deg"] if people else None)
    out = None
    if draw:
        out = bgr.copy()
        cv2.line(out, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        for n, p in enumerate(people):
            x0, y0, x1, y1 = (int(v * s) for v, s in zip(p["box"], (w, h, w, h)))
            col = (0, 255, 0) if n == 0 else (0, 165, 255)
            cv2.rectangle(out, (x0, y0), (x1, y1), col, 2)
            cx = int(p["x_frac"] * w)
            cv2.line(out, (cx, 0), (cx, h), col, 2)
            cv2.putText(out, f"{p['bearing_deg']:+.0f}deg", (x0 + 4, max(18, y0 + 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    return res, out
