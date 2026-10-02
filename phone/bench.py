"""Phone-side feasibility test for turn-to-me: run the pose model on one image, time it, and
print the person bearing. Needs only numpy, onnxruntime and Pillow (no torch / opencv / ultralytics).

  python bench.py pose_320x192.onnx frame.jpg [runs]

Mirrors person_vision.py: keypoints with conf >= 0.4 vote for the person's horizontal center,
bearing is the angle from the camera axis (+ = right of centre), HFOV 51 deg.
"""
import math
import sys
import time

import numpy as np
import onnxruntime as ort
from PIL import Image

HFOV_DEG = 51.0
MIN_PERSON_CONF, MIN_KP_CONF, MIN_BOX_AREA_FRAC = 0.35, 0.4, 0.02


def bearing_deg(x_frac):
    f = 0.5 / math.tan(math.radians(HFOV_DEG) / 2)
    return math.degrees(math.atan((x_frac - 0.5) / f))


def nms(boxes, scores, thr=0.5):
    order = scores.argsort()[::-1]
    keep = []
    while len(order):
        i = order[0]
        keep.append(i)
        if len(order) == 1:
            break
        r = order[1:]
        xx0 = np.maximum(boxes[i, 0], boxes[r, 0]); yy0 = np.maximum(boxes[i, 1], boxes[r, 1])
        xx1 = np.minimum(boxes[i, 2], boxes[r, 2]); yy1 = np.minimum(boxes[i, 3], boxes[r, 3])
        inter = np.maximum(0, xx1 - xx0) * np.maximum(0, yy1 - yy0)
        a = lambda b: (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])
        iou = inter / (a(boxes[i]) + a(boxes[r]) - inter + 1e-9)
        order = r[iou < thr]
    return keep


def prepare(img, in_h, in_w):
    """Letterbox a PIL image into (in_h, in_w), grey padding. Returns NCHW float32 + (scale, pad_x, pad_y)."""
    w, h = img.size
    s = min(in_w / w, in_h / h)
    nw, nh = round(w * s), round(h * s)
    px, py = (in_w - nw) // 2, (in_h - nh) // 2
    canvas = Image.new("RGB", (in_w, in_h), (114, 114, 114))
    canvas.paste(img.resize((nw, nh), Image.BILINEAR), (px, py))
    x = np.asarray(canvas, dtype=np.float32).transpose(2, 0, 1)[None] / 255.0
    return x, (s, px, py)


def detect(sess, img):
    in_h, in_w = sess.get_inputs()[0].shape[2:]
    x, (s, px, py) = prepare(img, in_h, in_w)
    out = sess.run(None, {sess.get_inputs()[0].name: x})[0][0].T      # (anchors, 56)
    out = out[out[:, 4] >= MIN_PERSON_CONF]
    W, H = img.size
    people = []
    if len(out):
        xc, yc, bw, bh = out[:, 0], out[:, 1], out[:, 2], out[:, 3]
        boxes = np.stack([xc - bw / 2, yc - bh / 2, xc + bw / 2, yc + bh / 2], 1)
        for i in nms(boxes, out[:, 4]):
            x0, y0, x1, y1 = (boxes[i] - [px, py, px, py]) / s
            if (x1 - x0) * (y1 - y0) / (W * H) < MIN_BOX_AREA_FRAC:
                continue
            kp = out[i, 5:].reshape(17, 3)
            xs = [(k[0] - px) / s for k in kp if k[2] >= MIN_KP_CONF]
            if len(xs) >= 2:
                cx, src = sum(xs) / len(xs), "keypoints"
            else:
                cx, src = (x0 + x1) / 2, "box"
            people.append(dict(x_frac=round(float(cx) / W, 4), bearing_deg=round(bearing_deg(float(cx) / W), 1),
                               conf=round(float(out[i, 4]), 2), n_kp=len(xs), source=src,
                               area=(x1 - x0) * (y1 - y0)))
    people.sort(key=lambda p: -p.pop("area"))
    return people


if __name__ == "__main__":
    model, image = sys.argv[1], sys.argv[2]
    runs = int(sys.argv[3]) if len(sys.argv) > 3 else 20
    opts = ort.SessionOptions()
    import os
    if os.environ.get("ORT_THREADS"):
        opts.intra_op_num_threads = int(os.environ["ORT_THREADS"])
    sess = ort.InferenceSession(model, opts, providers=["CPUExecutionProvider"])
    img = Image.open(image).convert("RGB")
    print("model input", sess.get_inputs()[0].shape, "image", img.size)
    people = detect(sess, img)                     # warm-up
    times = []
    for _ in range(runs):
        t = time.perf_counter(); people = detect(sess, img); times.append(time.perf_counter() - t)
    times.sort()
    print(f"{len(people)} person(s): {people}")
    print(f"per frame (incl. resize+decode): median {times[len(times)//2]*1000:.0f} ms, "
          f"min {times[0]*1000:.0f} ms, max {times[-1]*1000:.0f} ms over {runs} runs")
