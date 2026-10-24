#!/usr/bin/env python3
"""Find ArUco tags in an image and report where they are relative to the camera.

Returns, per tag: its id, the BEARING (degrees off the camera's centre line, negative =
left) and a RANGE estimate. Bearing is the trustworthy number -- it needs only the lens
field of view. Range comes from the tag's apparent size, which needs the tag's real size
and an uncalibrated-focal-length guess, so treat it as a rough gate, not a measurement.

    python3 tools/find_tags.py photo.jpg
    python3 tools/find_tags.py photo.jpg --tag-mm 250 --save marked.png
    python3 tools/find_tags.py --make 3,7,9,12 --out-dir printable/
    python3 tools/find_tags.py --selftest

The defaults assume DICT_4X4_50 (ids 0-49) and a ~65 degree horizontal field of view,
which is typical of a phone's main camera.
"""

import argparse, math, os, sys

import cv2
import numpy as np

DEFAULT_DICT = "DICT_4X4_50"
DEFAULT_HFOV = 65.0      # degrees, horizontal
DEFAULT_TAG_MM = 250.0   # 10in: ~40px across at 10ft in a 640px frame, comfortably detectable


def detector_for(dict_name):
    if not hasattr(cv2.aruco, dict_name):
        sys.exit(f"unknown dictionary {dict_name!r}")
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters()), d


def find_tags(img, dict_name=DEFAULT_DICT, tag_mm=DEFAULT_TAG_MM, hfov_deg=DEFAULT_HFOV):
    """-> list of {id, bearing_deg, range_mm, px, center}. Empty list means none seen,
    which is NOT the same as the camera being unable to see -- the caller keeps those apart."""
    detector, _ = detector_for(dict_name)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []

    h, w = gray.shape[:2]
    focal_px = (w / 2) / math.tan(math.radians(hfov_deg / 2))

    out = []
    for quad, tag_id in zip(corners, ids.flatten()):
        pts = quad.reshape(4, 2)
        cx, cy = pts.mean(axis=0)
        # mean of the four edge lengths: steadier than one side when the tag is angled
        side_px = float(np.mean([np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]))
        out.append({
            "id": int(tag_id),
            "bearing_deg": round(math.degrees(math.atan2(cx - w / 2, focal_px)), 2),
            "range_mm": round(focal_px * tag_mm / side_px) if side_px > 0 else None,
            "px": round(side_px, 1),
            "center": (round(float(cx), 1), round(float(cy), 1)),
        })
    return sorted(out, key=lambda t: t["id"])


def annotate(img, tags, dict_name=DEFAULT_DICT):
    detector, _ = detector_for(dict_name)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    corners, ids, _ = detector.detectMarkers(gray)
    marked = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(marked, corners, ids)
    for t in tags:
        cv2.putText(marked, f"{t['id']}: {t['bearing_deg']:+.1f}deg {t['range_mm']}mm",
                    (int(t["center"][0]) - 60, int(t["center"][1]) - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.line(marked, (marked.shape[1] // 2, 0), (marked.shape[1] // 2, marked.shape[0]),
             (120, 120, 120), 1)          # centre line: bearing 0
    return marked


def make_tags(ids, out_dir, dict_name=DEFAULT_DICT, px=1000, border_px=100):
    _, d = detector_for(dict_name)
    os.makedirs(out_dir, exist_ok=True)
    for tag_id in ids:
        img = cv2.aruco.generateImageMarker(d, tag_id, px)
        framed = cv2.copyMakeBorder(img, *([border_px] * 4), cv2.BORDER_CONSTANT, value=255)
        path = os.path.join(out_dir, f"tag_{tag_id:02d}.png")
        cv2.imwrite(path, framed)
        print(f"wrote {path}")
    print(f"\nPrint them all at the SAME physical size and pass it as --tag-mm.")
    print("The white border matters -- ArUco needs the quiet zone to find the edges.")


def selftest(dict_name=DEFAULT_DICT):
    """Render a known tag into a blank frame, off-centre, and detect it back."""
    _, d = detector_for(dict_name)
    marker = cv2.aruco.generateImageMarker(d, 9, 200)
    frame = np.full((480, 640), 255, dtype=np.uint8)
    frame[140:340, 380:580] = marker          # right of centre -> bearing must be positive
    tags = find_tags(frame, dict_name)
    print(f"detected: {tags}")
    ok = len(tags) == 1 and tags[0]["id"] == 9 and tags[0]["bearing_deg"] > 0
    print("PASS -- detector works and bearing sign is right" if ok else "FAIL")
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("image", nargs="?", help="image file to search")
    p.add_argument("--dict", default=DEFAULT_DICT, help=f"ArUco dictionary ({DEFAULT_DICT})")
    p.add_argument("--tag-mm", type=float, default=DEFAULT_TAG_MM,
                   help=f"real tag size in mm, used for range ({DEFAULT_TAG_MM:g})")
    p.add_argument("--hfov", type=float, default=DEFAULT_HFOV,
                   help=f"camera horizontal field of view in degrees ({DEFAULT_HFOV:g})")
    p.add_argument("--save", help="write an annotated copy here")
    p.add_argument("--make", help="generate printable tags, e.g. 3,7,9,12")
    p.add_argument("--out-dir", default="printable", help="where --make writes")
    p.add_argument("--selftest", action="store_true", help="check the detector with no camera")
    a = p.parse_args()

    if a.selftest:
        return selftest(a.dict)
    if a.make:
        return make_tags([int(x) for x in a.make.split(",")], a.out_dir, a.dict)
    if not a.image:
        p.error("give an image, or --make, or --selftest")

    img = cv2.imread(a.image)
    if img is None:
        sys.exit(f"could not read {a.image!r}")

    tags = find_tags(img, a.dict, a.tag_mm, a.hfov)
    if not tags:
        print(f"no tags found ({img.shape[1]}x{img.shape[0]}, dict {a.dict})")
    for t in tags:
        side = "left" if t["bearing_deg"] < 0 else "right"
        print(f"tag {t['id']:3d}  bearing {t['bearing_deg']:+7.2f} deg ({side})  "
              f"range ~{t['range_mm']} mm  [{t['px']}px across]")
        if t["px"] < 25:
            print(f"          ^ only {t['px']}px -- near the detection limit, "
                  "expect dropouts at this distance")
    if a.save:
        cv2.imwrite(a.save, annotate(img, tags, a.dict))
        print(f"\nwrote {a.save}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
