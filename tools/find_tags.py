#!/usr/bin/env python3
"""Find AprilTags in an image and print each tag's id and position.
Uses the official AprilRobotics library (install it with tools/install_apriltag.sh).
    mcp_server/.venv/bin/python tools/find_tags.py photo.jpg [--family tag36h11]
"""

import argparse
import sys

import cv2
from apriltag import apriltag


def detect(gray, family="tag36h11", detector=None):
    """[(id, (center_x, center_y), corners)] in pixels. Pass a detector to reuse one."""
    return [(d["id"], tuple(d["center"]), d["lb-rb-rt-lt"])
            for d in (detector or apriltag(family)).detect(gray)]


def find_tags(image_path, family="tag36h11"):
    gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        sys.exit("could not read image %s" % image_path)
    return detect(gray, family)


def main():
    ap = argparse.ArgumentParser(description="Find AprilTags in an image.")
    ap.add_argument("image")
    ap.add_argument("--family", default="tag36h11")
    args = ap.parse_args()
    tags = find_tags(args.image, args.family)
    if not tags:
        print("no tags found")
    for tag_id, (x, y), _ in tags:
        print("tag %d at (%.0f, %.0f)" % (tag_id, x, y))


if __name__ == "__main__":
    main()
