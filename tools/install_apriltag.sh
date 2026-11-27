#!/bin/sh
# Build the official AprilTag library (https://github.com/AprilRobotics/apriltag)
# and its Python bindings into mcp_server/.venv, so `import apriltag` works there.
# It is not on PyPI, so this replaces a requirements.txt line.
#
#   tools/install_apriltag.sh            # from the repo root
#
# Needs: git, cmake, a C compiler, and numpy importable from the venv's Python.
set -e
cd "$(dirname "$0")/.."
VENV="$(pwd)/mcp_server/.venv"
PY="$VENV/bin/python"
TAG=v3.4.5

[ -x "$PY" ] || { echo "error: no $PY -- run ./run_dashboard.sh once to create the venv" >&2; exit 1; }
"$PY" -c "import numpy" 2>/dev/null || { echo "error: numpy missing in the venv" >&2; exit 1; }
command -v cmake >/dev/null || { echo "error: install cmake first" >&2; exit 1; }

SRC="$(mktemp -d)"
trap 'rm -rf "$SRC"' EXIT
git clone -q --depth 1 --branch "$TAG" https://github.com/AprilRobotics/apriltag "$SRC/apriltag"
cmake -S "$SRC/apriltag" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF \
      -DPython3_EXECUTABLE="$PY" \
      -DCMAKE_INSTALL_PREFIX="$VENV" \
      -DCMAKE_INSTALL_RPATH='$ORIGIN/../..' >/dev/null
cmake --build "$SRC/build" -j"$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)" >/dev/null
cmake --install "$SRC/build" >/dev/null
"$PY" -c "import apriltag; apriltag.apriltag('tag36h11'); print('apriltag $TAG installed in $VENV')"
