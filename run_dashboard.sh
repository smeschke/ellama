#!/bin/sh
# eLlama operator dashboard launcher.
# One line on any fresh machine (from the repo root):  ./run_dashboard.sh [robot]
# First run creates mcp_server/.venv and installs requirements; later runs
# just start the server. Open http://127.0.0.1:8765 in a browser.
# The optional robot name picks calibration/robots/<robot>.json (see that
# dir's README); unset means the default calibration/config.json.
set -e
cd "$(dirname "$0")"

if [ -n "$1" ]; then
  if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "usage: ./run_dashboard.sh [robot]"
    echo "robots:"; ls calibration/robots/*.json 2>/dev/null || echo "  (none yet -- see calibration/robots/README.md)"
    exit 0
  fi
  export ELLAMA_ROBOT="$1"
fi
if [ -n "$ELLAMA_ROBOT" ] && [ ! -f "calibration/robots/$ELLAMA_ROBOT.json" ]; then
  echo "error: no such robot profile calibration/robots/$ELLAMA_ROBOT.json" >&2
  echo "available:"; ls calibration/robots/*.json 2>/dev/null || echo "  (none yet -- see calibration/robots/README.md)"
  exit 1
fi

# Pick a Python new enough for the MCP SDK (3.11+).
PY=""
for c in python3.12 python3.13 python3.11 python3; do
  if command -v "$c" >/dev/null 2>&1; then PY="$c"; break; fi
done
[ -n "$PY" ] || { echo "error: install Python 3.11+ first"; exit 1; }

if [ ! -x mcp_server/.venv/bin/python ]; then
  "$PY" -m venv --system-site-packages mcp_server/.venv
  mcp_server/.venv/bin/pip install -r mcp_server/requirements.txt
fi

# Serial port: explicit env wins; otherwise prefer a USB-serial adapter over
# whatever the OS auto-detect lists first (e.g. Bluetooth ports on macOS).
if [ -z "$ELLAMA_SERIAL_PORT" ]; then
  for p in /dev/cu.usbserial-* /dev/ttyUSB*; do
    if [ -e "$p" ]; then export ELLAMA_SERIAL_PORT="$p"; break; fi
  done
fi

exec mcp_server/.venv/bin/python mcp_server/robot_server.py --http --port "${ELLAMA_HTTP_PORT:-8765}"
