#!/usr/bin/env bash
# One-time setup on the Ubuntu laptop that talks to the ANAFI.
# Creates .venv (Python 3.10-3.12; Olympe breaks on 3.13) with Olympe + server deps,
# then runs the mock-drone smoke test. Does not touch the drone.
set -euo pipefail
cd "$(dirname "$0")"

PY=""
for c in python3.12 python3.11 python3.10 python3; do
  if command -v "$c" >/dev/null; then
    v=$("$c" -c 'import sys; print(sys.version_info[:2] <= (3,12) and sys.version_info[:2] >= (3,8))')
    if [ "$v" = "True" ]; then PY="$c"; break; fi
  fi
done
if [ -z "$PY" ]; then
  echo "Need Python 3.8-3.12 for Olympe. Try: sudo apt install python3.12 python3.12-venv"; exit 1
fi
echo "[1/3] using $($PY --version)"

if [ ! -d .venv ]; then
  "$PY" -m venv .venv || { echo "venv failed. Try: sudo apt install ${PY}-venv"; exit 1; }
fi
. .venv/bin/activate
python -m pip install -q --upgrade pip
echo "[2/3] installing parrot-olympe (+ deps its package misses) and server deps"
python -m pip install -q parrot-olympe certifi pysdl2 PyOpenGL flask gevent pymitter cflib psutil
python -c "import olympe" 2>/dev/null && echo "      olympe import ok"

echo "[3/3] smoke test with mock drones"
python bench/run_bench.py --n 1 3 --secs 5 --label smoke-ubuntu

cat <<'MSG'

Done. Next:
  1. Join the drone's ANAFI-xxxx Wi-Fi from the Wi-Fi menu (key is on the label under the battery).
  2. . .venv/bin/activate && python bench/anafi_check.py      # ground check, no flight
MSG
