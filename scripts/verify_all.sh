#!/bin/bash
# 本地 CI：不依赖 ROS Noetic 的验证 + Web 冒烟
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT}"
echo "=== verify_scaffold ==="
python3 scripts/verify_scaffold.py
echo "=== web smoke ==="
scripts/run_web_smoke.sh
echo "=== verify_all ok ==="
