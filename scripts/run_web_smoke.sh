#!/bin/bash
# 无 ROS 的 Web 冒烟测试：启动 Flask 并 curl 关键 API
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="${ROOT}/src/amr_web/scripts:${PYTHONPATH:-}"
TMP="$(mktemp -d)"
CFG="${TMP}/dw2_dual.yaml"
cp "${ROOT}/src/amr_camera/config/dw2_dual.yaml" "${CFG}"
PORT="${PORT:-18080}"

python3 - <<PY &
import os, sys
sys.path.insert(0, "${ROOT}/src/amr_web/scripts")
from web_routes import create_app

class S:
    load_mode = "empty"
    def status_dict(self):
        return {"load_mode": self.load_mode, "max_linear_speed": 1.5, "odom": None, "battery": None}
    def publish_cmd(self, l, a): pass
    def stop(self): pass
    def set_load_mode(self, m): self.load_mode = m

www = os.path.join("${ROOT}", "src/amr_web/www")
app = create_app(S(), www, "${CFG}")
app.run(host="127.0.0.1", port=${PORT}, threaded=True)
PY
PID=$!
sleep 1
curl -sf "http://127.0.0.1:${PORT}/api/status" | head -c 200
echo ""
curl -sf "http://127.0.0.1:${PORT}/debug" | head -c 80
echo ""
curl -sf -X POST "http://127.0.0.1:${PORT}/api/debug/config/camera" \
  -H 'Content-Type: application/json' \
  -d '{"camera1":{"usb_port":"9-9.1"},"camera2":{"usb_port":"9-9.2"}}' | head -c 200
echo ""
grep '9-9.1' "${CFG}"
kill "$PID" 2>/dev/null || true
echo "web smoke ok"
