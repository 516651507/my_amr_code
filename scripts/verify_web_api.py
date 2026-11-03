#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无 ROS 环境下验证 Web REST 与 config_store。"""

import os
import sys
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WEB_SCRIPTS = os.path.join(ROOT, "src/amr_web/scripts")
sys.path.insert(0, WEB_SCRIPTS)

from config_store import load_yaml, merge_camera_config, save_yaml  # noqa: E402


class MockState(object):
    load_mode = "empty"

    def status_dict(self):
        return {"load_mode": self.load_mode, "max_linear_speed": 1.5, "odom": None, "battery": None}

    def publish_cmd(self, linear, angular):
        self.last_cmd = (linear, angular)

    def stop(self):
        self.last_cmd = (0.0, 0.0)

    def set_load_mode(self, mode):
        self.load_mode = mode


def test_config_store():
    tmp = tempfile.mkdtemp(prefix="amr_cfg_")
    path = os.path.join(tmp, "dw2_dual.yaml")
    base = {
        "camera1": {"usb_port": "", "points_topic": "/camera/depth/points_cam"},
        "camera2": {"usb_port": ""},
    }
    save_yaml(path, base)
    merged = merge_camera_config(load_yaml(path), {
        "camera1": {"usb_port": "2-1.1", "serial_number": "SN1"},
        "camera2": {"usb_port": "2-1.2"},
    })
    assert merged["camera1"]["usb_port"] == "2-1.1"
    assert merged["camera1"]["points_topic"] == "/camera/depth/points_cam"
    save_yaml(path, merged)
    reloaded = load_yaml(path)
    assert reloaded["camera2"]["usb_port"] == "2-1.2"
    print("config_store ok")


def test_flask_routes():
    from web_routes import create_app

    www = os.path.join(ROOT, "src/amr_web/www")
    tmp = tempfile.mkdtemp(prefix="amr_yaml_")
    cfg_path = os.path.join(tmp, "dw2_dual.yaml")
    save_yaml(cfg_path, {"camera1": {"usb_port": ""}, "camera2": {"usb_port": ""}})

    app = create_app(MockState(), www, cfg_path)
    client = app.test_client()

    assert client.get("/").status_code == 200
    assert client.get("/debug").status_code == 200
    st = client.get("/api/status").get_json()
    assert st["load_mode"] == "empty"

    r = client.post("/api/cmd_vel", json={"linear": 0.3, "angular": 0.0})
    assert r.get_json()["ok"] is True

    r = client.post("/api/load_mode", json={"mode": "loaded"})
    assert r.get_json()["ok"] is True

    cam = client.get("/api/debug/config/camera").get_json()
    assert cam["ok"] and "camera1" in cam["config"]

    r = client.post("/api/debug/config/camera", json={
        "camera1": {"usb_port": "3-1.1"},
        "camera2": {"usb_port": "3-1.2"},
    })
    assert r.get_json()["ok"] is True
    assert load_yaml(cfg_path)["camera1"]["usb_port"] == "3-1.1"

    hints = client.get("/api/debug/usb/hints").get_json()
    assert hints["ok"] and "sysfs" in hints

    print("flask routes ok")


def main():
    test_config_store()
    test_flask_routes()
    print("verify_web_api ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
