# -*- coding: utf-8 -*-
"""Flask 路由（不依赖 rospy，便于离线验证）。"""

from __future__ import division

import os
import subprocess

from config_store import (
    list_usb_sysfs_hints,
    load_yaml,
    merge_camera_config,
    save_yaml,
)

try:
    from flask import Flask, jsonify, request, send_from_directory
except ImportError:
    Flask = None


def create_app(state, www_dir, camera_config_path):
    if Flask is None:
        raise RuntimeError("需要 Flask")

    app = Flask(__name__, static_folder=www_dir)

    @app.route("/")
    def index():
        return send_from_directory(www_dir, "index.html")

    @app.route("/debug")
    def debug_page():
        return send_from_directory(www_dir, "debug.html")

    @app.route("/api/status")
    def api_status():
        return jsonify(state.status_dict())

    @app.route("/api/load_mode", methods=["GET"])
    def api_get_mode():
        return jsonify({"mode": state.load_mode})

    @app.route("/api/load_mode", methods=["POST"])
    def api_set_mode():
        data = request.get_json(silent=True) or {}
        mode = data.get("mode", "empty")
        state.set_load_mode(mode)
        return jsonify({"ok": True, "mode": mode})

    @app.route("/api/cmd_vel", methods=["POST"])
    def api_cmd():
        data = request.get_json(silent=True) or {}
        state.publish_cmd(data.get("linear", 0.0), data.get("angular", 0.0))
        return jsonify({"ok": True})

    @app.route("/api/cmd_vel/stop", methods=["POST"])
    def api_stop():
        state.stop()
        return jsonify({"ok": True})

    @app.route("/api/debug/config/camera", methods=["GET"])
    def api_get_camera_config():
        if not camera_config_path:
            return jsonify({"ok": False, "error": "camera_config_path 未设置"}), 500
        try:
            data = load_yaml(camera_config_path)
        except (OSError, RuntimeError) as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500
        return jsonify({
            "ok": True,
            "path": camera_config_path,
            "config": data,
            "apply_hint": "保存后请重启：roslaunch amr_camera dw2_dual.launch start_cameras:=true",
        })

    @app.route("/api/debug/config/camera", methods=["POST"])
    def api_set_camera_config():
        if not camera_config_path:
            return jsonify({"ok": False, "error": "camera_config_path 未设置"}), 500
        patch = request.get_json(silent=True) or {}
        try:
            current = load_yaml(camera_config_path)
            merged = merge_camera_config(current, patch)
            save_yaml(camera_config_path, merged)
        except (OSError, RuntimeError, ValueError) as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        return jsonify({
            "ok": True,
            "config": merged,
            "apply_hint": "已写入 YAML；重启 DW2 驱动后生效",
        })

    @app.route("/api/debug/usb/hints", methods=["GET"])
    def api_usb_hints():
        hints = list_usb_sysfs_hints()
        lsusb = []
        try:
            out = subprocess.check_output(["lsusb"], stderr=subprocess.STDOUT, timeout=3, text=True)
            lsusb = [line.strip() for line in out.splitlines() if line.strip()]
        except (subprocess.SubprocessError, FileNotFoundError, OSError):
            pass
        return jsonify({"ok": True, "sysfs": hints, "lsusb": lsusb})

    return app
