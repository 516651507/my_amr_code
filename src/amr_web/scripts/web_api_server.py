#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HTTP REST + 静态 Web 页；手动速度发 /cmd_vel。"""

from __future__ import division

import json
import os
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

try:
    from flask import Flask, jsonify, request, send_from_directory
except ImportError:
    Flask = None


class RosBridgeState(object):
    def __init__(self):
        self.odom = None
        self.battery = None
        self.load_mode = "empty"
        self.lock = threading.Lock()
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.mode_cmd_pub = rospy.Publisher("/amr/load_mode_cmd", String, queue_size=1)
        rospy.Subscriber("/wheel_odom", Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber("/battery_state", BatteryState, self._on_bat, queue_size=1)
        rospy.Subscriber("/amr/load_mode", String, self._on_mode, queue_size=1)

    def _on_odom(self, msg):
        with self.lock:
            self.odom = msg

    def _on_bat(self, msg):
        with self.lock:
            self.battery = msg

    def _on_mode(self, msg):
        with self.lock:
            self.load_mode = msg.data

    def status_dict(self):
        with self.lock:
            odom = self.odom
            bat = self.battery
            mode = self.load_mode
        max_v = float(rospy.get_param("/amr/max_linear_speed", 1.5))
        out = {
            "load_mode": mode,
            "max_linear_speed": max_v,
            "odom": None,
            "battery": None,
        }
        if odom is not None:
            p = odom.pose.pose.position
            out["odom"] = {
                "x": p.x, "y": p.y,
                "vx": odom.twist.twist.linear.x,
                "omega": odom.twist.twist.angular.z,
            }
        if bat is not None:
            out["battery"] = {
                "percentage": bat.percentage,
                "voltage": bat.voltage,
                "current": bat.current,
            }
        return out

    def publish_cmd(self, linear, angular):
        msg = Twist()
        max_v = float(rospy.get_param("/amr/max_linear_speed", 1.5))
        msg.linear.x = max(-max_v, min(max_v, float(linear)))
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def set_load_mode(self, mode):
        self.mode_cmd_pub.publish(String(data=str(mode)))


def create_app(state, www_dir):
    app = Flask(__name__, static_folder=www_dir)

    @app.route("/")
    def index():
        return send_from_directory(www_dir, "index.html")

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

    return app


def main():
    if Flask is None:
        rospy.logfatal("需要 Flask: sudo apt install python3-flask")
        return
    rospy.init_node("web_api_server")
    www_dir = rospy.get_param("~www_dir", "")
    if not www_dir:
        www_dir = os.path.join(os.path.dirname(__file__), "..", "www")
    www_dir = os.path.abspath(www_dir)
    host = rospy.get_param("~host", "0.0.0.0")
    port = int(rospy.get_param("~port", 8080))

    state = RosBridgeState()
    app = create_app(state, www_dir)

    def run_flask():
        app.run(host=host, port=port, threaded=True, use_reloader=False)

    thread = threading.Thread(target=run_flask)
    thread.daemon = True
    thread.start()
    rospy.loginfo("Web API http://%s:%d  www=%s", host, port, www_dir)
    rospy.on_shutdown(state.stop)
    rospy.spin()


if __name__ == "__main__":
    main()
