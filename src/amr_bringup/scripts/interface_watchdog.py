#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查传感器与底盘话题是否在发布。不补假数据。"""

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class InterfaceWatchdog(object):
    def __init__(self):
        self.expected = rospy.get_param("~expected_topics", [
            "/lidar1/scan_filtered",
            "/wheel_odom",
            "/camera/depth/points_cam",
            "/camera2/depth/points_cam",
        ])
        self.optional = rospy.get_param("~optional_topics", [])
        self.auto_cmd = rospy.get_param("~auto_cmd_topic", "/autoVel_cmd")
        self.manual_cmd = rospy.get_param("~manual_cmd_topic", "/cmd_vel")
        hw = rospy.get_param("~hardware", {})
        self.lidar_model = hw.get("lidar_model", "LDS-50C-E")
        self.camera_model = hw.get("camera_model", "Orbbec_Dabai_DW2")

        self.seen = {topic: False for topic in self.expected}
        self.optional_seen = {topic: False for topic in self.optional}
        self.types = {}
        self.pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        rospy.Timer(rospy.Duration(2.0), self._report)
        rospy.loginfo(
            "接口看门狗：雷达 %s，相机 %s；自动 %s，手动 %s（本节点不发布速度）",
            self.lidar_model, self.camera_model, self.auto_cmd, self.manual_cmd)

    def _snapshot(self):
        try:
            published = rospy.get_published_topics()
        except rospy.ROSException as exc:
            rospy.logwarn_throttle(10.0, "读取话题列表失败: %s", exc)
            return
        advertised = {name: msg_type for name, msg_type in published}
        for topic in self.expected:
            if topic in advertised:
                self.seen[topic] = True
                self.types[topic] = advertised[topic]
        for topic in self.optional:
            if topic in advertised:
                self.optional_seen[topic] = True
                self.types[topic] = advertised[topic]

    def _report(self, _event):
        self._snapshot()
        missing = [topic for topic, ok in self.seen.items() if not ok]
        status = DiagnosticStatus()
        status.name = "amr_bringup/interfaces"
        status.hardware_id = "amr"
        if missing:
            status.level = DiagnosticStatus.WARN
            status.message = "缺少必需话题: " + ", ".join(missing)
        else:
            status.level = DiagnosticStatus.OK
            status.message = "必需传感器话题都已出现"
        for topic in self.expected:
            status.values.append(KeyValue(
                key=topic,
                value=self.types.get(topic, "MISSING" if not self.seen[topic] else "SEEN")))
        for topic in self.optional:
            if not topic:
                continue
            seen = self.optional_seen.get(topic, False)
            status.values.append(KeyValue(
                key=topic + " (optional)",
                value=self.types.get(topic, "MISSING" if not seen else "SEEN")))
        status.values.append(KeyValue(key="lidar_model", value=str(self.lidar_model)))
        status.values.append(KeyValue(key="camera_model", value=str(self.camera_model)))
        status.values.append(KeyValue(key="auto_cmd_topic", value=self.auto_cmd))
        status.values.append(KeyValue(key="manual_cmd_topic", value=self.manual_cmd))

        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub.publish(array)
        if missing:
            rospy.logwarn_throttle(10.0, "尚未发布: %s", ", ".join(missing))


def main():
    rospy.init_node("interface_watchdog")
    InterfaceWatchdog()
    rospy.spin()


if __name__ == "__main__":
    main()
