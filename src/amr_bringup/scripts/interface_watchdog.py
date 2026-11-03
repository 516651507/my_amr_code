#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查硬件说明里的传感器话题是否在发布。不补假数据。"""

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
        self.auto_cmd = rospy.get_param("~auto_cmd_topic", "/autoVel_cmd")
        self.manual_cmd = rospy.get_param("~manual_cmd_topic", "/cmd_vel")
        self.seen = {topic: False for topic in self.expected}
        self.types = {}
        self.pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        rospy.Timer(rospy.Duration(2.0), self._report)
        rospy.loginfo(
            "接口看门狗已启动。自动速度话题 %s，手动速度话题 %s（本节点不发布速度）",
            self.auto_cmd, self.manual_cmd)

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

    def _report(self, _event):
        self._snapshot()
        missing = [topic for topic, ok in self.seen.items() if not ok]
        status = DiagnosticStatus()
        status.name = "amr_bringup/interfaces"
        status.hardware_id = "amr"
        if missing:
            status.level = DiagnosticStatus.WARN
            status.message = "缺少话题: " + ", ".join(missing)
        else:
            status.level = DiagnosticStatus.OK
            status.message = "传感器话题都已出现"
        for topic in self.expected:
            status.values.append(KeyValue(
                key=topic,
                value=self.types.get(topic, "MISSING" if not self.seen[topic] else "SEEN")))
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
