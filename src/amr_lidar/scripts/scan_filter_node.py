#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LDS-50C-E 扫描后处理：无效距离、可选近距剔除，发布 /lidar1/scan_filtered。"""

from __future__ import division

import math

import rospy
from sensor_msgs.msg import LaserScan


class ScanFilterNode(object):
    def __init__(self):
        self.min_range = float(rospy.get_param("~min_range", 0.15))
        self.max_range = float(rospy.get_param("~max_range", 30.0))
        self.drop_zero = bool(rospy.get_param("~drop_zero_as_invalid", True))
        in_topic = rospy.get_param("~scan_in", "/lidar1/laser_fan")
        out_topic = rospy.get_param("~scan_out", "/lidar1/scan_filtered")
        self.pub = rospy.Publisher(out_topic, LaserScan, queue_size=1)
        rospy.Subscriber(in_topic, LaserScan, self._cb, queue_size=1)
        rospy.loginfo("scan_filter: %s -> %s (%.2f..%.2f m)", in_topic, out_topic, self.min_range, self.max_range)

    def _cb(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, self.min_range)
        out.range_max = min(msg.range_max, self.max_range)
        out.ranges = []
        out.intensities = []
        has_intensity = len(msg.intensities) == len(msg.ranges)
        for i, raw in enumerate(msg.ranges):
            value = raw
            if not math.isfinite(value):
                value = float("inf")
            elif self.drop_zero and abs(value) < 1e-6:
                value = float("inf")
            elif value < out.range_min or value > out.range_max:
                value = float("inf")
            out.ranges.append(value)
            if has_intensity:
                out.intensities.append(msg.intensities[i])
        self.pub.publish(out)


def main():
    rospy.init_node("lidar_scan_filter")
    ScanFilterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
