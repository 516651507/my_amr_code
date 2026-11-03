#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 /wheel_odom 转成 odom -> base_link。

底盘若已经在发这条 TF，把 launch 参数 publish_wheel_odom_tf 设为 false，
否则树里会出现两个父节点。
"""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def normalize_quaternion(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


class OdomTfBridge(object):
    def __init__(self):
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.force_frames = bool(rospy.get_param("~force_frames", True))
        topic = rospy.get_param("~odom_topic", "/wheel_odom")
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(topic, Odometry, self._on_odom, queue_size=20)
        rospy.loginfo("odom_tf_bridge: %s -> TF %s -> %s", topic, self.odom_frame, self.base_frame)

    def _on_odom(self, msg):
        parent = self.odom_frame if self.force_frames or not msg.header.frame_id else msg.header.frame_id
        child = self.base_frame if self.force_frames or not msg.child_frame_id else msg.child_frame_id
        if parent == child:
            rospy.logwarn_throttle(5.0, "里程计父子坐标系相同 (%s)，跳过 TF", parent)
            return

        pose = msg.pose.pose
        qx, qy, qz, qw = normalize_quaternion(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.broadcaster.sendTransform(transform)


def main():
    rospy.init_node("odom_tf_bridge")
    OdomTfBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
