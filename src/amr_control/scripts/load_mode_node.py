#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""空载/负载档位：同步 max_linear 到 chassis、cmd_smoother、TEB。"""

import rospy
from std_msgs.msg import String


class LoadModeNode(object):
    def __init__(self):
        self.profiles = rospy.get_param("~modes", {
            "empty": {"max_linear_speed": 1.5},
            "loaded": {"max_linear_speed": 0.8},
        })
        self.MODES = tuple(self.profiles.keys())
        default = rospy.get_param("~default_mode", "empty")
        self.pub = rospy.Publisher("/amr/load_mode", String, queue_size=1, latch=True)
        rospy.Subscriber("/amr/load_mode_cmd", String, self._on_cmd, queue_size=1)
        self.set_mode(default, announce=False)

    def set_mode(self, mode, announce=True):
        mode = str(mode).strip().lower()
        if mode not in self.MODES:
            rospy.logwarn("未知 load_mode: %s，可用 %s", mode, self.MODES)
            return False
        speed = float(self.profiles[mode]["max_linear_speed"])
        rospy.set_param("/amr/max_linear_speed", speed)
        rospy.set_param("/amr/load_mode", mode)
        # chassis / smoother 私有参数名
        for prefix in ("/chassis_serial", "/cmd_smoother"):
            try:
                rospy.set_param(prefix + "/max_linear_speed", speed)
                rospy.set_param(prefix + "/max_vel_x", speed)
            except Exception:
                pass
        # TEB（move_base 下插件命名空间）
        for prefix in ("/move_base/TebLocalPlannerROS",
                       "/move_base/teb_local_planner/TebLocalPlannerROS"):
            try:
                rospy.set_param(prefix + "/max_vel_x", speed)
                rospy.set_param(prefix + "/max_vel_trans", speed)
            except Exception:
                pass
        msg = String(data=mode)
        self.pub.publish(msg)
        if announce:
            rospy.loginfo("负载档位 -> %s，max_linear=%.2f m/s", mode, speed)
        return True

    def _on_cmd(self, msg):
        self.set_mode(msg.data)


def main():
    rospy.init_node("load_mode")
    LoadModeNode()
    rospy.spin()


if __name__ == "__main__":
    main()
