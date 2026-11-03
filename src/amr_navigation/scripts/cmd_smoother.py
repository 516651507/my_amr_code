#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TEB 输出的后处理：限幅、高速压角速度、死区、加速度限制，再发到 /autoVel_cmd。

不发布 /cmd_vel。手动速度仍走原来的 /cmd_vel。
"""


class SmootherConfig(object):
    def __init__(self,
                 max_vel_x=1.5,
                 max_vel_x_backwards=0.15,
                 max_vel_theta=1.0,
                 max_acc_x=0.8,
                 max_acc_theta=1.5,
                 timeout_acc_x=1.5,
                 timeout_acc_theta=2.0,
                 cmd_timeout=0.4,
                 w_deadband=0.04,
                 v_deadband_min=0.20,
                 yaw_rate_speed_scale=0.60,
                 min_yaw_fraction=0.35,
                 lpf_alpha=0.55):
        self.max_vel_x = float(max_vel_x)
        self.max_vel_x_backwards = float(max_vel_x_backwards)
        self.max_vel_theta = float(max_vel_theta)
        self.max_acc_x = float(max_acc_x)
        self.max_acc_theta = float(max_acc_theta)
        self.timeout_acc_x = float(timeout_acc_x)
        self.timeout_acc_theta = float(timeout_acc_theta)
        self.cmd_timeout = float(cmd_timeout)
        self.w_deadband = float(w_deadband)
        self.v_deadband_min = float(v_deadband_min)
        self.yaw_rate_speed_scale = float(yaw_rate_speed_scale)
        self.min_yaw_fraction = float(min_yaw_fraction)
        self.lpf_alpha = float(lpf_alpha)


def _clamp(value, low, high):
    return max(low, min(high, value))


class TwistSmoother(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.v = 0.0
        self.w = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = None
        self.last_step = None
        self.have_cmd = False

    def set_command(self, linear_x, angular_z, now):
        self.cmd_v = float(linear_x)
        self.cmd_w = float(angular_z)
        self.last_cmd_time = float(now)
        self.have_cmd = True

    def _shape(self, linear_x, angular_z):
        cfg = self.cfg
        v = _clamp(linear_x, -cfg.max_vel_x_backwards, cfg.max_vel_x)
        speed_ratio = 0.0 if cfg.max_vel_x <= 1e-6 else min(1.0, abs(v) / cfg.max_vel_x)
        fraction = max(cfg.min_yaw_fraction, 1.0 - cfg.yaw_rate_speed_scale * speed_ratio)
        w_lim = cfg.max_vel_theta * fraction
        w = _clamp(angular_z, -w_lim, w_lim)
        if abs(v) >= cfg.v_deadband_min and abs(w) < cfg.w_deadband:
            w = 0.0
        return v, w

    def step(self, now):
        now = float(now)
        if self.last_step is None:
            dt = 0.05
        else:
            dt = _clamp(now - self.last_step, 0.0, 0.2)
        self.last_step = now

        timed_out = (
            self.have_cmd
            and self.last_cmd_time is not None
            and (now - self.last_cmd_time) > self.cfg.cmd_timeout
        )
        if not self.have_cmd or timed_out:
            target_v, target_w = 0.0, 0.0
            acc_x = self.cfg.timeout_acc_x
            acc_w = self.cfg.timeout_acc_theta
        else:
            target_v, target_w = self._shape(self.cmd_v, self.cmd_w)
            acc_x = self.cfg.max_acc_x
            acc_w = self.cfg.max_acc_theta

        alpha = _clamp(self.cfg.lpf_alpha, 0.0, 1.0)
        filtered_v = alpha * target_v + (1.0 - alpha) * self.v
        filtered_w = alpha * target_w + (1.0 - alpha) * self.w
        self.v += _clamp(filtered_v - self.v, -acc_x * dt, acc_x * dt)
        self.w += _clamp(filtered_w - self.w, -acc_w * dt, acc_w * dt)
        if abs(self.v) < 1e-4:
            self.v = 0.0
        if abs(self.w) < 1e-4:
            self.w = 0.0
        return self.v, self.w, timed_out


def run_self_test():
    cfg = SmootherConfig(lpf_alpha=1.0)
    smoother = TwistSmoother(cfg)
    v, w, timed_out = smoother.step(0.0)
    assert (v, w, timed_out) == (0.0, 0.0, False)

    smoother.set_command(1.5, 0.0, 0.0)
    v, w, _ = smoother.step(0.05)
    assert abs(v - 0.04) < 1e-9, v
    assert w == 0.0

    t = 0.05
    for _ in range(80):
        t += 0.05
        smoother.set_command(1.5, 0.0, t)
        v, w, _ = smoother.step(t)
    assert abs(v - 1.5) < 1e-6, v

    fast = TwistSmoother(SmootherConfig(lpf_alpha=1.0))
    fast.set_command(1.5, 0.02, 0.0)
    _, w, _ = fast.step(0.05)
    assert w == 0.0

    yaw = TwistSmoother(SmootherConfig(lpf_alpha=1.0, w_deadband=0.0))
    t = 0.0
    w = 0.0
    for _ in range(80):
        t += 0.05
        yaw.set_command(1.5, 1.0, t)
        _, w, _ = yaw.step(t)
    assert abs(w - 0.4) < 1e-6, w

    stop = TwistSmoother(SmootherConfig(lpf_alpha=1.0, cmd_timeout=0.4, timeout_acc_x=1.5))
    t = 0.0
    for _ in range(40):
        t += 0.05
        stop.set_command(1.5, 0.0, t)
        stop.step(t)
    v_before = stop.v
    v_after, _, timed_out = stop.step(t + 0.5)
    assert timed_out
    assert v_after < v_before
    print("cmd_smoother self-test ok")


def _ros_main():
    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node("cmd_smoother")
    cfg = SmootherConfig(
        max_vel_x=rospy.get_param("~max_vel_x", 1.5),
        max_vel_x_backwards=rospy.get_param("~max_vel_x_backwards", 0.15),
        max_vel_theta=rospy.get_param("~max_vel_theta", 1.0),
        max_acc_x=rospy.get_param("~max_acc_x", 0.8),
        max_acc_theta=rospy.get_param("~max_acc_theta", 1.5),
        timeout_acc_x=rospy.get_param("~timeout_acc_x", 1.5),
        timeout_acc_theta=rospy.get_param("~timeout_acc_theta", 2.0),
        cmd_timeout=rospy.get_param("~cmd_timeout", 0.4),
        w_deadband=rospy.get_param("~w_deadband", 0.04),
        v_deadband_min=rospy.get_param("~v_deadband_min", 0.20),
        yaw_rate_speed_scale=rospy.get_param("~yaw_rate_speed_scale", 0.60),
        min_yaw_fraction=rospy.get_param("~min_yaw_fraction", 0.35),
        lpf_alpha=rospy.get_param("~lpf_alpha", 0.55),
    )
    smoother = TwistSmoother(cfg)
    cmd_in = rospy.get_param("~cmd_in_topic", "/amr/cmd_vel_raw")
    cmd_out = rospy.get_param("~cmd_out_topic", "/autoVel_cmd")
    pub = rospy.Publisher(cmd_out, Twist, queue_size=1)

    def on_twist(msg):
        smoother.set_command(msg.linear.x, msg.angular.z, rospy.Time.now().to_sec())

    rospy.Subscriber(cmd_in, Twist, on_twist, queue_size=1)
    rate = rospy.Rate(float(rospy.get_param("~publish_rate", 20.0)))
    rospy.loginfo("cmd_smoother: %s -> %s，线速度上限 %.2f m/s", cmd_in, cmd_out, cfg.max_vel_x)
    while not rospy.is_shutdown():
        v, w, timed_out = smoother.step(rospy.Time.now().to_sec())
        if timed_out:
            rospy.logwarn_throttle(2.0, "规划速度超时，正在减速到 0")
        out = Twist()
        out.linear.x = v
        out.angular.z = w
        pub.publish(out)
        rate.sleep()


def main():
    import sys
    if "--self-test" in sys.argv:
        run_self_test()
        return
    _ros_main()


if __name__ == "__main__":
    main()
