#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""STM32 串口底盘 ROS 节点：里程计、IMU、电池、速度下发。"""

from __future__ import division

import math
import threading

import rospy
import serial
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from tf2_ros import TransformBroadcaster

from amr_chassis.diff_odometry import DiffOdometry
from amr_chassis.stm32_protocol import (
    CM_DEV_STA_TYPE,
    COMID_MOTOR_COMMUN,
    CM_DEV_CTL_TYPE,
    STAID_BATTERY_DATAS,
    STAID_IMU_DATAS,
    STAID_MOTOR_DATAS,
    FrameParser,
    build_frame,
    decode_battery,
    decode_imu,
    decode_motor_encoder,
    encoder_to_wheel_speeds_mars,
    imu_omg_z_to_rad_s,
    pack_motor_command,
    twist_to_motor_rpm,
)


class ChassisSerialNode(object):
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyS4")
        self.baud = int(rospy.get_param("~baud_rate", 115200))
        self.wheel_diameter = float(rospy.get_param("~wheel_diameter", 0.165))
        self.wheel_spacing = float(rospy.get_param("~wheel_spacing", 0.42))
        self.max_linear = float(rospy.get_param("~max_linear_speed", 1.5))
        self.max_angular = float(rospy.get_param("~max_angular_speed", 1.5))
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.odom_topic = rospy.get_param("~odom_topic", "/wheel_odom")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.battery_topic = rospy.get_param("~battery_topic", "/battery_state")
        self.manual_topic = rospy.get_param("~manual_cmd_topic", "/cmd_vel")
        self.auto_topic = rospy.get_param("~auto_cmd_topic", "/autoVel_cmd")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))
        self.auto_priority = bool(rospy.get_param("~auto_cmd_priority", True))
        self.auto_timeout = float(rospy.get_param("~auto_cmd_timeout", 0.5))

        self.odom = DiffOdometry(self.wheel_spacing)
        self._last_odom_time = rospy.Time.now().to_sec()
        self._lock = threading.Lock()
        self._last_manual = (0.0, 0.0, 0.0)
        self._last_auto = (0.0, 0.0, 0.0)
        self._last_auto_time = 0.0

        self._serial = None
        self._open_serial()

        self._tf = TransformBroadcaster()
        self._odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self._imu_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
        self._bat_pub = rospy.Publisher(self.battery_topic, BatteryState, queue_size=2)

        rospy.Subscriber(self.manual_topic, Twist, self._on_manual, queue_size=1)
        rospy.Subscriber(self.auto_topic, Twist, self._on_auto, queue_size=1)

        self._parser = FrameParser(self._on_frame)
        self._read_timer = rospy.Timer(rospy.Duration(0.02), self._read_serial)
        self._cmd_timer = rospy.Timer(rospy.Duration(0.02), self._send_motor_if_needed)

        rospy.loginfo(
            "amr_chassis: %s @ %d, odom=%s, cmd %s / %s",
            self.port, self.baud, self.odom_topic, self.manual_topic, self.auto_topic)

    def _open_serial(self):
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.01,
                write_timeout=0.05,
            )
        except serial.SerialException as exc:
            rospy.logfatal("无法打开串口 %s: %s", self.port, exc)
            raise

    def _on_manual(self, msg):
        with self._lock:
            self._last_manual = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_auto(self, msg):
        with self._lock:
            self._last_auto = (msg.linear.x, msg.linear.y, msg.angular.z)
            self._last_auto_time = rospy.Time.now().to_sec()

    def _active_twist(self):
        now = rospy.Time.now().to_sec()
        with self._lock:
            if self.auto_priority and (now - self._last_auto_time) <= self.auto_timeout:
                lx, _, az = self._last_auto
            else:
                lx, _, az = self._last_manual
        lx = max(-self.max_linear, min(self.max_linear, lx))
        az = max(-self.max_angular, min(self.max_angular, az))
        return lx, az

    def _send_motor_if_needed(self, _event):
        if self._serial is None or not self._serial.is_open:
            return
        lx, az = self._active_twist()
        left_rpm, right_rpm = twist_to_motor_rpm(lx, az, self.wheel_diameter, self.wheel_spacing)
        payload = pack_motor_command(left_rpm, right_rpm, 0.0)
        frame = build_frame(CM_DEV_CTL_TYPE, COMID_MOTOR_COMMUN, payload)
        try:
            self._serial.write(frame)
        except serial.SerialException as exc:
            rospy.logwarn_throttle(2.0, "串口写入失败: %s", exc)

    def _read_serial(self, _event):
        if self._serial is None or not self._serial.is_open:
            return
        try:
            nbytes = self._serial.in_waiting
            if nbytes <= 0:
                return
            data = self._serial.read(nbytes)
            if data:
                self._parser.feed(data)
        except serial.SerialException as exc:
            rospy.logwarn_throttle(2.0, "串口读取失败: %s", exc)

    def _on_frame(self, msgtype, msgid, payload):
        if msgtype != CM_DEV_STA_TYPE:
            return
        if msgid == STAID_MOTOR_DATAS:
            self._handle_motor(payload)
        elif msgid == STAID_IMU_DATAS:
            self._handle_imu(payload)
        elif msgid == STAID_BATTERY_DATAS:
            self._handle_battery(payload)

    def _handle_motor(self, payload):
        try:
            right_enc, left_enc = decode_motor_encoder(payload)
            v_l, v_r = encoder_to_wheel_speeds_mars(right_enc, left_enc, self.wheel_diameter)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, "轮速帧解析失败: %s", exc)
            return
        now = rospy.Time.now()
        now_sec = now.to_sec()
        dt = now_sec - self._last_odom_time
        self._last_odom_time = now_sec
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02
        x, y, th = self.odom.update_from_wheel_speeds(v_l, v_r, dt)
        self._publish_odometry(now, x, y, th, self.odom.vx, self.odom.omega)

    def _publish_odometry(self, stamp, x, y, theta, vx, omega):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(theta * 0.5)
        odom.pose.pose.orientation.w = math.cos(theta * 0.5)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = omega
        self._odom_pub.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation.z = odom.pose.pose.orientation.z
            transform.transform.rotation.w = odom.pose.pose.orientation.w
            self._tf.sendTransform(transform)

    def _handle_imu(self, payload):
        try:
            imu_raw = decode_imu(payload)
            wz = imu_omg_z_to_rad_s(imu_raw["omg_z"])
        except ValueError:
            return
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.base_frame
        msg.angular_velocity.z = wz
        self._imu_pub.publish(msg)

    def _handle_battery(self, payload):
        try:
            bat = decode_battery(payload)
        except ValueError:
            return
        msg = BatteryState()
        msg.header.stamp = rospy.Time.now()
        msg.percentage = bat["percentage"] / 100.0 if bat["percentage"] > 1.0 else bat["percentage"]
        msg.voltage = bat["voltage"]
        msg.current = bat["current"]
        msg.temperature = bat["temperature"]
        msg.present = True
        self._bat_pub.publish(msg)


def main():
    rospy.init_node("chassis_serial")
    ChassisSerialNode()
    rospy.spin()


if __name__ == "__main__":
    main()
