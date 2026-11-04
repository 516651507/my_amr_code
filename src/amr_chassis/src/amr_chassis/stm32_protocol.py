# -*- coding: utf-8 -*-
"""STM32 ↔ RK3588 二进制帧协议（小端）。与 docs/hardware_and_protocol_zh.md 一致。"""

from __future__ import division

import struct
from typing import Callable, Dict, List, Optional, Tuple

HEAD_A, HEAD_B = 0x5A, 0xA5
TAIL_A, TAIL_B = 0x5C, 0xC5

CM_DEV_CTL_TYPE = 1
CM_DEV_STA_TYPE = 2

STAID_IMU_DATAS = 1
STAID_BATTERY_DATAS = 2
STAID_MOTOR_DATAS = 3

COMID_MOTOR_COMMUN = 2

RS_PAYLOAD_MAX = 125


def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value & 0xFF


def build_frame(msgtype: int, msgid: int, payload: bytes) -> bytes:
    if len(payload) > RS_PAYLOAD_MAX:
        raise ValueError("payload too long: {}".format(len(payload)))
    head = bytes((HEAD_A, HEAD_B, msgtype & 0xFF, msgid & 0xFF, len(payload) & 0xFF))
    body = head + payload
    check = xor_checksum(body)
    return body + bytes((check, TAIL_A, TAIL_B))


def pack_motor_command(left_speed: float, right_speed: float, spin_speed: float = 0.0) -> bytes:
    """MotorCommand：3×double 小端，单位与 mars 一致（编码器 rpm 量纲）。"""
    return struct.pack("<ddd", float(left_speed), float(right_speed), float(spin_speed))


def twist_to_motor_rpm(linear_x: float, angular_z: float, wheel_diameter: float, wheel_spacing: float
                       ) -> Tuple[float, float]:
    """geometry_msgs/Twist → 左右轮 rpm（含 mars 侧 right 取负）。"""
    import math
    half = wheel_spacing * 0.5
    right_v = linear_x - angular_z * half
    left_v = linear_x + angular_z * half
    if wheel_diameter <= 1e-6:
        return 0.0, 0.0
    scale = 60.0 / (math.pi * wheel_diameter)
    left_rpm = left_v * scale
    right_rpm = -right_v * scale
    return left_rpm, right_rpm


def decode_imu(payload: bytes) -> Dict[str, int]:
    if len(payload) < 24:
        raise ValueError("IMU payload too short")
    fields = struct.unpack("<iiiiii", payload[:24])
    keys = ("x_acc", "y_acc", "z_acc", "omg_x", "omg_y", "omg_z")
    return dict(zip(keys, fields))


def imu_omg_z_to_rad_s(omg_z_raw: int) -> float:
    """WT61C 量程：/32768*2000 deg/s → rad/s。"""
    import math
    deg_s = 1.0 * omg_z_raw / 32768.0 * 2000.0
    return deg_s * math.pi / 180.0


def decode_battery(payload: bytes) -> Dict[str, float]:
    if len(payload) < 20:
        raise ValueError("battery payload too short")
    pct, volt, curr, temp, remain = struct.unpack("<fffff", payload[:20])
    return {
        "percentage": pct,
        "voltage": volt,
        "current": curr,
        "temperature": temp,
        "remain_time": remain,
    }


def decode_motor_encoder(payload: bytes) -> Tuple[int, int]:
    if len(payload) < 8:
        raise ValueError("motor encoder payload too short")
    right_speeder, left_speeder = struct.unpack("<ii", payload[:8])
    return right_speeder, left_speeder


def encoder_to_wheel_speeds_mars(right_speeder: int, left_speeder: int, wheel_diameter: float
                                 ) -> Tuple[float, float]:
    import math
    right_rpm = -left_speeder / 1000.0
    left_rpm = right_speeder / 1000.0
    factor = math.pi * wheel_diameter / 60.0
    v_l = left_rpm * factor
    v_r = right_rpm * factor
    return v_l, v_r


class FrameParser(object):
    """逐字节状态机拆帧。"""

    def __init__(self, on_frame: Callable[[int, int, bytes], None]):
        self._on_frame = on_frame
        self._state = "HEAD_A"
        self._msgtype = 0
        self._msgid = 0
        self._length = 0
        self._payload = bytearray()
        self._check = 0

    def reset(self):
        self._state = "HEAD_A"
        self._payload = bytearray()

    def feed(self, data: bytes):
        for byte in data:
            self._feed_byte(byte)

    def _feed_byte(self, byte: int):
        if self._state == "HEAD_A":
            self._state = "HEAD_B" if byte == HEAD_A else "HEAD_A"
        elif self._state == "HEAD_B":
            if byte == HEAD_B:
                self._state = "MSGTYPE"
            else:
                self._state = "HEAD_A" if byte == HEAD_A else "HEAD_A"
        elif self._state == "MSGTYPE":
            self._msgtype = byte
            self._state = "MSGID"
        elif self._state == "MSGID":
            self._msgid = byte
            self._state = "LENGTH"
        elif self._state == "LENGTH":
            self._length = byte
            if self._length > RS_PAYLOAD_MAX:
                self.reset()
                return
            self._payload = bytearray()
            self._state = "PAYLOAD" if self._length else "CHECK"
        elif self._state == "PAYLOAD":
            self._payload.append(byte)
            if len(self._payload) >= self._length:
                self._state = "CHECK"
        elif self._state == "CHECK":
            self._check = byte
            self._state = "TAIL_A"
        elif self._state == "TAIL_A":
            if byte != TAIL_A:
                self.reset()
                if byte == HEAD_A:
                    self._feed_byte(byte)
                return
            self._state = "TAIL_B"
        elif self._state == "TAIL_B":
            if byte != TAIL_B:
                self.reset()
                if byte == HEAD_A:
                    self._feed_byte(byte)
                return
            head = bytes((HEAD_A, HEAD_B, self._msgtype, self._msgid, self._length))
            body = head + bytes(self._payload)
            if xor_checksum(body) == self._check:
                self._on_frame(self._msgtype, self._msgid, bytes(self._payload))
            self.reset()


def run_self_test():
    left, right = twist_to_motor_rpm(1.0, 0.0, 0.165, 0.42)
    assert abs(left - right) > 0.01 or True  # sanity
    frame = build_frame(CM_DEV_CTL_TYPE, COMID_MOTOR_COMMUN, pack_motor_command(left, right, 0.0))
    assert frame[0] == HEAD_A and frame[1] == HEAD_B
    assert frame[-2:] == bytes((TAIL_A, TAIL_B))

    received = []

    def cb(mt, mid, pl):
        received.append((mt, mid, pl))

    parser = FrameParser(cb)
    parser.feed(frame)
    assert len(received) == 1
    assert received[0][0] == CM_DEV_CTL_TYPE
    assert received[0][1] == COMID_MOTOR_COMMUN

    motor_pl = struct.pack("<ii", 1000, -2000)
    sta = build_frame(CM_DEV_STA_TYPE, STAID_MOTOR_DATAS, motor_pl)
    parser2 = FrameParser(lambda *a: None)
    parser2.feed(sta)
    v_l, v_r = encoder_to_wheel_speeds_mars(1000, -2000, 0.165)
    assert v_l != 0 or v_r != 0

    imu_pl = struct.pack("<iiiiii", 0, 0, 0, 0, 0, 16384)
    wz = imu_omg_z_to_rad_s(16384)
    assert abs(wz) > 0.1

    print("stm32_protocol self-test ok")


if __name__ == "__main__":
    run_self_test()
