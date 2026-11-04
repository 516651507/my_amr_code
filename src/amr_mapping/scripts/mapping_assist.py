#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订阅 /map，按 map_server 的 PGM/YAML 规则把栅格写到磁盘。

这是建图流程上的薄封装，不替换 gmapping。
"""

import os
import re

SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")


def color_for_cell(value, occupied_at=65, free_at=25):
    """与 map_saver 一致：占据黑、空闲白、未知灰。"""
    if value < 0:
        return 205
    if value >= occupied_at:
        return 0
    if value <= free_at:
        return 254
    return 205


def write_occupancy_map(width, height, resolution, origin_x, origin_y, data,
                        pgm_path, yaml_path, image_name=None):
    if width <= 0 or height <= 0:
        raise ValueError("地图宽高无效")
    if len(data) != width * height:
        raise ValueError("栅格长度与宽高不符: {} != {}x{}".format(len(data), width, height))

    image_name = image_name or os.path.basename(pgm_path)
    pixels = bytearray()
    for row in range(height):
        y = height - 1 - row
        for x in range(width):
            pixels.append(color_for_cell(int(data[x + y * width])))

    directory = os.path.dirname(os.path.abspath(pgm_path))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)

    tmp_pgm = pgm_path + ".tmp"
    with open(tmp_pgm, "wb") as handle:
        header = "P5\n{} {}\n255\n".format(width, height).encode("ascii")
        handle.write(header)
        handle.write(pixels)
    os.replace(tmp_pgm, pgm_path)

    yaml_text = (
        "image: {image}\n"
        "resolution: {res:.6f}\n"
        "origin: [{ox:.6f}, {oy:.6f}, 0.000000]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    ).format(image=image_name, res=float(resolution), ox=float(origin_x), oy=float(origin_y))
    tmp_yaml = yaml_path + ".tmp"
    with open(tmp_yaml, "w", encoding="ascii") as handle:
        handle.write(yaml_text)
    os.replace(tmp_yaml, yaml_path)
    return pgm_path, yaml_path


def run_self_test():
    import tempfile
    width, height = 2, 2
    # 行优先，y 从下往上：(0,0) 空闲，(1,0) 占据，(0,1) 未知，(1,1) 空闲
    data = [0, 100, -1, 0]
    tmp = tempfile.mkdtemp(prefix="amr_map_test_")
    pgm = os.path.join(tmp, "t.pgm")
    yaml_path = os.path.join(tmp, "t.yaml")
    write_occupancy_map(width, height, 0.05, -1.0, -2.0, data, pgm, yaml_path)
    raw = open(pgm, "rb").read()
    assert raw.startswith(b"P5\n2 2\n255\n"), raw[:16]
    body = raw.split(b"\n", 3)[-1]
    # PGM 第一行是地图 y=1：未知=205，空闲=254；第二行是 y=0：空闲=254，占据=0
    assert list(body) == [205, 254, 254, 0], list(body)
    text = open(yaml_path, encoding="ascii").read()
    assert "resolution: 0.050000" in text
    assert "origin: [-1.000000, -2.000000, 0.000000]" in text
    assert color_for_cell(-1) == 205
    assert color_for_cell(0) == 254
    assert color_for_cell(100) == 0
    print("mapping_assist self-test ok")


def _ros_main():
    import rospy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse

    class MappingAssist(object):
        def __init__(self):
            self.save_dir = rospy.get_param("~save_dir", os.path.expanduser("~/amr_maps"))
            self.map_name = rospy.get_param("~map_name", "amr_map")
            self.latest = None
            self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
            topic = rospy.get_param("~map_topic", "/map")
            rospy.Subscriber(topic, OccupancyGrid, self._on_map, queue_size=1)
            rospy.Service("~save_map", Trigger, self._save)
            rospy.loginfo("mapping_assist 监听 %s，保存目录 %s", topic, self.save_dir)

        def _on_map(self, msg):
            self.latest = msg
            info = msg.info
            self.status_pub.publish(String(data="{}x{} res={:.3f}".format(
                info.width, info.height, info.resolution)))

        def _save(self, _req):
            if self.latest is None:
                return TriggerResponse(success=False, message="还没有收到 /map")
            name = rospy.get_param("~map_name", self.map_name)
            if not SAFE_NAME.match(name):
                return TriggerResponse(success=False, message="地图名只允许字母数字、下划线和中划线")
            info = self.latest.info
            pgm = os.path.join(self.save_dir, name + ".pgm")
            yaml_path = os.path.join(self.save_dir, name + ".yaml")
            try:
                write_occupancy_map(
                    int(info.width), int(info.height), info.resolution,
                    info.origin.position.x, info.origin.position.y,
                    list(self.latest.data), pgm, yaml_path, image_name=name + ".pgm")
            except (OSError, ValueError) as exc:
                return TriggerResponse(success=False, message=str(exc))
            message = "已保存 {} 与 {}".format(yaml_path, pgm)
            rospy.loginfo(message)
            return TriggerResponse(success=True, message=message)

    rospy.init_node("mapping_assist")
    MappingAssist()
    rospy.spin()


def main():
    import sys
    if "--self-test" in sys.argv:
        run_self_test()
        return
    _ros_main()


if __name__ == "__main__":
    main()
