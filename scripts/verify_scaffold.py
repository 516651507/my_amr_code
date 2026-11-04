#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""不依赖 ROS 的脚手架检查：XML、YAML、Python 语法、融合与平滑的纯逻辑。"""

import ast
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:
    yaml = None

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def rel(path):
    return os.path.relpath(path, ROOT)


def walk_ext(exts):
    found = []
    for dirpath, dirnames, filenames in os.walk(os.path.join(ROOT, "src")):
        dirnames[:] = [name for name in dirnames if name not in ("build", "devel")]
        for name in filenames:
            if name.endswith(exts):
                found.append(os.path.join(dirpath, name))
    return found


def main():
    errors = []

    for path in walk_ext((".xml", ".launch")):
        try:
            ET.parse(path)
        except ET.ParseError as exc:
            errors.append("{}: {}".format(rel(path), exc))

    if yaml is None:
        errors.append("缺少 PyYAML，无法检查 yaml")
    else:
        for path in walk_ext((".yaml", ".yml")):
            try:
                with open(path, encoding="utf-8") as handle:
                    yaml.safe_load(handle.read())
            except yaml.YAMLError as exc:
                errors.append("{}: {}".format(rel(path), exc))

    for path in walk_ext(".py"):
        try:
            with open(path, encoding="utf-8") as handle:
                ast.parse(handle.read(), filename=path)
        except SyntaxError as exc:
            errors.append("{}: {}".format(rel(path), exc))

    py_tests = [
        os.path.join(ROOT, "src/amr_mapping/scripts/mapping_assist.py"),
        os.path.join(ROOT, "src/amr_navigation/scripts/cmd_smoother.py"),
    ]
    proto_test = os.path.join(ROOT, "src/amr_chassis/src/amr_chassis/stm32_protocol.py")
    if os.path.isfile(proto_test):
        result = subprocess.run([sys.executable, proto_test], capture_output=True, text=True)
        if result.returncode != 0:
            errors.append("stm32_protocol: {}".format(result.stderr or result.stdout))
        else:
            print(result.stdout.strip())
    for path in py_tests:
        result = subprocess.run([sys.executable, path, "--self-test"], capture_output=True, text=True)
        if result.returncode != 0:
            errors.append("{} self-test failed: {}".format(rel(path), result.stderr or result.stdout))
        else:
            print(result.stdout.strip())

    binary = "/tmp/amr_test_scan_fusion"
    compile_cmd = [
        "g++", "-std=c++14", "-Wall", "-Wextra", "-Werror",
        "-I", os.path.join(ROOT, "src/amr_obstacle_fusion/include"),
        os.path.join(ROOT, "src/amr_obstacle_fusion/test/test_scan_fusion.cpp"),
        "-o", binary,
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    if compiled.returncode != 0:
        errors.append("scan_fusion compile failed:\n" + compiled.stderr)
    else:
        ran = subprocess.run([binary], capture_output=True, text=True)
        if ran.returncode != 0:
            errors.append("scan_fusion test failed:\n" + ran.stderr + ran.stdout)
        else:
            print(ran.stdout.strip())

    required = [
        "/lidar1/scan_filtered",
        "/wheel_odom",
        "/autoVel_cmd",
        "/camera/depth/points_cam",
        "/camera2/depth/points_cam",
        "base_link",
        "laser_link",
        "gmapping",
        "teb_local_planner/TebLocalPlannerROS",
        "global_planner/GlobalPlanner",
        "max_vel_x: 1.5",
        "LDS-50C-E",
        "Dabai_DW2",
        "bluesea2",
        "wheel_diameter: 0.165",
        "wheel_spacing: 0.42",
        "/dev/ttyS4",
        "192.168.158.98",
        "amr_chassis",
        "chassis_serial",
        "scan_filter",
        "/api/debug/config/camera",
        "/debug",
    ]
    blob = []
    for path in walk_ext((".launch", ".yaml", ".yml", ".py", ".xacro", ".cpp", ".h", ".hpp", ".html")):
        with open(path, encoding="utf-8", errors="replace") as handle:
            blob.append(handle.read())
    text = "\n".join(blob)

    extra = [
        os.path.join(ROOT, "src/amr_chassis/src/amr_chassis/diff_odometry.py"),
        os.path.join(ROOT, "src/amr_web/scripts/config_store.py"),
    ]
    for path in extra:
        if os.path.isfile(path):
            result = subprocess.run([sys.executable, path], capture_output=True, text=True)
            if result.returncode != 0:
                errors.append("{}: {}".format(rel(path), result.stderr or result.stdout))
            else:
                print(result.stdout.strip())

    web_verify = subprocess.run([sys.executable, os.path.join(ROOT, "scripts/verify_web_api.py")],
                                capture_output=True, text=True)
    if web_verify.returncode != 0:
        errors.append("verify_web_api:\n" + (web_verify.stderr or web_verify.stdout))
    else:
        print(web_verify.stdout.strip())

    for token in required:
        if token not in text:
            errors.append("仓库里找不到约定字段: {}".format(token))

    if errors:
        print("FAILED")
        for item in errors:
            print(item)
        return 1
    print("scaffold verify ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
