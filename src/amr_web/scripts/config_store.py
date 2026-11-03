# -*- coding: utf-8 -*-
"""读写调试配置 YAML（DW2 USB 等）。保存后需重启对应 ROS 节点才生效。"""

from __future__ import division

import os
import shutil
import tempfile

try:
    import yaml
except ImportError:
    yaml = None


def _require_yaml():
    if yaml is None:
        raise RuntimeError("需要 PyYAML")


def load_yaml(path):
    _require_yaml()
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle.read()) or {}


def save_yaml(path, data):
    _require_yaml()
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    text = yaml.safe_dump(data, allow_unicode=True, default_flow_style=False, sort_keys=False)
    fd, tmp = tempfile.mkstemp(dir=directory or ".", prefix=".cfg_", suffix=".yaml")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
        shutil.copy2(path, path + ".bak") if os.path.isfile(path) else None
        os.replace(tmp, path)
    except Exception:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def merge_camera_config(existing, patch):
    """只合并 camera1/camera2 下允许 Web 修改的字段。"""
    allowed = ("usb_port", "serial_number", "depth_width", "depth_height", "depth_fps")
    out = dict(existing)
    for cam_key in ("camera1", "camera2"):
        if cam_key not in patch:
            continue
        block = dict(out.get(cam_key) or {})
        for field in allowed:
            if field in patch[cam_key]:
                block[field] = patch[cam_key][field]
        out[cam_key] = block
    for key in ("enable_point_cloud", "enable_depth"):
        if key in patch:
            out[key] = patch[key]
    return out


def list_usb_sysfs_hints():
    """轻量列举 USB 拓扑路径（供调试页选择），不依赖 lsusb。"""
    root = "/sys/bus/usb/devices"
    hints = []
    if not os.path.isdir(root):
        return hints
    for name in sorted(os.listdir(root)):
        if ":" in name:
            continue
        path = os.path.join(root, name)
        product = os.path.join(path, "product")
        manufacturer = os.path.join(path, "manufacturer")
        label = name
        try:
            if os.path.isfile(product):
                with open(product, encoding="utf-8", errors="replace") as handle:
                    label = "{} ({})".format(name, handle.read().strip())
        except OSError:
            pass
        hints.append({"usb_port": name, "label": label})
    return hints
