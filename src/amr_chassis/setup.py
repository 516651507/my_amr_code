#!/usr/bin/env python3
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup(generate_distutils_setup(
    packages=["amr_chassis"],
    package_dir={"": "src"},
    version="0.1.0",
))
