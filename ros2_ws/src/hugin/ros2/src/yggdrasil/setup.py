# Copyright (c) Sensrad 2025
"""Setup for Sensrad's ROS 2 runner package"""

import glob
import os

from setuptools import setup

PACKAGE_NAME = "yggdrasil"

# Install *all* YAML and RViz files
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
    (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
    (os.path.join("share", PACKAGE_NAME, "launch"), glob.glob("launch/*.py")),
    (os.path.join("share", PACKAGE_NAME, "launch", "profiles"), glob.glob("launch/profiles/*.py")),
    (os.path.join("share", PACKAGE_NAME, "config"), glob.glob("config/*.yaml")),
    (os.path.join("share", PACKAGE_NAME, "extrinsics"), glob.glob("extrinsics/*.yaml")),
    (os.path.join("share", PACKAGE_NAME, "rviz"), glob.glob("rviz/*.rviz")),
]

setup(
    name=PACKAGE_NAME,
    version="1.0.0",
    packages=[PACKAGE_NAME],  # empty __init__.py keeps setuptools happy
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Per Ception",
    maintainer_email="do_not_use@sensrad.com",
    description="ROS2 launch files for Sensrad platforms",
    license="See LICENSE file",
    tests_require=["pytest"],
    entry_points={},  # no console-scripts needed here
)
