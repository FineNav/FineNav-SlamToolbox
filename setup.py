# SPDX-License-Identifier: MPL-2.0
# Copyright (c) FineNav
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from setuptools import setup, find_packages

setup(
    name="slam_toolbox",
    version="0.2.8",
    packages=find_packages(),
    install_requires=[
        "questionary>=2.0.0",
        "imageio>=2.31.0",
        "matplotlib>=3.5.0",
        "numpy>=1.20.0",
        "open3d>=0.15.0",
        "psutil>=5.9.0",
        "pyyaml>=6.0",
        "rich>=12.0.0",
        "scipy>=1.7.0",
    ],
    entry_points={
        "console_scripts": [
            "slam_toolbox=slam_toolbox.cli:main",
        ],
    },
    author="FineNav",
    license="MPL-2.0",
    description="A Python CLI tool for interactive SLAM map processing.",
)
