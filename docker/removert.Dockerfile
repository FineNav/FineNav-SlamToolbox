# SPDX-License-Identifier: MPL-2.0
# Copyright (c) FineNav
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Removert 自包含镜像
# 包含预编译的 catkin workspace
FROM osrf/ros:noetic-desktop-full

# 复制 workspace 源码并编译
COPY slam_toolbox/removert/src /opt/removert_ws/src

WORKDIR /opt/removert_ws

RUN . /opt/ros/noetic/setup.sh && \
    catkin_make -j$(nproc) && \
    echo "source /opt/removert_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /opt/removert_ws
