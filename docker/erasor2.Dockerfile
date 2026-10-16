# SPDX-License-Identifier: MPL-2.0
# Copyright (c) FineNav
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# ERASOR2 自包含镜像
# 包含 C++ 二进制 + Python 聚类脚本
FROM stevenmhy/erasor2:ubuntu22

RUN mkdir -p /opt/erasor2/bin /opt/erasor2/scripts

COPY slam_toolbox/bin/erasor2/mapgen /opt/erasor2/bin/
COPY slam_toolbox/bin/erasor2/run_erasor2 /opt/erasor2/bin/
COPY slam_toolbox/erasor2_scripts/kitti_clustering.py /opt/erasor2/scripts/
COPY slam_toolbox/erasor2_scripts/pcd_preprocess.py /opt/erasor2/scripts/

RUN chmod +x /opt/erasor2/bin/*

WORKDIR /opt/erasor2
