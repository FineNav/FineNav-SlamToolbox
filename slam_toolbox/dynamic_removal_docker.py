# SPDX-License-Identifier: MPL-2.0
# Copyright (c) FineNav
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""Docker image helpers for dynamic-removal methods."""

import subprocess

def _ensure_or_pull_image(image, fallback=None):
    """检查 Docker 镜像是否存在，否则拉取。返回实际的 image tag。"""
    local = subprocess.run(
        ["docker", "image", "inspect", image],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    if local.returncode == 0:
        print(f"本地已有镜像: {image}")
        return image

    if fallback:
        local2 = subprocess.run(
            ["docker", "image", "inspect", fallback],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        if local2.returncode == 0:
            print(f"使用本地镜像: {fallback}")
            return fallback

    print(f"本地未找到镜像，正在从 Docker Hub 拉取 {image}...")
    subprocess.run(["docker", "pull", image], check=True)
    return image
