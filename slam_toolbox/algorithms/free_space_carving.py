#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (c) FineNav
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""Free-space carving 动态障碍物清除（离线 KITTI 逐扫描版本）。

算法移植自 mid360_localization/launch/localization.py 的 ``VoxelHitMap``
（逐帧整数命中图 + 对称 hit/miss 射线清除），把 ROS2 在线流程改成对
``velodyne/*.bin`` + ``poses_odom_base.txt`` 的离线回放：

  1. 每帧用真实位姿把局部点云变换到世界系，按水平 XY 距离门限
     ``map_max_range`` 过滤；
  2. ``insert`` 对命中体素 +1（封顶 ``hit_cap``），拿到本帧命中集；
  3. 每 ``interval`` 帧、且位姿有效时，从传感器原点向观测点做
     ``carve``：途经已有体素 -1，减到 0 即删除；射线长度受
     ``carve_max_range`` 限制，终点体素与本帧命中集受保护；
  4. 全部帧处理完后，按最终命中图把降采样后的全量点拆成
     static（hits_of > 0，失败点用 3x3x3 邻域兜底）与 dynamic。

与 raycast_voxel_cleanup.py 的 Octree + log-odds 不同：这里每体素每帧
严格 ±1、证据有界，动态障碍约 ``hit_cap`` 帧内必然消散，且不会因
远处的历史累计而迟滞。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np


# ---------------------------------------------------------------------------
# VoxelHitMap：移植自 mid360_localization/launch/localization.py L35-295
# 纯 numpy，与 ROS 无关。key 位段：ix(21bit) | iy(21bit) | iz(21bit)，
# 坐标偏移 2^20 支持负值；只存被观测过的体素。
# ---------------------------------------------------------------------------
class VoxelHitMap:
    """OctoMap 风格的 3D 体素命中图。

    每帧对命中体素 +1（每体素每帧最多一次），射线清除做对称的 -1；
    命中计数封顶 ``hit_cap``，整体等价 clamped log-odds，动态障碍不会
    积累出需要等量帧数才能清除的巨额证据。
    """

    _AXIS_BITS = 21
    _OFFSET = 1 << 20
    _MASK = (1 << 21) - 1

    def __init__(self, voxel_size):
        self.voxel_size = float(voxel_size)
        self._keys = np.zeros(0, dtype=np.int64)   # 体素 key，严格升序
        self._vals = np.zeros(0, dtype=np.int32)   # 命中计数（与 _keys 对齐）

    def clear(self):
        self._keys = np.zeros(0, dtype=np.int64)
        self._vals = np.zeros(0, dtype=np.int32)

    def __len__(self):
        return int(self._keys.size)

    def get(self, key):
        """查询单个 key 的命中数（不存在返回 None）。"""
        i = int(np.searchsorted(self._keys, np.int64(key)))
        if i < self._keys.size and self._keys[i] == key:
            return int(self._vals[i])
        return None

    def keys_of(self, pts):
        """点云 (N,3) -> packed key 数组；越界点返回 -1。"""
        vs = self.voxel_size
        off = self._OFFSET
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        ix = np.floor(p[:, 0] / vs).astype(np.int64) + off
        iy = np.floor(p[:, 1] / vs).astype(np.int64) + off
        iz = np.floor(p[:, 2] / vs).astype(np.int64) + off
        key = (ix << 42) | (iy << 21) | iz
        bad = ((ix < 0) | (ix > self._MASK) |
               (iy < 0) | (iy > self._MASK) |
               (iz < 0) | (iz > self._MASK))
        if np.any(bad):
            key[bad] = -1
        return key

    def hits_of(self, keys):
        """key 数组 -> 命中帧数；不在图中（含 -1）返回 0。"""
        keys = np.asarray(keys, dtype=np.int64).reshape(-1)
        out = np.zeros(keys.size, dtype=np.int32)
        if keys.size == 0 or self._keys.size == 0:
            return out
        pos = np.searchsorted(self._keys, keys)
        in_range = pos < self._keys.size
        idx = np.where(in_range, pos, 0)
        found = in_range & (self._keys[idx] == keys)
        out[found] = self._vals[idx[found]]
        return out

    def neighbor_hits_max(self, keys):
        """key 数组 -> 3x3x3 邻域（含自身）内的最大命中帧数。

        给 PCD 裁剪兜底：降采样质心可能落进相邻的未命中体素，只查邻域
        最大值，避免因这种错位把真实障碍误删。
        """
        keys = np.asarray(keys, dtype=np.int64).reshape(-1)
        out = np.zeros(keys.size, dtype=np.int32)
        live = keys >= 0
        if not np.any(live) or self._keys.size == 0:
            return out
        kv = keys[live]
        best = np.zeros(kv.size, dtype=np.int32)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    delta = dx * (1 << 42) + dy * (1 << 21) + dz
                    np.maximum(best, self.hits_of(kv + delta), out=best)
        out[live] = best
        return out

    def insert(self, pts, hit_cap=255):
        """插入一帧点云，返回本帧被命中的体素 key（去重升序）。

        同一体素在一帧内无论命中多少点只 +1，命中计数封顶 hit_cap。
        """
        if pts is None or len(pts) == 0:
            return np.zeros(0, dtype=np.int64)
        key = self.keys_of(pts)
        key = key[key >= 0]
        if key.size == 0:
            return np.zeros(0, dtype=np.int64)
        ukey = np.unique(key)
        cap = max(1, int(hit_cap))

        n = self._keys.size
        if n == 0:
            self._keys = ukey
            self._vals = np.ones(ukey.size, dtype=np.int32)
            return ukey

        pos = np.searchsorted(self._keys, ukey)
        found = self._keys[np.minimum(pos, n - 1)] == ukey
        if np.any(found):
            fp = pos[found]
            self._vals[fp] = np.minimum(self._vals[fp] + 1, cap)
        new = ukey[~found]
        if new.size:
            npos = pos[~found]
            self._keys = np.insert(self._keys, npos, new)
            self._vals = np.insert(self._vals, npos, 1)
        return ukey

    def carve(self, origin, pts, max_range, protect_keys=None,
              max_rays=None, max_steps=512):
        """射线清除：沿「传感器原点 → 观测点」把途经体素 -1（归零即删除）。

        与 insert 的 +1 严格对称：同一格同一帧最多 ±1。安全规则：终点
        体素永不被清、protect_keys（本帧命中集）跳过、超过 max_range 的
        射线只清到 max_range 处、只对图中已有体素减计数。
        """
        if pts is None or len(pts) == 0 or self._keys.size == 0:
            return 0
        vs = self.voxel_size
        off = self._OFFSET
        o = np.asarray(origin, dtype=np.float64).reshape(3)
        p = np.asarray(pts, dtype=np.float64)
        if p.ndim != 2 or p.shape[1] != 3:
            return 0

        # 射线抽稀（等距、确定性，便于复现）
        n = p.shape[0]
        if max_rays is not None and 0 < int(max_rays) < n:
            stride = int(np.ceil(n / float(max_rays)))
            p = p[::stride]
        d = p - o
        dist = np.linalg.norm(d, axis=1)

        eff = np.minimum(dist, float(max_range))
        k = np.ceil(eff / (vs * 0.5)).astype(np.int64)
        np.minimum(k, int(max_steps), out=k)
        ok = k >= 2
        if not np.any(ok):
            return 0
        p, d, dist, eff, k = p[ok], d[ok], dist[ok], eff[ok], k[ok]
        K = int(k.max())

        # 中点采样：射线 i 的第 j 段中点在 eff_i * (j+0.5)/k_i 处
        j = np.arange(K, dtype=np.float64)
        s = ((j[None, :] + 0.5) / k[:, None]) * eff[:, None]      # (N,K)
        u = d / np.maximum(dist, 1e-12)[:, None]

        ex = np.floor(p[:, 0] / vs).astype(np.int64) + off
        ey = np.floor(p[:, 1] / vs).astype(np.int64) + off
        ez = np.floor(p[:, 2] / vs).astype(np.int64) + off

        valid = j[None, :] < k[:, None]

        key = None
        eq_end = None
        tmp = np.empty((p.shape[0], K), dtype=np.float64)
        for a, shift, ea in ((0, 42, ex), (1, 21, ey), (2, 0, ez)):
            np.multiply(u[:, a, None], s, out=tmp)
            tmp += o[a]
            tmp /= vs
            np.floor(tmp, out=tmp)
            ia = tmp.astype(np.int64) + off
            if ia.min() < 0 or ia.max() > self._MASK:
                valid &= (ia >= 0) & (ia <= self._MASK)
            eqa = ia == ea[:, None]
            eq_end = eqa if eq_end is None else (eq_end & eqa)
            key = (ia << shift) if key is None else (key | (ia << shift))
        valid &= ~eq_end

        keys = key[valid]
        if keys.size == 0:
            return 0

        # 只对图中已存在的体素减计数，先按图筛选再去重
        pos = np.searchsorted(self._keys, keys)
        found = self._keys[np.minimum(pos, self._keys.size - 1)] == keys
        if not np.any(found):
            return 0
        free = np.unique(keys[found])

        # 命中优先：本帧被命中的体素不减
        if protect_keys is not None and len(protect_keys) > 0:
            free = free[~np.isin(free, np.asarray(protect_keys, dtype=np.int64))]
            if free.size == 0:
                return 0

        fp = np.searchsorted(self._keys, free)
        self._vals[fp] -= 1
        cleared = int(fp.size)
        dead = self._vals[fp] == 0
        if np.any(dead):
            keep = np.ones(self._keys.size, dtype=bool)
            keep[fp[dead]] = False
            self._keys = self._keys[keep]
            self._vals = self._vals[keep]
        return cleared

    def arrays(self):
        """返回 (ix, iy, iz, hits)：体素整数坐标与命中帧数。"""
        ks = self._keys
        hs = self._vals
        if ks.size == 0:
            return (np.zeros(0, dtype=np.int64),
                    np.zeros(0, dtype=np.int64),
                    np.zeros(0, dtype=np.int64),
                    np.zeros(0, dtype=np.int32))
        iz = (ks & self._MASK).astype(np.int64) - self._OFFSET
        iy = ((ks >> 21) & self._MASK).astype(np.int64) - self._OFFSET
        ix = ((ks >> 42) & self._MASK).astype(np.int64) - self._OFFSET
        return ix, iy, iz, hs


# ---------------------------------------------------------------------------
# 数据读写（写法对齐 raycast_voxel_cleanup.py，保持脚本自包含）
# ---------------------------------------------------------------------------
def load_poses(path):
    """每行 12 float（3x4 行主序）或 16 float（4x4）的 odom->base 位姿。"""
    poses = []
    for line_no, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip():
            continue
        vals = [float(x) for x in line.split()]
        if len(vals) == 12:
            mat = np.eye(4, dtype=np.float64)
            mat[:3, :4] = np.asarray(vals, dtype=np.float64).reshape(3, 4)
        elif len(vals) == 16:
            mat = np.asarray(vals, dtype=np.float64).reshape(4, 4)
        else:
            raise ValueError(f"unsupported pose line {line_no}: {len(vals)} values in {path}")
        poses.append(mat)
    return poses


def read_scan(path):
    scan = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    return scan[np.isfinite(scan).all(axis=1)]


def transform_scan(scan, pose, map_max_range):
    """局部帧 xyzi -> 世界帧 xyzi，按水平 XY 距离门限过滤。

    对齐源 L1578-1585：门限是水平 xy 距离而非 3D 欧氏距离，且全高度
    的点都要进图（地面点也要，源注释 L1632）。
    """
    if scan.size == 0:
        return scan
    if map_max_range > 0:
        dist2 = scan[:, 0].astype(np.float64) ** 2 + scan[:, 1].astype(np.float64) ** 2
        scan = scan[dist2 <= map_max_range * map_max_range]
    if scan.size == 0:
        return scan
    world = np.empty_like(scan, dtype=np.float32)
    world[:, :3] = scan[:, :3].astype(np.float64) @ pose[:3, :3].T + pose[:3, 3]
    world[:, 3] = scan[:, 3]
    return world


def sensor_origin(pose, offset):
    """世界系传感器原点 = 位姿平移 + R @ LiDAR 安装偏移（源 L1474）。"""
    return pose[:3, 3] + pose[:3, :3] @ np.asarray(offset, dtype=np.float64).reshape(3)


def voxel_downsample(points, vmap):
    """按 vmap 体素网格取质心（对齐源 merge_buffer_points 的 voxel_down_sample）。"""
    keys = vmap.keys_of(points[:, :3])
    valid = keys >= 0
    if not valid.all():
        points = points[valid]
        keys = keys[valid]
    if keys.size == 0:
        return np.zeros((0, 4), dtype=np.float32), np.zeros(0, dtype=np.int64)
    order = np.argsort(keys)
    sorted_keys = keys[order]
    sorted_pts = points[order][:, :4].astype(np.float64)
    uniq, start, counts = np.unique(sorted_keys, return_index=True, return_counts=True)
    sums = np.add.reduceat(sorted_pts, start, axis=0)
    centroids = (sums / counts[:, None]).astype(np.float32)
    return centroids, uniq


def write_pcd_from_points(points, out_path):
    """写 binary PCD（FIELDS x y z intensity），与 extractor._read_pcd 兼容。"""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    points = points.astype(np.float32, copy=False).reshape(-1, 4)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {points.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points.shape[0]}\n"
        "DATA binary\n"
    ).encode("ascii")
    with out_path.open("wb") as f:
        f.write(header)
        points.tofile(f)


def prune_points(vmap, points, keys=None):
    """按体素证据拆 (static, dynamic)，移植自源 _prune_cleared_pcd_points。"""
    if keys is None:
        keys = vmap.keys_of(points[:, :3])
    keep = vmap.hits_of(keys) > 0
    if not keep.all():
        failed = ~keep
        keep[failed] = vmap.neighbor_hits_max(keys[failed]) > 0
    return points[keep], points[~keep]


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Free-space carving (VoxelHitMap) for converted KITTI-style lidar sequences."
    )
    p.add_argument("--dataset", required=True, type=Path)
    p.add_argument("--seq", default="00")
    p.add_argument("--pose", type=str, default=None, help="Default: dataset/sequences/<seq>/poses_odom_base.txt")
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--start", type=int, default=0)
    p.add_argument("--end", type=int, default=None)
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--voxel-size", type=float, default=0.05)
    p.add_argument("--map-max-range", type=float, default=30.0,
                   help="插入门限：小车局部 XY 水平距离，更远的点不进图")
    p.add_argument("--carve-max-range", type=float, default=8.0,
                   help="射线清除最大长度，更远处不做任何假设")
    p.add_argument("--hit-cap", type=int, default=8)
    p.add_argument("--max-rays", type=int, default=800)
    p.add_argument("--interval", type=int, default=1, help="每 N 帧做一次射线清除")
    p.add_argument("--ray-origin-offset", type=str, default="0,0,0",
                   help="LiDAR 在 base_link 中的安装位置 x,y,z（米）")
    p.add_argument("--ray-clearing-enabled", action="store_true",
                   help="开启对称 hit/miss 射线清除；关闭则退化为纯命中累计")
    # 仅用于统一参数关系校验与 GIF 的 Z 范围，本算法全高度进图、不做 Z 过滤
    p.add_argument("--local-z-min", type=float, default=-5.0)
    p.add_argument("--local-z-max", type=float, default=5.0)
    p.add_argument("--write-before", action="store_true")
    p.add_argument("--save-dynamic-frames", action="store_true")
    p.add_argument("--progress-interval", type=int, default=25)
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    seq_dir = args.dataset / "dataset" / "sequences" / args.seq
    scan_paths = sorted((seq_dir / "velodyne").glob("*.bin"))
    pose_path = Path(args.pose) if args.pose else seq_dir / "poses_odom_base.txt"
    if not scan_paths:
        print(f"[error] 没有找到逐帧点云: {seq_dir / 'velodyne'}", flush=True)
        return 1
    if not pose_path.exists():
        print(f"[error] 没有找到位姿文件: {pose_path}", flush=True)
        return 1

    poses = load_poses(pose_path)
    last = len(scan_paths) - 1
    if args.end is not None:
        last = min(args.end, last)
    frame_ids = list(range(args.start, last + 1, max(1, args.stride)))
    if not frame_ids:
        print("[error] 没有可处理的帧", flush=True)
        return 1

    try:
        offset = [float(v) for v in args.ray_origin_offset.split(",")]
        if len(offset) != 3:
            raise ValueError
    except ValueError:
        print("[error] --ray-origin-offset 必须是 3 个数字 x,y,z", flush=True)
        return 1

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    dynamic_frames_dir = out_dir / "dynamic_frames"
    if args.save_dynamic_frames:
        dynamic_frames_dir.mkdir(parents=True, exist_ok=True)

    vmap = VoxelHitMap(args.voxel_size)
    cap = args.hit_cap if args.ray_clearing_enabled else 255
    before_chunks = []
    carved_frames = 0
    cleared_voxels = 0
    skipped_no_origin = 0

    print(
        f"[info] 射线清除: {len(frame_ids)} 帧, voxel={args.voxel_size}, "
        f"map_max_range={args.map_max_range}, carve_max_range={args.carve_max_range}, "
        f"hit_cap={cap}, max_rays={args.max_rays}, interval={args.interval}, "
        f"ray_clearing={args.ray_clearing_enabled}",
        flush=True,
    )

    for i, frame_id in enumerate(frame_ids):
        if frame_id >= len(poses):
            print(f"[warn] 帧 {frame_id} 缺少位姿，跳过", flush=True)
            continue
        pose = poses[frame_id]
        world = transform_scan(read_scan(scan_paths[frame_id]), pose, args.map_max_range)
        if world.size == 0:
            continue

        hit_keys = vmap.insert(world[:, :3], hit_cap=cap)

        # 源 L1637：关闭射线清除时命中证据封顶 255（纯命中累计）
        if args.ray_clearing_enabled and (i + 1) % max(1, args.interval) == 0:
            if np.isfinite(pose).all():
                origin = sensor_origin(pose, offset)
                cleared_voxels += vmap.carve(
                    origin, world[:, :3],
                    max_range=args.carve_max_range,
                    protect_keys=hit_keys,
                    max_rays=args.max_rays,
                )
                carved_frames += 1
            else:
                # 取不到可靠位姿宁可不清：瞎猜原点的射线会误清真障碍
                skipped_no_origin += 1

        # 帧内先降采样，避免全量逐帧拼接撑爆内存；最后再全局合并一次
        downsampled, _ = voxel_downsample(world, vmap)
        if downsampled.size:
            before_chunks.append(downsampled)

        if args.progress_interval > 0 and (i + 1) % args.progress_interval == 0:
            print(f"[info] 已处理 {i + 1}/{len(frame_ids)} 帧, 存活体素 {len(vmap)}", flush=True)

    if not before_chunks:
        print("[error] 没有任何有效点进入命中图", flush=True)
        return 1

    before = np.concatenate(before_chunks, axis=0)
    before, before_keys = voxel_downsample(before, vmap)
    static, dynamic = prune_points(vmap, before, before_keys)

    if args.save_dynamic_frames:
        for frame_id in frame_ids:
            if frame_id >= len(poses):
                continue
            world = transform_scan(read_scan(scan_paths[frame_id]), poses[frame_id], args.map_max_range)
            if world.size == 0:
                write_pcd_from_points(np.zeros((0, 4), dtype=np.float32),
                                      dynamic_frames_dir / f"{frame_id:06d}.pcd")
                continue
            _, removed = prune_points(vmap, world)
            write_pcd_from_points(removed, dynamic_frames_dir / f"{frame_id:06d}.pcd")

    write_pcd_from_points(static, out_dir / "fsc_after.pcd")
    write_pcd_from_points(dynamic, out_dir / "fsc_removed.pcd")
    if args.write_before:
        write_pcd_from_points(before, out_dir / "fsc_before.pcd")

    print(
        f"[done] before={len(before)} static={len(static)} dynamic={len(dynamic)} "
        f"voxels={len(vmap)} carved_frames={carved_frames} "
        f"cleared_voxels={cleared_voxels} skipped_no_origin={skipped_no_origin}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
