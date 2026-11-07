# SPDX-License-Identifier: MPL-2.0
# Copyright (c) FineNav
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""KITTI dataset preparation shared by dynamic-removal workflows."""

import hashlib
import os
import shutil
from pathlib import Path

import numpy as np
import yaml

from .dynamic_removal_common import console
from .dynamic_removal_kitti import convert_bag_to_kitti, convert_frames_to_kitti

def _load_map_config(map_path):
    config_path = os.path.join(map_path, "config.yaml")
    try:
        from .config import DEFAULT_CONFIG, load_config
        return load_config(config_path) if os.path.exists(config_path) else DEFAULT_CONFIG
    except Exception as e:
        raise RuntimeError(f"读取 config.yaml 失败: {e}") from e


def _has_current_bag_local_transform(seq_dir):
    notes_path = os.path.join(seq_dir, "conversion_notes.txt")
    if not os.path.exists(notes_path):
        return False
    notes = Path(notes_path).read_text(errors="replace")
    return (
        "source_bag:" in notes
        and "point_transform:" in notes
        and "cloud_frame_written: base_link" in notes
        and "time_source:" in notes
        and (
            "pose_source: interactive_slam_corrected" in notes
            or "pose_source: raw_odom" in notes
        )
    )


def _ensure_kitti_dataset(map_path):
    """确保 KITTI 使用逐扫描局部点云和 Interactive SLAM 修正轨迹。"""
    kitti_root = os.path.join(map_path, "erasor2_dataset")
    seq_dir = os.path.join(kitti_root, "dataset", "sequences", "00")
    velodyne_dir = os.path.join(seq_dir, "velodyne")

    if os.path.isdir(velodyne_dir) and os.listdir(velodyne_dir):
        bin_files = [f for f in os.listdir(velodyne_dir) if f.endswith(".bin")]
        if _has_current_bag_local_transform(seq_dir):
            frame_count = len(bin_files)
            print(f"复用已有 corrected bag-local KITTI 数据集: {velodyne_dir} ({frame_count} 帧)")
            return kitti_root, frame_count
        print("检测到未使用 Interactive SLAM 修正轨迹的 KITTI 数据集，将重新生成。")

    bag_dir = os.path.join(map_path, "bag")
    if not os.path.isdir(bag_dir):
        raise RuntimeError(f"bag 目录不存在，无法生成 KITTI 数据集: {bag_dir}")

    print("从 bag 逐扫描点云和 Interactive SLAM 修正轨迹生成 KITTI...")
    return convert_bag_to_kitti(map_path, _load_map_config(map_path))


def _prepare_z_limited_kitti_dataset(kitti_root, map_path, method_name, z_min, z_max):
    """Create a method-specific KITTI dataset with local-frame z filtering."""
    source_root = Path(kitti_root)
    source_seq = source_root / "dataset" / "sequences" / "00"
    source_velodyne = source_seq / "velodyne"
    scan_paths = sorted(source_velodyne.glob("*.bin"))
    if not scan_paths:
        raise FileNotFoundError(f"KITTI 输入目录中没有点云: {source_velodyne}")

    limited_root = Path(map_path) / f"{method_name}_dataset_z_limited"
    limited_seq = limited_root / "dataset" / "sequences" / "00"
    limited_velodyne = limited_seq / "velodyne"
    limited_labels = limited_seq / "labels"
    if limited_root.exists():
        shutil.rmtree(limited_root)
    limited_velodyne.mkdir(parents=True)
    limited_labels.mkdir(parents=True)

    reports = []
    total_before = 0
    total_after = 0
    for scan_path in scan_paths:
        scan = np.fromfile(scan_path, dtype=np.float32)
        if scan.size % 4 != 0:
            raise ValueError(f"KITTI 点云不是 xyzi 格式: {scan_path}")
        scan = scan.reshape(-1, 4)
        keep = np.ones(len(scan), dtype=bool)
        keep &= scan[:, 2] >= z_min
        keep &= scan[:, 2] <= z_max
        filtered = scan[keep].astype(np.float32, copy=False)
        filtered.tofile(limited_velodyne / scan_path.name)
        np.zeros(len(filtered), dtype=np.uint32).tofile(
            limited_labels / f"{scan_path.stem}.label"
        )

        before_count = len(scan)
        after_count = len(filtered)
        total_before += before_count
        total_after += after_count
        reports.append(
            {
                "frame": scan_path.stem,
                "before_points": before_count,
                "after_points": after_count,
            }
        )

    for source_path in source_seq.iterdir():
        if source_path.is_file():
            shutil.copy2(source_path, limited_seq / source_path.name)

    (limited_seq / "z_filter_report.yaml").write_text(
        yaml.safe_dump(
            {
                "source_dataset": str(source_root.resolve()),
                "method": method_name,
                "local_z_min": z_min,
                "local_z_max": z_max,
                "input_frames": len(scan_paths),
                "input_points": total_before,
                "output_points": total_after,
                "frames": reports,
            },
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    console.print(
        f"[dim]{method_name} 局部 Z 过滤: [{z_min:g}, {z_max:g}] m, "
        f"{total_before:,} -> {total_after:,} 点[/dim]"
    )
    return str(limited_root), len(scan_paths)


def _ensure_local_kitti_dataset(map_path):
    """确保存在适合 local hash voxel / raycasting 的逐帧 local KITTI 数据集。"""
    kitti_root = os.path.join(map_path, "erasor2_dataset")
    seq_dir = os.path.join(kitti_root, "dataset", "sequences", "00")
    velodyne_dir = os.path.join(seq_dir, "velodyne")
    pose_path = os.path.join(seq_dir, "poses_odom_base.txt")

    if os.path.isdir(velodyne_dir) and os.path.exists(pose_path):
        bin_files = [f for f in os.listdir(velodyne_dir) if f.endswith(".bin")]
        if bin_files and _has_current_bag_local_transform(seq_dir):
            _require_sensor_trajectory(seq_dir)
            print(f"复用已有 corrected 逐帧 KITTI 数据集: {velodyne_dir} ({len(bin_files)} 帧)")
            return kitti_root, len(bin_files)
        if bin_files:
            print("检测到未使用 Interactive SLAM 修正轨迹的 KITTI 数据集，将重新生成。")

    print("生成 bag 逐扫描点云 + corrected 插值轨迹 KITTI 数据集...")
    kitti_root, frame_count = convert_bag_to_kitti(map_path, _load_map_config(map_path))
    _require_sensor_trajectory(seq_dir)
    return kitti_root, frame_count


def _frames_signature(pcd_files, odom_files):
    """frame/ 内容指纹：位姿按内容、点云按文件名/大小/时间，用于判断数据集是否过时。"""
    digest = hashlib.sha256()
    for path in pcd_files:
        stat = path.stat()
        digest.update(f"pcd {path.name} {stat.st_size} {stat.st_mtime_ns}\n".encode())
    for path in odom_files:
        digest.update(f"odom {path.name}\n".encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _ensure_frames_kitti_dataset(map_path):
    """确保存在由 frame/*.pcd + frame/*.odom 位姿生成的逐帧 KITTI 数据集。

    位姿直接来自 frame/（含平面约束、Interactive SLAM 的修正结果），不从原始
    bag 的 TF 推导；输出到独立目录，避免覆盖其他方法使用的 bag 逐扫描数据集。
    frame/ 内容变化（尤其是位姿修正）后会自动重建，避免使用旧位姿。
    """
    frame_dir = Path(map_path) / "frame"
    pcd_files = sorted(frame_dir.glob("*.pcd"))
    odom_files = sorted(frame_dir.glob("*.odom"))
    if not pcd_files:
        raise FileNotFoundError(
            f"frame/ 中没有 .pcd 文件: {frame_dir}；请先运行 Frame Extractor"
        )

    kitti_root = os.path.join(map_path, "free_space_carving_dataset")
    seq_dir = os.path.join(kitti_root, "dataset", "sequences", "00")
    velodyne_dir = os.path.join(seq_dir, "velodyne")
    pose_path = os.path.join(seq_dir, "poses_odom_base.txt")
    notes_path = Path(seq_dir) / "conversion_notes.txt"
    signature = _frames_signature(pcd_files, odom_files)
    marker = f"frame_signature: {signature}"

    bin_count = (
        len(list(Path(velodyne_dir).glob("*.bin")))
        if os.path.isdir(velodyne_dir) else 0
    )
    notes = notes_path.read_text(errors="replace") if notes_path.exists() else ""
    reused = (
        bin_count == len(pcd_files)
        and os.path.exists(pose_path)
        and "pose_source: frame_odom_base" in notes
        and marker in notes
    )
    if reused:
        _require_sensor_trajectory(seq_dir)
        print(f"复用已有 frame 位姿 KITTI 数据集: {velodyne_dir} ({bin_count} 帧)")
        return kitti_root, bin_count

    print("从 frame/*.pcd 点云与 frame/*.odom 位姿生成 KITTI 数据集...")
    root, frame_count = convert_frames_to_kitti(map_path, kitti_root=kitti_root)
    with notes_path.open("a", encoding="utf-8") as f:
        f.write(marker + "\n")
    _require_sensor_trajectory(seq_dir)
    return root, frame_count


def _require_sensor_trajectory(seq_dir):
    pose_path = os.path.join(seq_dir, "poses_odom_base.txt")
    identity_path = os.path.join(seq_dir, "poses_identity.txt")

    if not os.path.exists(pose_path):
        if os.path.exists(identity_path):
            raise RuntimeError(
                "当前数据集只有 poses_identity.txt，没有真实传感器轨迹。"
                "local hash voxel 和 raycasting 需要逐帧传感器位姿，YunJingFull 这类数据暂不支持。"
            )
        raise RuntimeError(f"缺少真实传感器轨迹文件: {pose_path}")

    translations = []
    with open(pose_path, "r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            vals = [float(x) for x in line.split()]
            if len(vals) == 12:
                translations.append((vals[3], vals[7], vals[11]))
            elif len(vals) == 16:
                translations.append((vals[3], vals[7], vals[11]))
            else:
                raise RuntimeError(f"不支持的 pose 格式: {pose_path}")

    if len(translations) < 2:
        raise RuntimeError("真实传感器轨迹少于 2 帧，无法进行 local hash voxel/raycasting。")

    arr = np.asarray(translations, dtype=np.float64)
    movement = np.linalg.norm(arr - arr[0], axis=1).max()
    if movement < 1e-3:
        raise RuntimeError(
            "检测到传感器轨迹几乎全为同一位姿，无法进行 local hash voxel/raycasting。"
            "YunJingFull 这类无真实传感器轨迹的数据请先跳过。"
        )
