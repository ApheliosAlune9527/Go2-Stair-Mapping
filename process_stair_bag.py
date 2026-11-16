#!/usr/bin/env python3
"""Offline trim + 4-DOF Point-to-Plane ICP 3D stair reconstruction for Go2 bags."""

import argparse
import glob
import math
import os
import shutil
import sqlite3
import sys

import numpy as np
from scipy.spatial import cKDTree

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from unitree_go.msg import SportModeState


def find_latest_raw_bag(bags_dir: str) -> str:
    candidates = sorted(
        [
            p
            for p in glob.glob(os.path.join(bags_dir, "*"))
            if os.path.isdir(p)
            and os.path.exists(os.path.join(p, "metadata.yaml"))
            and not p.endswith("_3d")
        ],
        key=os.path.getmtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(f"在 {bags_dir} 下未找到可用的 ROS 2 数据包")
    return candidates[0]


def find_db3_file(bag_dir: str) -> str:
    files = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    if not files:
        raise FileNotFoundError(f"在 {bag_dir} 中未找到 .db3 文件")
    if len(files) > 1:
        raise RuntimeError(f"{bag_dir} 包含多个 .db3 分片，当前脚本仅支持单文件 db3")
    return files[0]


def quat_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def inspect_timeline(db_path: str):
    """Detect motion segments (climb up / descend / flat) from robot_odom (+ sportmodestate)."""
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    topics = dict(conn.execute("SELECT name, id FROM topics"))
    t0 = conn.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    t1 = conn.execute("SELECT MAX(timestamp) FROM messages").fetchone()[0]
    duration = (t1 - t0) / 1e9

    if "/utlidar/robot_odom" not in topics or "/utlidar/cloud_deskewed" not in topics:
        conn.close()
        return t0, duration, [], (0.0, duration)

    ro_rows = conn.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topics["/utlidar/robot_odom"],),
    ).fetchall()

    sm_by_bin = {}
    if "/sportmodestate" in topics:
        for ts, blob in conn.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics["/sportmodestate"],),
        ):
            b = int((ts - t0) / 5e8)
            if b not in sm_by_bin:
                sm_by_bin[b] = deserialize_message(blob, SportModeState)
    conn.close()

    # Sample at 0.5s bins
    bins = {}
    for ts, blob in ro_rows:
        b = int((ts - t0) / 5e8)
        if b not in bins:
            om = deserialize_message(blob, Odometry)
            p = om.pose.pose.position
            _, pitch, yaw = quat_to_rpy(om.pose.pose.orientation)
            bins[b] = ((ts - t0) / 1e9, p.x, p.y, math.degrees(pitch), yaw)

    sorted_b = sorted(bins.keys())
    active_bins = []
    for idx in range(len(sorted_b)):
        b = sorted_b[idx]
        t_s, x, y, pitch_deg, yaw = bins[b]
        speed = 0.0
        yaw_rate = 0.0
        if idx > 0:
            pb = sorted_b[idx - 1]
            pt, px, py, _, pyaw = bins[pb]
            dt = max(1e-3, t_s - pt)
            speed = math.hypot(x - px, y - py) / dt
            dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
            yaw_rate = abs(dyaw) / dt
        if b in sm_by_bin:
            sm = sm_by_bin[b]
            speed = max(speed, math.hypot(sm.velocity[0], sm.velocity[1]))
            yaw_rate = max(yaw_rate, abs(sm.yaw_speed))
            pitch_deg = math.degrees(sm.imu_state.rpy[1])
        is_moving = speed > 0.09 or yaw_rate > 0.25 or abs(pitch_deg) > 6.0
        active_bins.append((t_s, is_moving, pitch_deg, yaw_rate))

    # Merge active regions with <= 2.0s gap
    segments = []
    cur_start = None
    cur_end = None
    pitches = []
    last_active_t = None
    for t_s, moving, pitch_deg, _ in active_bins:
        if moving:
            if cur_start is None:
                cur_start = t_s
                pitches = []
            cur_end = t_s
            last_active_t = t_s
            pitches.append(pitch_deg)
        else:
            if cur_start is not None and last_active_t is not None and (t_s - last_active_t) > 2.0:
                if cur_end - cur_start >= 1.5:
                    segments.append((cur_start, cur_end, float(np.min(pitches)), float(np.max(pitches))))
                cur_start = None
    if cur_start is not None and cur_end - cur_start >= 1.5:
        segments.append((cur_start, cur_end, float(np.min(pitches)), float(np.max(pitches))))

    # Pick primary climb-up segment (strongest negative pitch, Go2 pitch < -8 deg when climbing up)
    climb_candidates = [s for s in segments if s[2] <= -8.0]
    if climb_candidates:
        best = min(climb_candidates, key=lambda s: s[2])
        rec_start = max(0.0, best[0] - 2.5)
        rec_end = min(duration, best[1] + 3.5)
    elif segments:
        best = max(segments, key=lambda s: s[1] - s[0])
        rec_start = max(0.0, best[0] - 2.0)
        rec_end = min(duration, best[1] + 2.5)
    else:
        rec_start, rec_end = 0.0, duration

    labeled = []
    for s0, s1, pmin, pmax in segments:
        if pmin <= -8.0 and pmax >= 8.0:
            kind = "上楼梯+下楼梯"
        elif pmin <= -8.0:
            kind = f"上楼梯 (最大仰角 {pmin:.1f}°)"
        elif pmax >= 8.0:
            kind = f"下楼梯 (最大俯角 +{pmax:.1f}°)"
        else:
            kind = "平地移动/转向"
        labeled.append((s0, s1, kind))

    return t0, duration, labeled, (round(rec_start, 1), round(rec_end, 1))


def voxel_ds(pts: np.ndarray, v: float) -> np.ndarray:
    if len(pts) == 0:
        return pts
    q = np.floor(pts / v).astype(np.int64)
    _, idx = np.unique(q, axis=0, return_index=True)
    return pts[idx]


def estimate_normals(pts: np.ndarray, k: int = 10):
    tree = cKDTree(pts)
    k_eff = min(k, len(pts))
    _, idxs = tree.query(pts, k=k_eff, workers=4)
    nbrs = pts[idxs]
    centered = nbrs - nbrs.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", centered, centered)
    eigvals, eigvecs = np.linalg.eigh(cov)
    normals = eigvecs[:, :, 0]
    planar = eigvals[:, 0] < 0.06 * (eigvals[:, 1] + 1e-6)
    return normals, planar


def icp_4dof_centered(src_pts, robot_xy, map_pts, map_n, map_planar, tx, ty, tz, dyaw, iters=18, max_dist=0.20):
    tree = cKDTree(map_pts)
    rx, ry = robot_xy
    for _ in range(iters):
        c, s = math.cos(dyaw), math.sin(dyaw)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        rel = src_pts - np.array([rx, ry, 0.0])
        p = rel @ R.T + np.array([rx + tx, ry + ty, tz])
        dists, idxs = tree.query(p, workers=4)
        mask = (dists < max_dist) & map_planar[idxs]
        if mask.sum() < 80:
            break
        q = map_pts[idxs[mask]]
        n = map_n[idxs[mask]]
        pm = p[mask]
        r = np.sum((pm - q) * n, axis=1)
        med = np.median(np.abs(r)) + 1e-3
        keep = np.abs(r) < max(0.05, 2.5 * med)
        if keep.sum() < 60:
            break
        pm, n, r = pm[keep], n[keep], r[keep]
        dx_c = pm[:, 0] - (rx + tx)
        dy_c = pm[:, 1] - (ry + ty)
        J = np.column_stack([n[:, 0], n[:, 1], n[:, 2], -dy_c * n[:, 0] + dx_c * n[:, 1]])
        H = J.T @ J + np.diag([0.5, 0.5, 0.1, 40.0])
        g = -J.T @ r
        delta = np.linalg.solve(H, g)
        tx += delta[0]
        ty += delta[1]
        tz += delta[2]
        dyaw += delta[3]
        if np.linalg.norm(delta) < 1e-4:
            break
    return tx, ty, tz, dyaw


def make_xyzrgb_msg(pts: np.ndarray, rgb_float: np.ndarray, stamp) -> PointCloud2:
    n = len(pts)
    arr = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
    arr["x"] = pts[:, 0]
    arr["y"] = pts[:, 1]
    arr["z"] = pts[:, 2]
    arr["rgb"] = rgb_float
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id="odom")
    msg.height = 1
    msg.width = n
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = arr.tobytes()
    return msg


def make_xyz_msg(pts: np.ndarray, stamp) -> PointCloud2:
    n = len(pts)
    pts32 = np.ascontiguousarray(pts, dtype=np.float32)
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id="odom")
    msg.height = 1
    msg.width = n
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = pts32.tobytes()
    return msg


def filter_roi_around_trajectory(pts: np.ndarray, traj_xy_z: np.ndarray, tangents_xy: np.ndarray, cutaway: bool) -> np.ndarray:
    """Clip walls/ceiling relative to actual ICP trajectory so any stair angle/height works cleanly."""
    traj_tree = cKDTree(traj_xy_z[:, :2])
    dists, idx = traj_tree.query(pts[:, :2], workers=4)
    nearest_traj = traj_xy_z[idx]
    nearest_tan = tangents_xy[idx]
    delta_xy = pts[:, :2] - nearest_traj[:, :2]
    # Signed lateral offset: + = left of climb direction, - = right of climb direction
    lat = nearest_tan[:, 0] * delta_xy[:, 1] - nearest_tan[:, 1] * delta_xy[:, 0]
    min_lat = -0.45 if cutaway else -1.05
    max_lat = 0.95 if cutaway else 1.15
    z_floor = float(np.min(traj_xy_z[:, 2])) - 0.25
    return (
        (dists <= 2.6)
        & (lat >= min_lat)
        & (lat <= max_lat)
        & (pts[:, 2] >= z_floor)
        & (pts[:, 2] <= nearest_traj[:, 2] + 1.15)
    )


def build_3d_bag(src_bag: str, dst_bag: str, start_sec: float, end_sec: float, cutaway: bool):
    db_path = find_db3_file(src_bag)
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    topics = dict(conn.execute("SELECT name, id FROM topics"))
    t0 = conn.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    t_start = t0 + int(start_sec * 1e9)
    t_end = t0 + int(end_sec * 1e9)

    if "/utlidar/cloud_deskewed" not in topics or "/utlidar/robot_odom" not in topics:
        conn.close()
        raise RuntimeError("数据包缺少 /utlidar/cloud_deskewed 或 /utlidar/robot_odom，无法做 3D 重建")

    cd_rows = conn.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp",
        (topics["/utlidar/cloud_deskewed"], t_start, t_end),
    ).fetchall()
    ro_rows = conn.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp",
        (topics["/utlidar/robot_odom"], t_start, t_end),
    ).fetchall()
    conn.close()

    if len(cd_rows) < 6 or len(ro_rows) < 6:
        raise RuntimeError(f"选定区间 [{start_sec:.1f}s, {end_sec:.1f}s] 内雷达帧太少 ({len(cd_rows)} 帧)")

    ro_ts = np.array([r[0] for r in ro_rows], dtype=np.int64)
    om0 = deserialize_message(ro_rows[int(np.argmin(np.abs(ro_ts - cd_rows[0][0])))][1], Odometry)
    p0 = np.array([om0.pose.pose.position.x, om0.pose.pose.position.y, om0.pose.pose.position.z])
    _, _, yaw0 = quat_to_rpy(om0.pose.pose.orientation)
    c0, s0 = math.cos(-yaw0), math.sin(-yaw0)
    R0 = np.array([[c0, -s0, 0.0], [s0, c0, 0.0], [0.0, 0.0, 1.0]])

    frames = []
    frame_stamps = []
    frame_ts = []
    raw_odom_at_scan = []
    for ts, blob in cd_rows:
        om = deserialize_message(ro_rows[int(np.argmin(np.abs(ro_ts - ts)))][1], Odometry)
        pr = np.array([om.pose.pose.position.x, om.pose.pose.position.y, om.pose.pose.position.z])
        raw_odom_at_scan.append(pr)
        m = deserialize_message(blob, PointCloud2)
        pts = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
        xyz = np.ascontiguousarray(pts[:, :12]).view(np.float32).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        xyz = xyz[(xyz[:, 0] != 0) | (xyz[:, 1] != 0) | (xyz[:, 2] != 0)]
        xyz = xyz[np.linalg.norm(xyz - pr, axis=1) > 0.50]
        frames.append((xyz - p0) @ R0.T)
        frame_stamps.append(m.header.stamp)
        frame_ts.append(ts)

    print(f"[1/4] 正在对 {len(frames)} 帧 /utlidar/cloud_deskewed 执行 4-DOF ICP 3D 配准...")
    init_n = min(5, len(frames))
    map_pts = voxel_ds(np.vstack(frames[:init_n]), 0.025)
    map_n, map_p = estimate_normals(map_pts)
    tx, ty, tz, dyaw = 0.0, 0.0, 0.0, 0.0
    last_kf = np.array([0.0, 0.0, 0.0])
    aligned_frames = []
    icp_poses = []
    traj_pts = []

    for i, f in enumerate(frames):
        rxy = ((raw_odom_at_scan[i] - p0) @ R0.T)[:2]
        f_ds = voxel_ds(f, 0.035)
        if len(f_ds) >= 80:
            tx, ty, tz, dyaw = icp_4dof_centered(f_ds, rxy, map_pts, map_n, map_p, tx, ty, tz, dyaw)
        c, s = math.cos(dyaw), math.sin(dyaw)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        rel = f - np.array([rxy[0], rxy[1], 0.0])
        al = rel @ R.T + np.array([rxy[0] + tx, rxy[1] + ty, tz])
        aligned_frames.append(al)
        cur_p = np.array([rxy[0] + tx, rxy[1] + ty, tz])
        icp_poses.append((rxy[0], rxy[1], tx, ty, tz, dyaw))
        traj_pts.append(cur_p)
        if i >= init_n and np.linalg.norm(cur_p - last_kf) >= 0.20:
            map_pts = voxel_ds(np.vstack([map_pts, voxel_ds(al, 0.025)]), 0.025)
            map_n, map_p = estimate_normals(map_pts)
            last_kf = cur_p

    icp_poses = np.array(icp_poses)
    frame_ts = np.array(frame_ts, dtype=np.int64)
    traj_pts = np.array(traj_pts)
    print(f"      配准完成：末端修正 tx={tx:.2f}m, ty={ty:.2f}m, 爬升高度 tz={tz:.2f}m, 航向补偿={math.degrees(dyaw):.1f}°")

    # Compute smooth local climb tangents for trajectory-relative cutaway clipping
    tangents = np.zeros((len(traj_pts), 2), dtype=np.float64)
    for i in range(len(traj_pts)):
        i0 = max(0, i - 10)
        i1 = min(len(traj_pts) - 1, i + 10)
        v = traj_pts[i1, :2] - traj_pts[i0, :2]
        norm = np.linalg.norm(v)
        if norm > 0.05:
            tangents[i] = v / norm
        else:
            tangents[i] = [1.0, 0.0] if i == 0 else tangents[i - 1]

    print("[2/4] 正在去噪并生成踏面/立面对比着色点云...")
    voxel_size = 0.018
    voxel_dict = {}
    scan_roi_masks = []
    for i, al in enumerate(aligned_frames):
        mask = filter_roi_around_trajectory(al, traj_pts, tangents, cutaway=cutaway)
        scan_roi_masks.append(mask)
        sub = al[mask]
        if len(sub) == 0:
            continue
        q = np.floor(sub / voxel_size).astype(np.int64)
        _, u_idx = np.unique(q, axis=0, return_index=True)
        for key, pt in zip(map(tuple, q[u_idx]), sub[u_idx]):
            if key not in voxel_dict:
                voxel_dict[key] = (pt, i)

    all_pts = np.array([v[0] for v in voxel_dict.values()], dtype=np.float32)
    first_seen = np.array([v[1] for v in voxel_dict.values()], dtype=np.int32)

    tree = cKDTree(all_pts)
    dists, _ = tree.query(all_pts, k=min(8, len(all_pts)), workers=4)
    keep = dists[:, min(5, dists.shape[1] - 1)] < 0.055
    all_pts = all_pts[keep]
    first_seen = first_seen[keep]

    tree2 = cKDTree(all_pts)
    _, idxs2 = tree2.query(all_pts, k=min(16, len(all_pts)), workers=4)
    nbrs = all_pts[idxs2]
    centered = nbrs - nbrs.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", centered, centered)
    _, eigvecs = np.linalg.eigh(cov)
    normals = eigvecs[:, :, 0]
    flip = (normals[:, 2] * 0.8 - normals[:, 0] * 0.5) < 0
    normals[flip] *= -1

    light = np.array([-0.4, -0.3, 0.86])
    light /= np.linalg.norm(light)
    shade = np.clip(0.35 + 0.65 * np.abs(normals @ light), 0.25, 1.0)
    is_tread = np.abs(normals[:, 2]) > 0.72

    z_lo = float(np.percentile(all_pts[:, 2], 1))
    z_hi = float(np.percentile(all_pts[:, 2], 99))
    z_span = max(0.5, z_hi - z_lo)
    t_val = np.clip((all_pts[:, 2] - z_lo) / z_span, 0.0, 1.0)
    r_col = np.clip(1.5 - np.abs(4 * t_val - 3), 0, 1)
    g_col = np.clip(1.5 - np.abs(4 * t_val - 2), 0, 1)
    b_col = np.clip(1.5 - np.abs(4 * t_val - 1), 0, 1)
    rgb = np.column_stack([r_col, g_col, b_col]) * shade[:, None]
    rgb[is_tread] = np.clip(rgb[is_tread] * 1.15 + 0.08, 0, 1)
    rgb[~is_tread] *= 0.78

    r_u8 = np.clip(rgb[:, 0] * 255, 0, 255).astype(np.uint32)
    g_u8 = np.clip(rgb[:, 1] * 255, 0, 255).astype(np.uint32)
    b_u8 = np.clip(rgb[:, 2] * 255, 0, 255).astype(np.uint32)
    alpha = np.full_like(r_u8, 255, dtype=np.uint32)
    rgb_f32 = ((alpha << 24) | (r_u8 << 16) | (g_u8 << 8) | b_u8).view(np.float32)

    print(f"[3/4] 正在写入输出数据包: {dst_bag}")
    if os.path.exists(dst_bag):
        shutil.rmtree(dst_bag)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=src_bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=dst_bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    for tm in reader.get_all_topics_and_types():
        writer.create_topic(tm)
    writer.create_topic(rosbag2_py.TopicMetadata(name="/stair_3d/cloud_map", type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name="/stair_3d/scan", type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name="/stair_3d/odom", type="nav_msgs/msg/Odometry", serialization_format="cdr"))

    extra_msgs = []
    for i in range(len(cd_rows)):
        ts = int(frame_ts[i])
        stmp = frame_stamps[i]
        scan_pts = aligned_frames[i][scan_roi_masks[i]]
        extra_msgs.append((ts, "/stair_3d/scan", serialize_message(make_xyz_msg(scan_pts, stmp))))
        if i % 3 == 0 or i == len(cd_rows) - 1:
            vis = first_seen <= i
            extra_msgs.append((ts, "/stair_3d/cloud_map", serialize_message(make_xyzrgb_msg(all_pts[vis], rgb_f32[vis], stmp))))

    for k in range(0, len(ro_rows), 3):
        ts, blob = ro_rows[k]
        om = deserialize_message(blob, Odometry)
        p = np.array([om.pose.pose.position.x, om.pose.pose.position.y, om.pose.pose.position.z])
        rx_i = float(np.interp(ts, frame_ts, icp_poses[:, 0]))
        ry_i = float(np.interp(ts, frame_ts, icp_poses[:, 1]))
        tx_i = float(np.interp(ts, frame_ts, icp_poses[:, 2]))
        ty_i = float(np.interp(ts, frame_ts, icp_poses[:, 3]))
        tz_i = float(np.interp(ts, frame_ts, icp_poses[:, 4]))
        dy_i = float(np.interp(ts, frame_ts, icp_poses[:, 5]))
        c_i, s_i = math.cos(dy_i), math.sin(dy_i)
        R_i = np.array([[c_i, -s_i, 0.0], [s_i, c_i, 0.0], [0.0, 0.0, 1.0]])
        p_base = (p - p0) @ R0.T
        p_al = (p_base - np.array([rx_i, ry_i, 0.0])) @ R_i.T + np.array([rx_i + tx_i, ry_i + ty_i, tz_i])
        q = om.pose.pose.orientation
        half = 0.5 * (-yaw0 + dy_i)
        qzw, qzz = math.cos(half), math.sin(half)
        om_out = Odometry()
        om_out.header = Header(stamp=om.header.stamp, frame_id="odom")
        om_out.child_frame_id = "base_link"
        om_out.pose.pose.position.x = float(p_al[0])
        om_out.pose.pose.position.y = float(p_al[1])
        om_out.pose.pose.position.z = float(p_al[2])
        om_out.pose.pose.orientation.w = qzw * q.w - qzz * q.z
        om_out.pose.pose.orientation.x = qzw * q.x - qzz * q.y
        om_out.pose.pose.orientation.y = qzw * q.y + qzz * q.x
        om_out.pose.pose.orientation.z = qzw * q.z + qzz * q.w
        extra_msgs.append((ts, "/stair_3d/odom", serialize_message(om_out)))

    extra_msgs.sort(key=lambda x: x[0])
    ei = 0
    en = len(extra_msgs)
    reader.seek(t_start)
    copied = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if ts < t_start:
            continue
        if ts > t_end:
            break
        while ei < en and extra_msgs[ei][0] <= ts:
            writer.write(extra_msgs[ei][1], extra_msgs[ei][2], extra_msgs[ei][0])
            ei += 1
        writer.write(topic, data, ts)
        copied += 1
    while ei < en:
        writer.write(extra_msgs[ei][1], extra_msgs[ei][2], extra_msgs[ei][0])
        ei += 1
    writer.close()
    print(f"[4/4] 完成! 输出: {dst_bag}")
    print(f"      裁剪区间: {start_sec:.1f}s ~ {end_sec:.1f}s (时长 {end_sec - start_sec:.1f}s), 保留原消息 {copied} 条, 3D体素 {len(all_pts)} 个")


def main():
    parser = argparse.ArgumentParser(description="Go2 爬楼梯数据包离线裁剪 + 3D ICP 重建工具")
    parser.add_argument("bag_path", nargs="?", help="原始数据包目录（默认自动选用 ~/unitree_ros2/bags 下最新原始包）")
    parser.add_argument("-o", "--output", help="输出数据包目录（默认 <原包>_3d）")
    parser.add_argument("--start", type=float, help="裁剪起始秒数（相对包开头）")
    parser.add_argument("--end", type=float, help="裁剪结束秒数（相对包开头）")
    parser.add_argument("--auto", action="store_true", help="非交互模式：自动选择上楼梯时间段")
    parser.add_argument("--no-trim", action="store_true", help="不裁剪时间，对整包重建")
    parser.add_argument("--no-cutaway", action="store_true", help="保留双侧墙体，不做近侧剖切")
    args = parser.parse_args()

    bags_dir = os.path.expanduser("~/unitree_ros2/bags")
    src_bag = os.path.abspath(os.path.expanduser(args.bag_path)) if args.bag_path else find_latest_raw_bag(bags_dir)
    if not os.path.isdir(src_bag):
        sys.exit(f"找不到目录: {src_bag}")

    print(f"目标数据包: {src_bag}")
    db_path = find_db3_file(src_bag)
    _, duration, labeled, (rec_s, rec_e) = inspect_timeline(db_path)
    print(f"数据包总时长: {duration:.2f}s")
    if labeled:
        print("检测到运动片段:")
        for s0, s1, kind in labeled:
            print(f"  - {s0:6.1f}s ~ {s1:6.1f}s : {kind}")
    else:
        print("未检测到明显爬坡/运动分段，默认保留全长。")

    if args.no_trim:
        start_sec, end_sec = 0.0, duration
    elif args.start is not None or args.end is not None:
        start_sec = 0.0 if args.start is None else max(0.0, args.start)
        end_sec = duration if args.end is None else min(duration, args.end)
    elif args.auto:
        start_sec, end_sec = rec_s, rec_e
    else:
        prompt = f"\n按回车使用推荐区间 [{rec_s:.1f} {rec_e:.1f}]，或输入 起始秒 结束秒 (如 {rec_s:.0f} {rec_e:.0f})，或输入 all 保留整包: "
        ans = input(prompt).strip()
        if not ans:
            start_sec, end_sec = rec_s, rec_e
        elif ans.lower() == "all":
            start_sec, end_sec = 0.0, duration
        else:
            parts = ans.split()
            if len(parts) != 2:
                sys.exit("输入格式错误，请重新运行并输入两个数字（如: 15 38）")
            start_sec = max(0.0, float(parts[0]))
            end_sec = min(duration, float(parts[1]))

    if end_sec <= start_sec + 1.0:
        sys.exit(f"无效区间: [{start_sec}, {end_sec}]")

    dst_bag = os.path.abspath(os.path.expanduser(args.output)) if args.output else f"{src_bag.rstrip('/')}_3d"
    build_3d_bag(src_bag, dst_bag, start_sec, end_sec, cutaway=not args.no_cutaway)


if __name__ == "__main__":
    main()
