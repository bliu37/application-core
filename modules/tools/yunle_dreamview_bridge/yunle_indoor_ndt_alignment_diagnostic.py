#!/usr/bin/env python3

"""Compare one live lidar frame with the saved LIORF map."""

import argparse
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
from scipy.spatial import cKDTree

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.sensor_msgs import pointcloud_pb2
from yunle_pcd_topdown_renderer import read_pcd_points


DEFAULT_MAP = Path(
    "/apollo_workspace/data/map_work/yunle_indoor/"
    "jd03_indoor_20260803_01/global.pcd")
DEFAULT_TOPIC = "/apollo/sensor/lslidar16v4/PointCloud2"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Find which right-angle Z rotation best aligns a live LS-C16 "
            "frame with the saved LIORF global.pcd."))
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--voxel", type=float, default=0.20)
    parser.add_argument("--max-source-points", type=int, default=5000)
    return parser.parse_args()


def filtered_array(points):
    array = np.asarray(points, dtype=np.float64)
    array = array[np.all(np.isfinite(array), axis=1)]
    radius = np.hypot(array[:, 0], array[:, 1])
    return array[(radius >= 1.0) & (radius <= 30.0) &
                 (array[:, 2] >= -2.5) & (array[:, 2] <= 2.5)]


def voxel_filter(points, voxel):
    keys = np.floor(points / voxel).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def score_pose(tree, rotated, tx, ty, tz=0.0):
    candidate = rotated + np.array((tx, ty, tz))
    distances, _ = tree.query(candidate, k=1)
    clipped = np.minimum(distances, 2.0)
    return (float(np.mean(clipped * clipped)),
            float(np.median(distances)),
            float(np.mean(distances < 0.50)))


def grid_values(center, half_width, step):
    count = int(round(2.0 * half_width / step))
    return [center - half_width + index * step
            for index in range(count + 1)]


def optimize_translation(tree, rotated):
    best = None
    for tx in grid_values(0.0, 5.0, 1.0):
        for ty in grid_values(0.0, 5.0, 1.0):
            metrics = score_pose(tree, rotated, tx, ty)
            candidate = (metrics[0], tx, ty, metrics[1], metrics[2])
            if best is None or candidate[0] < best[0]:
                best = candidate

    coarse_tx, coarse_ty = best[1], best[2]
    for tx in grid_values(coarse_tx, 1.0, 0.10):
        for ty in grid_values(coarse_ty, 1.0, 0.10):
            metrics = score_pose(tree, rotated, tx, ty)
            candidate = (metrics[0], tx, ty, metrics[1], metrics[2])
            if candidate[0] < best[0]:
                best = candidate
    return best


def optimize_translation_3d(tree, rotated, initial_tx, initial_ty):
    best = None
    for tz in grid_values(0.0, 2.0, 0.25):
        metrics = score_pose(tree, rotated, initial_tx, initial_ty, tz)
        candidate = (metrics[0], initial_tx, initial_ty, tz,
                     metrics[1], metrics[2])
        if best is None or candidate[0] < best[0]:
            best = candidate

    coarse_tz = best[3]
    for tx in grid_values(initial_tx, 0.30, 0.10):
        for ty in grid_values(initial_ty, 0.30, 0.10):
            for tz in grid_values(coarse_tz, 0.50, 0.10):
                metrics = score_pose(tree, rotated, tx, ty, tz)
                candidate = (metrics[0], tx, ty, tz,
                             metrics[1], metrics[2])
                if candidate[0] < best[0]:
                    best = candidate
    return best


def z_summary(points):
    percentiles = np.percentile(points[:, 2], (5.0, 50.0, 95.0))
    return "p05={:+.2f} median={:+.2f} p95={:+.2f}".format(*percentiles)


def main():
    args = parse_args()
    if not math.isfinite(args.timeout) or not 0.0 < args.timeout <= 60.0:
        raise ValueError("timeout must be in (0, 60]")
    if not math.isfinite(args.voxel) or not 0.05 <= args.voxel <= 1.0:
        raise ValueError("voxel must be in [0.05, 1.0]")
    if not 500 <= args.max_source_points <= 20000:
        raise ValueError("max-source-points must be in [500, 20000]")

    map_path = args.map.resolve()
    map_points, _, _ = read_pcd_points(str(map_path))
    map_array = voxel_filter(filtered_array(map_points), args.voxel)
    if map_array.shape[0] < 100:
        raise RuntimeError("saved map has too few usable points")

    event = threading.Event()
    captured = []

    def callback(message):
        if event.is_set():
            return
        captured.extend((point.x, point.y, point.z)
                        for point in message.point)
        event.set()

    cyber.init()
    node = cyber.Node(
        "yunle_indoor_ndt_alignment_diagnostic_{}".format(os.getpid()))
    reader = node.create_reader(args.topic, pointcloud_pb2.PointCloud, callback)
    if reader is None:
        raise RuntimeError("failed to create point-cloud reader")
    print("Waiting for one live lidar frame on {}...".format(args.topic),
          flush=True)
    deadline = time.monotonic() + args.timeout
    while not event.is_set() and time.monotonic() < deadline:
        time.sleep(0.05)
    cyber.shutdown()
    if not captured:
        print("RESULT: INCOMPLETE -- no live lidar frame received.", flush=True)
        return 2

    source = voxel_filter(filtered_array(captured), args.voxel)
    if source.shape[0] > args.max_source_points:
        indices = np.linspace(
            0, source.shape[0] - 1, args.max_source_points, dtype=np.int64)
        source = source[indices]
    tree = cKDTree(map_array)

    rotations = (
        ("identity_0deg", np.array(((1.0, 0.0), (0.0, 1.0)))),
        ("plus_90deg", np.array(((0.0, -1.0), (1.0, 0.0)))),
        ("minus_90deg", np.array(((0.0, 1.0), (-1.0, 0.0)))),
        ("rotate_180deg", np.array(((-1.0, 0.0), (0.0, -1.0)))),
    )
    results = []
    print("map_points={} source_points={}".format(
        map_array.shape[0], source.shape[0]), flush=True)
    print("map_z: {} source_z: {}".format(
        z_summary(map_array), z_summary(source)), flush=True)
    rotated_by_label = {}
    for label, rotation in rotations:
        rotated = np.array(source, copy=True)
        rotated[:, :2] = np.dot(source[:, :2], rotation.T)
        rotated_by_label[label] = rotated
        best = optimize_translation(tree, rotated)
        results.append((best[0], label, best))
        print(("{}: translation=({:+.2f},{:+.2f}) capped_mse={:.4f} "
               "median_nn={:.4f}m inliers_lt_0.5m={:.1%}").format(
                   label, best[1], best[2], best[0], best[3], best[4]),
              flush=True)

    results.sort()
    winner = results[0]
    runner_up = results[1]
    ratio = runner_up[0] / winner[0] if winner[0] > 0.0 else float("inf")
    print("BEST_ROTATION={} score_margin={:.3f}x".format(
        winner[1], ratio), flush=True)
    best_2d = winner[2]
    best_3d = optimize_translation_3d(
        tree, rotated_by_label[winner[1]], best_2d[1], best_2d[2])
    print(("BEST_3D_TRANSLATION=({:+.2f},{:+.2f},{:+.2f}) "
           "capped_mse={:.4f} median_nn={:.4f}m "
           "inliers_lt_0.5m={:.1%}").format(
               best_3d[1], best_3d[2], best_3d[3], best_3d[0],
               best_3d[4], best_3d[5]), flush=True)

    z_probe_parts = []
    for delta_z in (-0.50, -0.25, 0.0, 0.25, 0.50):
        metrics = score_pose(
            tree, rotated_by_label[winner[1]], best_3d[1], best_3d[2],
            best_3d[3] + delta_z)
        z_probe_parts.append("{:+.2f}:{:.4f}".format(delta_z, metrics[0]))
    print("Z_SCORE_AROUND_BEST delta_z:capped_mse {}".format(
        " ".join(z_probe_parts)), flush=True)
    print("RESULT: live lidar/map orientation comparison completed.", flush=True)
    return 0


if __name__ == "__main__":
    os._exit(main())
