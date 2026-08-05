#!/usr/bin/env python3

"""Verify fixed-map JD03 indoor localization without publishing commands."""

import argparse
import collections
import math
import os
import threading
import time

from cyber.python.cyber_py3 import cyber
from modules.slam_local.proto import localization_status_pb2
from modules.slam_local.proto import odometry_pb2


DEFAULT_MAP_DIR = (
    "/apollo_workspace/data/map_work/yunle_indoor/"
    "jd03_indoor_20260803_01")
REQUIRED_MAP_FILES = (
    "3D-Pose.txt",
    "6D-Pose.txt",
    "surfPointCloud.txt",
    "gnss-map-offset.txt",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Check the saved map files and summarize Apollo slam_local "
            "fixed-map localization status and odometry."))
    parser.add_argument("--map-dir", default=DEFAULT_MAP_DIR,
                        help="saved LIORF map directory")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="observation time in seconds, in (0, 120]")
    parser.add_argument(
        "--status-topic", default="liorf/mapping/localization_status",
        help="slam_local localization status channel")
    parser.add_argument("--odometry-topic", default="liorf/mapping/odometry",
                        help="slam_local odometry channel")
    return parser.parse_args()


def count_nonempty_lines(path):
    count = 0
    with open(path, "rb") as stream:
        for line in stream:
            if line.strip():
                count += 1
    return count


def validate_map(map_dir):
    map_dir = os.path.realpath(map_dir)
    if not os.path.isdir(map_dir):
        raise ValueError("map directory does not exist: {}".format(map_dir))

    details = []
    for name in REQUIRED_MAP_FILES:
        path = os.path.join(map_dir, name)
        if not os.path.isfile(path):
            raise ValueError("required map file is missing: {}".format(path))
        size = os.path.getsize(path)
        if size <= 0:
            raise ValueError("required map file is empty: {}".format(path))
        line_count = count_nonempty_lines(path)
        if line_count <= 0:
            raise ValueError("required map file has no records: {}".format(
                path))
        details.append((name, size, line_count))
    return map_dir, details


def quaternion_to_yaw(qx, qy, qz, qw):
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationMonitor(object):

    def __init__(self):
        self._lock = threading.Lock()
        self._qualities = collections.Counter()
        self._sources = collections.Counter()
        self._status_messages = 0
        self._poses = []

    def status_callback(self, message):
        if message.HasField("lidar_slam_quality"):
            quality_value = message.DESCRIPTOR.fields_by_name[
                "lidar_slam_quality"].enum_type.values_by_number.get(
                    message.lidar_slam_quality)
            quality = (quality_value.name
                       if quality_value is not None else
                       "UNKNOWN_{}".format(message.lidar_slam_quality))
        else:
            quality = "UNSET"
        if message.HasField("localization_souce"):
            source_value = message.DESCRIPTOR.fields_by_name[
                "localization_souce"].enum_type.values_by_number.get(
                    message.localization_souce)
            source = (source_value.name
                      if source_value is not None else
                      "UNKNOWN_{}".format(message.localization_souce))
        else:
            source = "UNSET"
        with self._lock:
            self._status_messages += 1
            self._qualities[quality] += 1
            self._sources[source] += 1

    def odometry_callback(self, message):
        pose = message.pose.pose
        position = pose.position
        orientation = pose.orientation
        sample = (
            message.header.timestamp_sec,
            position.x,
            position.y,
            position.z,
            quaternion_to_yaw(
                orientation.qx, orientation.qy, orientation.qz,
                orientation.qw),
        )
        if not all(math.isfinite(value) for value in sample):
            print("WARNING: non-finite localization pose: {}".format(sample),
                  flush=True)
            return
        with self._lock:
            self._poses.append(sample)

    def snapshot(self):
        with self._lock:
            return (self._status_messages, self._qualities.copy(),
                    self._sources.copy(), list(self._poses))


def print_pose(label, sample):
    timestamp, x, y, z, yaw = sample
    print(("{}: timestamp={:.9f} position=({:.4f},{:.4f},{:.4f}) "
           "yaw_deg={:.3f}").format(
               label, timestamp, x, y, z, math.degrees(yaw)), flush=True)


def main():
    args = parse_args()
    if not math.isfinite(args.duration) or not 0.0 < args.duration <= 120.0:
        raise ValueError("duration must be finite and in (0, 120]")

    map_dir, map_details = validate_map(args.map_dir)
    print("map_dir={}".format(map_dir), flush=True)
    print("required map files:", flush=True)
    for name, size, line_count in map_details:
        print("  {}: {} bytes, {} records".format(
            name, size, line_count), flush=True)

    cyber.init()
    node = cyber.Node(
        "yunle_indoor_localization_diagnostic_{}".format(os.getpid()))
    monitor = LocalizationMonitor()
    status_reader = node.create_reader(
        args.status_topic, localization_status_pb2.LocalizationStatus,
        monitor.status_callback)
    odometry_reader = node.create_reader(
        args.odometry_topic, odometry_pb2.Odometry,
        monitor.odometry_callback)
    if status_reader is None or odometry_reader is None:
        raise RuntimeError("failed to create typed localization readers")

    print(("Listening for fixed-map localization on {} and {} for "
           "{:.1f}s...").format(
               args.status_topic, args.odometry_topic, args.duration),
          flush=True)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        time.sleep(0.05)

    status_messages, qualities, sources, poses = monitor.snapshot()
    print("status_messages={} qualities={} sources={}".format(
        status_messages, dict(qualities), dict(sources)), flush=True)
    print("odometry_messages={}".format(len(poses)), flush=True)

    if poses:
        first = poses[0]
        last = poses[-1]
        elapsed = last[0] - first[0]
        rate = ((len(poses) - 1) / elapsed
                if len(poses) > 1 and elapsed > 0.0 else 0.0)
        dx = last[1] - first[1]
        dy = last[2] - first[2]
        dz = last[3] - first[3]
        yaw_delta = normalize_angle(last[4] - first[4])
        max_planar_radius = max(
            math.hypot(sample[1] - first[1], sample[2] - first[2])
            for sample in poses)
        print("odometry_rate={:.3f}Hz timestamp_span={:.3f}s".format(
            rate, max(elapsed, 0.0)), flush=True)
        print_pose("first", first)
        print_pose("last ", last)
        print(("stationary_delta: dx={:.4f}m dy={:.4f}m dz={:.4f}m "
               "planar={:.4f}m yaw={:.3f}deg "
               "max_planar_radius={:.4f}m").format(
                   dx, dy, dz, math.hypot(dx, dy),
                   math.degrees(yaw_delta), max_planar_radius), flush=True)

    acceptable = (
        qualities.get("SLAM_LOCAL_LIDAR_VERY_GOOD", 0) +
        qualities.get("SLAM_LOCAL_LIDAR_GOOD", 0) +
        qualities.get("SLAM_LOCAL_LIDAR_NOT_BAD", 0))
    if not poses or not status_messages:
        print("RESULT: INCOMPLETE -- localization topics are not both active.",
              flush=True)
        exit_code = 2
    elif acceptable <= 0:
        print(("RESULT: FAILED -- localization produced no acceptable "
               "fixed-map match."), flush=True)
        exit_code = 3
    else:
        print(("RESULT: FIXED-MAP LOCALIZATION ACTIVE -- saved map files are "
               "complete and slam_local reports an acceptable match."),
              flush=True)
        exit_code = 0

    cyber.shutdown()
    os._exit(exit_code)


if __name__ == "__main__":
    main()
