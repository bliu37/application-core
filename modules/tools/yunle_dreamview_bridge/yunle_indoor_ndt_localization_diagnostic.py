#!/usr/bin/env python3

"""Summarize Yunle indoor CPU-NDT fixed-map localization output."""

import argparse
import glob
import math
import os
from pathlib import Path
import re
import sys
import threading
import time

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.localization_msgs import gps_pb2
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.sensor_msgs import pointcloud_pb2
from modules.loam_velodyne_indoor.proto import odometry_pb2


DEFAULT_NDT_MAP_DIR = Path(
    "/apollo_workspace/data/map_work/yunle_indoor/"
    "jd03_indoor_20260803_01/ndt_map/local_map")
DEFAULT_LIORF_ODOMETRY_TOPIC = "liorf/mapping/odometry"
DEFAULT_NDT_ODOMETRY_TOPIC = "/apollo/yunle/indoor/ndt/odometry"
DEFAULT_NDT_CLOUD_TOPIC = "/apollo/yunle/indoor/ndt/PointCloud2"
DEFAULT_LOCALIZATION_TOPIC = "/apollo/localization/pose"
DEFAULT_NDT_LIDAR_TOPIC = "/apollo/localization/ndt_lidar"
DEFAULT_RAW_LOCALIZATION_TOPIC = "/apollo/yunle/indoor/ndt/raw_pose"
DEFAULT_RAW_NDT_LIDAR_TOPIC = "/apollo/yunle/indoor/ndt/raw_lidar"
DEFAULT_LOG_DIR = Path("/apollo_workspace/data/log")
DEFAULT_LOG_TAIL_BYTES = 8 * 1024 * 1024


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Check the generated NDT map and summarize fixed-map input and "
            "output topics."))
    parser.add_argument("--duration", type=float, default=15.0,
                        help="observation time in seconds, in (0, 120]")
    parser.add_argument("--map-dir", type=Path, default=DEFAULT_NDT_MAP_DIR,
                        help="generated Apollo NDT map directory")
    parser.add_argument("--liorf-odometry-topic",
                        default=DEFAULT_LIORF_ODOMETRY_TOPIC,
                        help="typed indoor LIORF odometry source channel")
    parser.add_argument("--ndt-odometry-topic",
                        default=DEFAULT_NDT_ODOMETRY_TOPIC,
                        help="bridge output odometry channel consumed by NDT")
    parser.add_argument("--ndt-cloud-topic", default=DEFAULT_NDT_CLOUD_TOPIC,
                        help="bridge output point-cloud channel consumed by NDT")
    parser.add_argument("--localization-topic",
                        default=DEFAULT_LOCALIZATION_TOPIC,
                        help="NDT fused localization output channel")
    parser.add_argument("--ndt-lidar-topic", default=DEFAULT_NDT_LIDAR_TOPIC,
                        help="NDT lidar-only localization output channel")
    parser.add_argument("--raw-localization-topic",
                        default=DEFAULT_RAW_LOCALIZATION_TOPIC,
                        help="optional raw NDT fused output before stabilization")
    parser.add_argument("--raw-ndt-lidar-topic",
                        default=DEFAULT_RAW_NDT_LIDAR_TOPIC,
                        help="optional raw NDT lidar output before stabilization")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR,
                        help="Apollo log directory for NDT fitness summary")
    parser.add_argument("--log-tail-bytes", type=int,
                        default=DEFAULT_LOG_TAIL_BYTES,
                        help="bytes to read from the end of each NDT log")
    parser.add_argument("--skip-log-summary", action="store_true",
                        help="do not parse NDTlocalization/mainboard logs")
    return parser.parse_args()


def quaternion_to_euler(qx, qy, qz, qw):
    sin_roll = 2.0 * (qw * qx + qy * qz)
    cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (qw * qy - qz * qx)
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)

    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def percentile(values, percent):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * percent / 100.0
    low = int(math.floor(index))
    high = int(math.ceil(index))
    if low == high:
        return ordered[low]
    return ordered[low] * (high - index) + ordered[high] * (index - low)


def correlation(left, right):
    if len(left) != len(right) or len(left) < 3:
        return float("nan")
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
    left_var = sum((value - left_mean) ** 2 for value in left)
    right_var = sum((value - right_mean) ** 2 for value in right)
    if left_var <= 0.0 or right_var <= 0.0:
        return float("nan")
    cov = sum((x - left_mean) * (y - right_mean)
              for x, y in zip(left, right))
    return cov / math.sqrt(left_var * right_var)


class TopicMonitor(object):

    def __init__(self):
        self._lock = threading.Lock()
        self._poses = {
            "fused": [],
            "lidar": [],
            "raw_fused": [],
            "raw_lidar": [],
        }
        self._odometry = {"liorf": [], "ndt": []}
        self._clouds = []

    def localization_callback(self, source, message):
        if (not message.HasField("pose") or
                not message.pose.HasField("position") or
                not message.pose.HasField("orientation")):
            return
        position = message.pose.position
        orientation = message.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(
            orientation.qx, orientation.qy, orientation.qz, orientation.qw)
        sample = (message.measurement_time, position.x, position.y,
                  position.z, roll, pitch, yaw)
        if not all(math.isfinite(value) for value in sample):
            return
        with self._lock:
            self._poses[source].append(sample)

    def liorf_odometry_callback(self, message):
        if (not message.HasField("header") or not message.HasField("pose") or
                not message.pose.HasField("pose") or
                not message.pose.pose.HasField("position") or
                not message.pose.pose.HasField("orientation")):
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(
            orientation.qx, orientation.qy, orientation.qz, orientation.qw)
        sample = (
            message.header.timestamp_sec,
            position.x,
            position.y,
            position.z,
            roll,
            pitch,
            yaw,
        )
        if not all(math.isfinite(value) for value in sample):
            return
        with self._lock:
            self._odometry["liorf"].append(sample)

    def ndt_odometry_callback(self, message):
        if (not message.HasField("header") or
                not message.HasField("localization") or
                not message.localization.HasField("position") or
                not message.localization.HasField("orientation")):
            return
        position = message.localization.position
        orientation = message.localization.orientation
        roll, pitch, yaw = quaternion_to_euler(
            orientation.qx, orientation.qy, orientation.qz, orientation.qw)
        sample = (
            message.header.timestamp_sec,
            position.x,
            position.y,
            position.z,
            roll,
            pitch,
            yaw,
        )
        if not all(math.isfinite(value) for value in sample):
            return
        with self._lock:
            self._odometry["ndt"].append(sample)

    def ndt_cloud_callback(self, message):
        timestamp = message.measurement_time
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            timestamp = (message.header.timestamp_sec
                         if message.HasField("header") else 0.0)
        sample = (timestamp, len(message.point), message.width,
                  message.height)
        if not math.isfinite(sample[0]):
            return
        with self._lock:
            self._clouds.append(sample)

    def snapshot(self):
        with self._lock:
            return {
                "poses": {key: list(value)
                          for key, value in self._poses.items()},
                "odometry": {key: list(value)
                             for key, value in self._odometry.items()},
                "clouds": list(self._clouds),
            }


def print_pose_summary(label, samples):
    if not samples:
        print("{}: messages=0".format(label))
        return
    first = samples[0]
    last = samples[-1]
    dx = last[1] - first[1]
    dy = last[2] - first[2]
    dz = last[3] - first[3]
    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    zs = [sample[3] for sample in samples]
    rolls = [math.degrees(sample[4]) for sample in samples]
    pitches = [math.degrees(sample[5]) for sample in samples]
    yaws = [math.degrees(sample[6]) for sample in samples]
    elapsed = max(last[0] - first[0], 0.0)
    rate = ((len(samples) - 1) / elapsed
            if len(samples) > 1 and elapsed > 0.0 else 0.0)
    print(("{}: messages={} rate={:.3f}Hz first=({:.4f},{:.4f},{:.4f}) "
           "last=({:.4f},{:.4f},{:.4f}) planar_delta={:.4f}m "
           "z_delta={:.4f}m span_x={:.4f}m span_y={:.4f}m span_z={:.4f}m "
           "rpy_first=({:.2f},{:.2f},{:.2f})deg "
           "rpy_last=({:.2f},{:.2f},{:.2f})deg "
           "span_rpy=({:.2f},{:.2f},{:.2f})deg").format(
               label, len(samples), rate, first[1], first[2], first[3],
               last[1], last[2], last[3], math.hypot(dx, dy),
               dz, max(xs) - min(xs), max(ys) - min(ys),
               max(zs) - min(zs), rolls[0], pitches[0], yaws[0],
               rolls[-1], pitches[-1], yaws[-1], max(rolls) - min(rolls),
               max(pitches) - min(pitches), max(yaws) - min(yaws)))


def print_cloud_summary(label, samples):
    if not samples:
        print("{}: messages=0".format(label))
        return
    first = samples[0]
    last = samples[-1]
    elapsed = max(last[0] - first[0], 0.0)
    rate = ((len(samples) - 1) / elapsed
            if len(samples) > 1 and elapsed > 0.0 else 0.0)
    point_counts = [sample[1] for sample in samples]
    print(("{}: messages={} rate={:.3f}Hz first_time={:.9f} "
           "last_time={:.9f} points={}..{} last_shape={}x{}").format(
               label, len(samples), rate, first[0], last[0],
               min(point_counts), max(point_counts), last[2], last[3]))


def latest_ndt_logs(log_dir):
    ndt_logs = sorted(
        glob.glob(str(log_dir / "NDTlocalization.log.INFO.*")),
        key=os.path.getmtime)
    if not ndt_logs:
        return None, None
    ndt_log = Path(ndt_logs[-1])
    pid = ndt_log.name.rsplit(".", 1)[-1]
    mainboard_logs = sorted(
        glob.glob(str(log_dir / "mainboard.log.INFO.*.{}".format(pid))),
        key=os.path.getmtime)
    mainboard_log = Path(mainboard_logs[-1]) if mainboard_logs else None
    return ndt_log, mainboard_log


def parse_pose_debug_line(line, pose_re):
    match = pose_re.search(line)
    if not match:
        return None
    values = [float(value) for value in match.groups()]
    roll, pitch, yaw = quaternion_to_euler(
        values[4], values[5], values[6], values[7])
    return values[:4] + [roll, pitch, yaw]


def iter_log_tail_lines(log_path, max_bytes):
    if log_path is None:
        return []
    try:
        file_size = log_path.stat().st_size
    except OSError:
        return []
    offset = max(file_size - max(max_bytes, 0), 0)
    with log_path.open("rb") as log_file:
        log_file.seek(offset)
        data = log_file.read()
    text = data.decode("utf-8", errors="replace")
    lines = text.splitlines()
    if offset > 0 and lines:
        return lines[1:]
    return lines


def parse_ndt_debug_log(ndt_log, log_tail_bytes):
    number = r"[-+0-9.eE]+"
    pose_re = re.compile(
        r"time: ({0}), x: ({0}), y: ({0}), z: ({0}), qx: ({0}), "
        r"qy: ({0}), qz: ({0}), qw: ({0})".format(number))
    lidar_poses = []
    odometry_poses = []
    for line in iter_log_tail_lines(ndt_log, log_tail_bytes):
        if "NDTLocalization Debug Log: odometry for lidar pose:" in line:
            pose = parse_pose_debug_line(line, pose_re)
            if pose is not None:
                odometry_poses.append(pose)
        elif "NDTLocalization Debug Log: lidar pose:" in line:
            pose = parse_pose_debug_line(line, pose_re)
            if pose is not None:
                lidar_poses.append(pose)
    return lidar_poses, odometry_poses


def parse_mainboard_ndt_summary(mainboard_log, log_tail_bytes):
    if mainboard_log is None:
        return []
    number = r"[-+0-9.eE]+"
    fitness_re = re.compile(r"Fitness Score: ({})".format(number))
    relative_re = re.compile(
        r"Relative Ndt pose: ({}), ({}), ({})".format(number, number, number))
    iteration_re = re.compile(r"Iteration: %d: ([0-9]+)")
    summaries = []
    current = None
    for line in iter_log_tail_lines(mainboard_log, log_tail_bytes):
        fitness_match = fitness_re.search(line)
        if fitness_match:
            if current is not None:
                summaries.append(current)
            current = {"fitness": float(fitness_match.group(1))}
            continue
        if current is None:
            continue
        relative_match = relative_re.search(line)
        if relative_match:
            current["relative"] = tuple(
                float(value) for value in relative_match.groups())
            continue
        iteration_match = iteration_re.search(line)
        if iteration_match:
            current["iterations"] = int(iteration_match.group(1))
            continue
    if current is not None:
        summaries.append(current)
    return summaries


def print_ndt_log_summary(log_dir, start_time, end_time, log_tail_bytes):
    ndt_log, mainboard_log = latest_ndt_logs(log_dir)
    if ndt_log is None:
        print("ndt_log_summary: no NDTlocalization log found")
        return
    lidar_poses, odometry_poses = parse_ndt_debug_log(ndt_log, log_tail_bytes)
    summaries = parse_mainboard_ndt_summary(mainboard_log, log_tail_bytes)
    count = min(len(lidar_poses), len(odometry_poses), len(summaries))
    selected = [
        index for index in range(count)
        if start_time - 1.0 <= lidar_poses[index][0] <= end_time + 1.0
    ]
    if not selected:
        print(("ndt_log_summary: no log samples matched diagnostic window "
               "latest_ndt_log={} log_tail_bytes={}").format(
                   ndt_log, log_tail_bytes))
        return

    lidar_z = [lidar_poses[index][3] for index in selected]
    odometry_z = [odometry_poses[index][3] for index in selected]
    fitness = [summaries[index].get("fitness", float("nan"))
               for index in selected]
    relative_z = [
        summaries[index].get("relative", (0.0, 0.0, float("nan")))[2]
        for index in selected
    ]
    iterations = [summaries[index].get("iterations", 0)
                  for index in selected]
    valid_pairs = [
        index for index in range(len(selected))
        if math.isfinite(fitness[index]) and math.isfinite(relative_z[index])
    ]
    fitness_for_corr = [fitness[index] for index in valid_pairs]
    z_for_corr = [lidar_z[index] for index in valid_pairs]
    relative_z_for_corr = [relative_z[index] for index in valid_pairs]
    print(("ndt_log_summary: latest_ndt_log={} latest_mainboard_log={} "
           "matched_samples={} lidar_z_first={:.4f} lidar_z_last={:.4f} "
           "lidar_z_span={:.4f}m odom_z_span={:.4f}m "
           "fitness_min/med/p95/max={:.4f}/{:.4f}/{:.4f}/{:.4f} "
           "fitness_gt_0.5={}/{} rel_z_abs_gt_0.5={}/{} "
           "iter_med/max={:.1f}/{} corr_z_fitness={:.3f} "
           "corr_z_relz={:.3f}").format(
               ndt_log, mainboard_log, len(selected), lidar_z[0],
               lidar_z[-1], max(lidar_z) - min(lidar_z),
               max(odometry_z) - min(odometry_z), min(fitness),
               percentile(fitness, 50.0), percentile(fitness, 95.0),
               max(fitness), sum(value > 0.5 for value in fitness),
               len(fitness), sum(abs(value) > 0.5 for value in relative_z),
               len(relative_z), percentile(iterations, 50.0),
               max(iterations), correlation(z_for_corr, fitness_for_corr),
               correlation(z_for_corr, relative_z_for_corr)))


def print_cross_checks(samples):
    ndt_inputs = samples["odometry"]["ndt"]
    fused = samples["poses"]["fused"]
    lidar = samples["poses"]["lidar"]
    if ndt_inputs and fused:
        first_input = ndt_inputs[0]
        last_input = ndt_inputs[-1]
        first_fused = fused[0]
        last_fused = fused[-1]
        print(("z_input_output_check: ndt_input_z_delta={:.4f}m "
               "localization_z_delta={:.4f}m "
               "localization_minus_input_z_first={:.4f}m "
               "localization_minus_input_z_last={:.4f}m").format(
                   last_input[3] - first_input[3],
                   last_fused[3] - first_fused[3],
                   first_fused[3] - first_input[3],
                   last_fused[3] - last_input[3]))
    if ndt_inputs and lidar:
        first_input = ndt_inputs[0]
        last_input = ndt_inputs[-1]
        first_lidar = lidar[0]
        last_lidar = lidar[-1]
        print(("z_ndt_lidar_check: ndt_input_z_delta={:.4f}m "
               "ndt_lidar_z_delta={:.4f}m "
               "ndt_lidar_minus_input_z_first={:.4f}m "
               "ndt_lidar_minus_input_z_last={:.4f}m").format(
                   last_input[3] - first_input[3],
                   last_lidar[3] - first_lidar[3],
                   first_lidar[3] - first_input[3],
                   last_lidar[3] - last_input[3]))

    print_raw_final_pair_check(
        "raw_final_localization_check",
        samples["poses"]["raw_fused"],
        samples["poses"]["fused"])
    print_raw_final_pair_check(
        "raw_final_ndt_lidar_check",
        samples["poses"]["raw_lidar"],
        samples["poses"]["lidar"])
    print_output_vs_input_check(
        "localization_vs_input_check", ndt_inputs, fused)
    print_output_vs_input_check(
        "ndt_lidar_vs_input_check", ndt_inputs, lidar)
    print_output_vs_input_check(
        "raw_localization_vs_input_check",
        ndt_inputs,
        samples["poses"]["raw_fused"])
    print_output_vs_input_check(
        "raw_ndt_lidar_vs_input_check",
        ndt_inputs,
        samples["poses"]["raw_lidar"])


def print_raw_final_pair_check(label, raw_samples, final_samples):
    count = min(len(raw_samples), len(final_samples))
    if count == 0:
        return
    xy_diffs = []
    z_raw_minus_final = []
    yaw_diffs = []
    for raw_sample, final_sample in zip(raw_samples[:count],
                                        final_samples[:count]):
        xy_diffs.append(math.hypot(raw_sample[1] - final_sample[1],
                                   raw_sample[2] - final_sample[2]))
        z_raw_minus_final.append(raw_sample[3] - final_sample[3])
        yaw_diffs.append(math.degrees(raw_sample[6] - final_sample[6]))
    print(("{}: paired_samples={} max_xy_diff={:.4f}m "
           "z_raw_minus_final_first={:.4f}m "
           "z_raw_minus_final_last={:.4f}m "
           "z_raw_minus_final_span={:.4f}m "
           "max_abs_yaw_diff={:.4f}deg").format(
               label, count, max(xy_diffs), z_raw_minus_final[0],
               z_raw_minus_final[-1],
               max(z_raw_minus_final) - min(z_raw_minus_final),
               max(abs(value) for value in yaw_diffs)))


def angle_delta(left, right):
    diff = left - right
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def nearest_sample(timestamp, samples):
    if not samples:
        return None, float("inf")
    nearest = min(samples, key=lambda sample: abs(sample[0] - timestamp))
    return nearest, abs(nearest[0] - timestamp)


def print_output_vs_input_check(label, input_samples, output_samples):
    if not input_samples or not output_samples:
        return
    pairs = []
    for output in output_samples:
        input_sample, time_diff = nearest_sample(output[0], input_samples)
        if input_sample is not None and time_diff <= 0.50:
            pairs.append((input_sample, output, time_diff))
    if len(pairs) < 2:
        return

    first_input, first_output, _ = pairs[0]
    last_input, last_output, _ = pairs[-1]
    first_offset_x = first_output[1] - first_input[1]
    first_offset_y = first_output[2] - first_input[2]
    last_offset_x = last_output[1] - last_input[1]
    last_offset_y = last_output[2] - last_input[2]
    input_dx = last_input[1] - first_input[1]
    input_dy = last_input[2] - first_input[2]
    output_dx = last_output[1] - first_output[1]
    output_dy = last_output[2] - first_output[2]
    yaw_offset_first = math.degrees(
        angle_delta(first_output[6], first_input[6]))
    yaw_offset_last = math.degrees(
        angle_delta(last_output[6], last_input[6]))
    yaw_delta_error = math.degrees(
        angle_delta(last_output[6] - first_output[6],
                    last_input[6] - first_input[6]))
    xy_offsets = [
        math.hypot(output[1] - input_sample[1],
                   output[2] - input_sample[2])
        for input_sample, output, _ in pairs
    ]
    time_diffs = [time_diff for _, _, time_diff in pairs]
    print(("{}: paired_samples={} max_time_diff={:.3f}s "
           "offset_first=({:+.4f},{:+.4f})m "
           "offset_last=({:+.4f},{:+.4f})m "
           "offset_delta_planar={:.4f}m max_offset_planar={:.4f}m "
           "delta_error=({:+.4f},{:+.4f})m "
           "delta_error_planar={:.4f}m "
           "yaw_offset_first/last={:+.3f}/{:+.3f}deg "
           "yaw_delta_error={:+.3f}deg").format(
               label, len(pairs), max(time_diffs),
               first_offset_x, first_offset_y, last_offset_x, last_offset_y,
               math.hypot(last_offset_x - first_offset_x,
                          last_offset_y - first_offset_y),
               max(xy_offsets), output_dx - input_dx, output_dy - input_dy,
               math.hypot(output_dx - input_dx, output_dy - input_dy),
               yaw_offset_first, yaw_offset_last, yaw_delta_error))


def main():
    args = parse_args()
    if not math.isfinite(args.duration) or not 0.0 < args.duration <= 120.0:
        raise ValueError("duration must be finite and in (0, 120]")

    map_dir = args.map_dir.resolve()
    config = map_dir / "config.xml"
    map_files = ([path for path in map_dir.rglob("*") if path.is_file()]
                 if map_dir.is_dir() else [])
    print("ndt_map_dir={}".format(map_dir))
    print("config_xml={} map_files={}".format(config.is_file(), len(map_files)))
    if not config.is_file() or len(map_files) < 2:
        print("RESULT: INCOMPLETE -- generate the NDT map before localization.")
        return 2

    cyber.init()
    node = cyber.Node(
        "yunle_indoor_ndt_localization_diagnostic_{}".format(os.getpid()))
    monitor = TopicMonitor()
    readers = [
        node.create_reader(
            args.localization_topic,
            localization_pb2.LocalizationEstimate,
            lambda message: monitor.localization_callback("fused", message)),
        node.create_reader(
            args.ndt_lidar_topic,
            localization_pb2.LocalizationEstimate,
            lambda message: monitor.localization_callback("lidar", message)),
        node.create_reader(
            args.raw_localization_topic,
            localization_pb2.LocalizationEstimate,
            lambda message: monitor.localization_callback(
                "raw_fused", message)),
        node.create_reader(
            args.raw_ndt_lidar_topic,
            localization_pb2.LocalizationEstimate,
            lambda message: monitor.localization_callback(
                "raw_lidar", message)),
        node.create_reader(
            args.liorf_odometry_topic,
            odometry_pb2.Odometry,
            monitor.liorf_odometry_callback),
        node.create_reader(
            args.ndt_odometry_topic,
            gps_pb2.Gps,
            monitor.ndt_odometry_callback),
        node.create_reader(
            args.ndt_cloud_topic,
            pointcloud_pb2.PointCloud,
            monitor.ndt_cloud_callback),
    ]
    if any(reader is None for reader in readers):
        raise RuntimeError("failed to create one or more diagnostic readers")

    print("Listening for CPU-NDT fixed-map localization for {:.1f}s...".format(
        args.duration), flush=True)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        time.sleep(0.05)

    samples = monitor.snapshot()
    print_pose_summary("liorf_odometry", samples["odometry"]["liorf"])
    print_pose_summary("bridge_ndt_odometry", samples["odometry"]["ndt"])
    print_cloud_summary("bridge_ndt_cloud", samples["clouds"])
    print_pose_summary("localization_pose", samples["poses"]["fused"])
    print_pose_summary("ndt_lidar_pose", samples["poses"]["lidar"])
    print_pose_summary("raw_localization_pose", samples["poses"]["raw_fused"])
    print_pose_summary("raw_ndt_lidar_pose", samples["poses"]["raw_lidar"])
    print_cross_checks(samples)

    log_window_samples = (
        samples["poses"]["lidar"] or samples["poses"]["fused"] or
        samples["odometry"]["ndt"])
    if not args.skip_log_summary and log_window_samples:
        print_ndt_log_summary(
            args.log_dir.resolve(),
            log_window_samples[0][0],
            log_window_samples[-1][0],
            args.log_tail_bytes)

    valid_lidar = [sample for sample in samples["poses"]["lidar"]
                   if abs(sample[1]) > 1000.0 and abs(sample[2]) > 1000.0]
    if not samples["odometry"]["liorf"]:
        print(("RESULT: INCOMPLETE -- IndoorLiorfOdometry is not publishing "
               "typed indoor odometry on {}.").format(
                   args.liorf_odometry_topic))
        result = 2
    elif not samples["odometry"]["ndt"]:
        print(("RESULT: INCOMPLETE -- YunleIndoorNdtInputBridge is not "
               "publishing NDT odometry on {}.").format(
                   args.ndt_odometry_topic))
        result = 2
    elif not samples["clouds"]:
        print(("RESULT: INCOMPLETE -- YunleIndoorNdtInputBridge is not "
               "publishing paired NDT clouds on {}. Check raw cloud/LIORF "
               "timestamp pairing in yunle_dreamview_bridge logs.").format(
                   args.ndt_cloud_topic))
        result = 2
    elif not samples["poses"]["lidar"]:
        print(("RESULT: INCOMPLETE -- CPU-NDT has bridge inputs but has not "
               "published a lidar map-frame pose. Check NDTlocalization logs."))
        result = 2
    elif samples["poses"]["fused"] and valid_lidar:
        print("RESULT: CPU-NDT fixed-map localization topics are active.")
        result = 0
    else:
        print(("RESULT: INCOMPLETE -- CPU-NDT topics are present, but no "
               "valid map-frame lidar pose was observed."))
        result = 2
    sys.stdout.flush()
    sys.stderr.flush()
    return result


if __name__ == "__main__":
    os._exit(main())
