#!/usr/bin/env python3

"""Measure Yunle indoor LIORF odometry without publishing any messages."""

import argparse
import math
import os
import threading
import time

from cyber.python.cyber_py3 import cyber
from modules.loam_velodyne_indoor.proto import odometry_pb2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Summarize typed LIORF indoor odometry messages.")
    parser.add_argument("--topic", default="liorf/mapping/odometry",
                        help="indoor LIORF odometry channel")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="observation time in seconds, in (0, 120]")
    parser.add_argument("--max-samples", type=int, default=5,
                        help="maximum first/last pose samples to print")
    return parser.parse_args()


def quaternion_to_yaw(qx, qy, qz, qw):
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OdometryMonitor(object):

    def __init__(self):
        self._lock = threading.Lock()
        self._samples = []

    def callback(self, message):
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
            print("WARNING: non-finite odometry sample: {}".format(sample),
                  flush=True)
            return
        with self._lock:
            self._samples.append(sample)

    def snapshot(self):
        with self._lock:
            return list(self._samples)


def print_sample(label, sample):
    timestamp, x, y, z, yaw = sample
    print(("{}: timestamp={:.9f} position=({:.4f},{:.4f},{:.4f}) "
           "yaw_deg={:.3f}").format(
               label, timestamp, x, y, z, math.degrees(yaw)), flush=True)


def main():
    args = parse_args()
    if not math.isfinite(args.duration) or not 0.0 < args.duration <= 120.0:
        raise ValueError("duration must be finite and in (0, 120]")
    if args.max_samples < 0 or args.max_samples > 20:
        raise ValueError("max-samples must be in [0, 20]")

    cyber.init()
    node = cyber.Node(
        "yunle_indoor_odometry_diagnostic_{}".format(os.getpid()))
    monitor = OdometryMonitor()
    reader = node.create_reader(args.topic, odometry_pb2.Odometry,
                                monitor.callback)
    if reader is None:
        raise RuntimeError(
            "failed to create typed reader on {}".format(args.topic))

    print("Listening on {} for typed indoor Odometry for {:.1f}s...".format(
        args.topic, args.duration), flush=True)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        time.sleep(0.05)

    samples = monitor.snapshot()
    if not samples:
        print("RESULT: no typed indoor odometry was received.", flush=True)
        cyber.shutdown()
        os._exit(2)

    first = samples[0]
    last = samples[-1]
    elapsed = last[0] - first[0]
    rate = ((len(samples) - 1) / elapsed
            if len(samples) > 1 and elapsed > 0.0 else 0.0)

    dx = last[1] - first[1]
    dy = last[2] - first[2]
    dz = last[3] - first[3]
    planar_displacement = math.hypot(dx, dy)
    displacement_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw_delta = normalize_angle(last[4] - first[4])

    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    zs = [sample[3] for sample in samples]
    yaws = [normalize_angle(sample[4] - first[4]) for sample in samples]
    max_planar_radius = max(
        math.hypot(sample[1] - first[1], sample[2] - first[2])
        for sample in samples)

    print("messages={} timestamp_span={:.3f}s average_rate={:.3f}Hz".format(
        len(samples), max(elapsed, 0.0), rate), flush=True)
    print_sample("first", first)
    print_sample("last ", last)
    print(("end_delta: dx={:.4f}m dy={:.4f}m dz={:.4f}m "
           "planar={:.4f}m distance_3d={:.4f}m yaw={:.3f}deg").format(
               dx, dy, dz, planar_displacement, displacement_3d,
               math.degrees(yaw_delta)), flush=True)
    print(("observed_span: x={:.4f}m y={:.4f}m z={:.4f}m "
           "yaw={:.3f}deg max_planar_radius_from_start={:.4f}m").format(
               max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs),
               math.degrees(max(yaws) - min(yaws)), max_planar_radius),
          flush=True)

    if args.max_samples:
        count = min(args.max_samples, len(samples))
        print("first {} samples:".format(count), flush=True)
        for index, sample in enumerate(samples[:count], 1):
            print_sample("  {:02d}".format(index), sample)
        if len(samples) > count:
            print("last {} samples:".format(count), flush=True)
            for index, sample in enumerate(samples[-count:], 1):
                print_sample("  {:02d}".format(index), sample)

    print("RESULT: typed indoor odometry was received and summarized.",
          flush=True)
    cyber.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
