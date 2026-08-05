#!/usr/bin/env python3

"""Verify MapCreator can decode indoor LIORF slamStatus messages."""

import argparse
import collections
import math
import os
import threading
import time

from cyber.python.cyber_py3 import cyber
from modules.loam_velodyne.proto import slamStatus_pb2


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Read indoor LIORF status using MapCreator's standard "
            "apollo.loam_velodyne.slamStatus protobuf type."))
    parser.add_argument("--topic", default="liorf/mapping/slam_status")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="observation time in seconds, in (0, 30]")
    return parser.parse_args()


class StatusMonitor(object):

    def __init__(self):
        self._lock = threading.Lock()
        self._samples = []

    def callback(self, message):
        sample = (
            message.lidar_slam_quality,
            message.slam_iter_count_number,
            message.slam_matched_lidar_points,
            message.slam_pc_icp_score,
        )
        with self._lock:
            self._samples.append(sample)

    def snapshot(self):
        with self._lock:
            return list(self._samples)


def quality_name(value):
    try:
        return slamStatus_pb2.LidarSlamQuality.Name(value)
    except ValueError:
        return "UNKNOWN_{}".format(value)


def main():
    args = parse_args()
    if not math.isfinite(args.duration) or not 0.0 < args.duration <= 30.0:
        raise ValueError("duration must be finite and in (0, 30]")

    cyber.init()
    node = cyber.Node(
        "yunle_slam_status_compatibility_diagnostic_{}".format(os.getpid()))
    monitor = StatusMonitor()
    reader = node.create_reader(args.topic, slamStatus_pb2.slamStatus,
                                monitor.callback)
    if reader is None:
        raise RuntimeError(
            "failed to create standard typed reader on {}".format(
                args.topic))

    print(("Listening on {} with MapCreator's standard "
           "apollo.loam_velodyne.slamStatus type for {:.1f}s...").format(
               args.topic, args.duration), flush=True)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        time.sleep(0.05)

    samples = monitor.snapshot()
    if not samples:
        print(("RESULT: INCOMPATIBLE -- no status message was decoded by the "
               "standard typed reader."), flush=True)
        cyber.shutdown()
        os._exit(2)

    invalid_scores = sum(
        1 for sample in samples if not math.isfinite(sample[3]))
    qualities = collections.Counter(
        quality_name(sample[0]) for sample in samples)
    iterations = [sample[1] for sample in samples]
    matched_points = [sample[2] for sample in samples]
    scores = [sample[3] for sample in samples if math.isfinite(sample[3])]
    last = samples[-1]

    print("messages={} qualities={}".format(
        len(samples), dict(sorted(qualities.items()))), flush=True)
    print("iteration_range={}..{} matched_points_range={}..{}".format(
        min(iterations), max(iterations), min(matched_points),
        max(matched_points)), flush=True)
    if scores:
        print("icp_score_range={:.6f}..{:.6f} invalid_scores={}".format(
            min(scores), max(scores), invalid_scores), flush=True)
    else:
        print("icp_score_range=none invalid_scores={}".format(
            invalid_scores), flush=True)
    print(("last: quality={} iteration={} matched_points={} "
           "icp_score={:.6f}").format(
               quality_name(last[0]), last[1], last[2], last[3]),
          flush=True)
    print(("RESULT: COMPATIBLE -- MapCreator's standard protobuf reader "
           "decoded the indoor LIORF status payload."), flush=True)

    cyber.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
