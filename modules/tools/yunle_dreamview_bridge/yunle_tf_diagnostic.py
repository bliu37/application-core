#!/usr/bin/env python3

"""Inspect Apollo transform messages with their concrete protobuf type.

This avoids ``cyber_channel echo`` ambiguity when a RawMessage reader from
Cyber Monitor is also present on the transform channel.  It is diagnostic
only: the script creates no writers and changes no vehicle state.
"""

import argparse
import collections
import math
import os
import threading
import time

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.transform_msgs import transform_pb2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read and summarize typed Apollo TF messages.")
    parser.add_argument("--topic", default="/tf",
                        choices=("/tf", "/tf_static"))
    parser.add_argument("--duration", type=float, default=3.0,
                        help="observation time in seconds, in (0, 30]")
    parser.add_argument("--parent", default="localization",
                        help="expected parent frame")
    parser.add_argument("--child", default="imu",
                        help="expected child frame")
    parser.add_argument("--max-samples", type=int, default=5,
                        help="maximum matching transforms to print")
    return parser.parse_args()


class TransformMonitor(object):

    def __init__(self, parent, child, max_samples):
        self._parent = parent
        self._child = child
        self._max_samples = max_samples
        self._lock = threading.Lock()
        self._message_count = 0
        self._transform_count = 0
        self._matching_count = 0
        self._frames = collections.Counter()
        self._samples = []

    def callback(self, message):
        with self._lock:
            self._message_count += 1
            for stamped in message.transforms:
                parent = stamped.header.frame_id
                child = stamped.child_frame_id
                self._transform_count += 1
                self._frames[(parent, child)] += 1
                if parent != self._parent or child != self._child:
                    continue

                self._matching_count += 1
                if len(self._samples) >= self._max_samples:
                    continue
                translation = stamped.transform.translation
                rotation = stamped.transform.rotation
                self._samples.append((
                    stamped.header.timestamp_sec,
                    translation.x,
                    translation.y,
                    translation.z,
                    rotation.qx,
                    rotation.qy,
                    rotation.qz,
                    rotation.qw,
                ))

    def snapshot(self):
        with self._lock:
            return {
                "message_count": self._message_count,
                "transform_count": self._transform_count,
                "matching_count": self._matching_count,
                "frames": list(self._frames.items()),
                "samples": list(self._samples),
            }


def main():
    args = parse_args()
    if not math.isfinite(args.duration) or not 0.0 < args.duration <= 30.0:
        raise ValueError("duration must be finite and in (0, 30]")
    if args.max_samples < 0 or args.max_samples > 20:
        raise ValueError("max-samples must be in [0, 20]")

    cyber.init()
    node = cyber.Node("yunle_tf_diagnostic_{}".format(os.getpid()))
    monitor = TransformMonitor(args.parent, args.child, args.max_samples)
    reader = node.create_reader(
        args.topic, transform_pb2.TransformStampeds, monitor.callback)
    if reader is None:
        raise RuntimeError("failed to create typed reader on {}".format(
            args.topic))

    print("Listening on {} for typed TransformStampeds for {:.1f}s...".format(
        args.topic, args.duration), flush=True)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        time.sleep(0.05)

    result = monitor.snapshot()
    print("messages={} transforms={} target={} -> {} matches={}".format(
        result["message_count"], result["transform_count"], args.parent,
        args.child, result["matching_count"]), flush=True)

    if result["frames"]:
        print("observed frame pairs:", flush=True)
        for (parent, child), count in sorted(result["frames"]):
            print("  {} -> {}: {}".format(parent, child, count), flush=True)
    else:
        print("observed frame pairs: none", flush=True)

    for index, sample in enumerate(result["samples"], 1):
        print(("target sample {}: timestamp={:.9f} "
               "translation=({:.6f},{:.6f},{:.6f}) "
               "rotation=({:.6f},{:.6f},{:.6f},{:.6f})").format(
                   index, *sample), flush=True)

    if result["matching_count"] == 0:
        print("RESULT: target transform was NOT received.", flush=True)
        exit_code = 2
    else:
        print("RESULT: target transform was received by a typed reader.",
              flush=True)
        exit_code = 0

    cyber.shutdown()
    os._exit(exit_code)


if __name__ == "__main__":
    main()
