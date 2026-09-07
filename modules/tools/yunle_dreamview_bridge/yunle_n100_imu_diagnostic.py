#!/usr/bin/env python3

"""Summarize Apollo IMU topics produced by the Yunle N100 driver."""

import argparse
import math
import os
import threading
import time

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.localization_msgs import imu_pb2 as corrected_imu_pb2
from modules.common_msgs.sensor_msgs import imu_pb2 as raw_imu_pb2


RAW_IMU_TOPIC = "/apollo/sensor/gnss/imu"
CORRECTED_IMU_TOPIC = "/apollo/sensor/gnss/corrected_imu"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check N100 raw/corrected Apollo IMU topic rates.")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="observation time in seconds, in (0, 120]")
    parser.add_argument("--raw-topic", default=RAW_IMU_TOPIC)
    parser.add_argument("--corrected-topic", default=CORRECTED_IMU_TOPIC)
    parser.add_argument("--require-corrected", action="store_true",
                        help="fail if corrected_imu is not received")
    return parser.parse_args()


def norm3(point):
    return math.sqrt(point.x * point.x + point.y * point.y + point.z * point.z)


def finite3(point):
    return (math.isfinite(point.x) and math.isfinite(point.y) and
            math.isfinite(point.z))


def rate(samples):
    if len(samples) < 2:
        return 0.0
    span = samples[-1][0] - samples[0][0]
    return ((len(samples) - 1) / span
            if math.isfinite(span) and span > 0.0 else 0.0)


def average(values):
    return sum(values) / len(values) if values else float("nan")


def quaternion_to_yaw(q):
    norm = math.sqrt(q.qw * q.qw + q.qx * q.qx + q.qy * q.qy + q.qz * q.qz)
    if norm <= 0.0 or not math.isfinite(norm):
        return float("nan")
    qw = q.qw / norm
    qx = q.qx / norm
    qy = q.qy / norm
    qz = q.qz / norm
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


class ImuMonitor(object):

    def __init__(self):
        self._lock = threading.Lock()
        self._raw = []
        self._corrected = []

    def raw_callback(self, message):
        if (not message.HasField("linear_acceleration") or
                not message.HasField("angular_velocity")):
            return
        if (not finite3(message.linear_acceleration) or
                not finite3(message.angular_velocity)):
            return
        timestamp = message.header.timestamp_sec
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            timestamp = time.time()
        sample = (
            timestamp,
            norm3(message.linear_acceleration),
            norm3(message.angular_velocity),
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        with self._lock:
            self._raw.append(sample)

    def corrected_callback(self, message):
        if (not message.HasField("imu") or
                not message.imu.HasField("linear_acceleration") or
                not message.imu.HasField("angular_velocity")):
            return
        if (not finite3(message.imu.linear_acceleration) or
                not finite3(message.imu.angular_velocity)):
            return
        timestamp = message.header.timestamp_sec
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            timestamp = time.time()
        yaw = (quaternion_to_yaw(message.imu.orientation)
               if message.imu.HasField("orientation") else float("nan"))
        sample = (
            timestamp,
            norm3(message.imu.linear_acceleration),
            norm3(message.imu.angular_velocity),
            yaw,
        )
        with self._lock:
            self._corrected.append(sample)

    def snapshot(self):
        with self._lock:
            return list(self._raw), list(self._corrected)


def print_raw_summary(samples):
    print("raw_imu_messages={}".format(len(samples)), flush=True)
    if not samples:
        return
    acc_norms = [sample[1] for sample in samples]
    gyro_norms = [sample[2] for sample in samples]
    last = samples[-1]
    print("raw_imu_rate={:.3f}Hz".format(rate(samples)), flush=True)
    print(("raw_acc_norm: mean={:.4f} min={:.4f} max={:.4f} m/s^2 "
           "last_xyz=({:.4f},{:.4f},{:.4f})").format(
               average(acc_norms), min(acc_norms), max(acc_norms),
               last[3], last[4], last[5]), flush=True)
    print(("raw_gyro_norm: mean={:.6f} max={:.6f} rad/s "
           "last_xyz=({:.6f},{:.6f},{:.6f})").format(
               average(gyro_norms), max(gyro_norms),
               last[6], last[7], last[8]), flush=True)


def print_corrected_summary(samples):
    print("corrected_imu_messages={}".format(len(samples)), flush=True)
    if not samples:
        return
    yaws = [sample[3] for sample in samples if math.isfinite(sample[3])]
    print("corrected_imu_rate={:.3f}Hz".format(rate(samples)), flush=True)
    if yaws:
        print("corrected_yaw: first={:.3f}deg last={:.3f}deg span={:.3f}deg".
              format(math.degrees(yaws[0]), math.degrees(yaws[-1]),
                     math.degrees(max(yaws) - min(yaws))), flush=True)


def main():
    args = parse_args()
    if not math.isfinite(args.duration) or not 0.0 < args.duration <= 120.0:
        raise ValueError("duration must be finite and in (0, 120]")

    cyber.init()
    node = cyber.Node("yunle_n100_imu_diagnostic_{}".format(os.getpid()))
    monitor = ImuMonitor()
    raw_reader = node.create_reader(
        args.raw_topic, raw_imu_pb2.Imu, monitor.raw_callback)
    corrected_reader = node.create_reader(
        args.corrected_topic, corrected_imu_pb2.CorrectedImu,
        monitor.corrected_callback)
    if raw_reader is None or corrected_reader is None:
        raise RuntimeError("failed to create typed IMU readers")

    print("Listening on {} and {} for {:.1f}s...".format(
        args.raw_topic, args.corrected_topic, args.duration), flush=True)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        time.sleep(0.05)

    raw, corrected = monitor.snapshot()
    print_raw_summary(raw)
    print_corrected_summary(corrected)

    if not raw:
        print("RESULT: FAIL -- no raw N100 Apollo IMU was received.",
              flush=True)
        exit_code = 2
    elif args.require_corrected and not corrected:
        print("RESULT: FAIL -- raw IMU exists but corrected_imu is missing.",
              flush=True)
        exit_code = 3
    else:
        if corrected:
            print("RESULT: PASS -- raw and corrected N100 IMU are active.",
                  flush=True)
        else:
            print("RESULT: PASS -- raw N100 IMU is active.", flush=True)
        exit_code = 0

    cyber.shutdown()
    os._exit(exit_code)


if __name__ == "__main__":
    main()
