#!/usr/bin/env python3

"""Bounded straight or constant-curvature trajectory publisher for Yunle tests.

This tool intentionally publishes only /apollo/planning.  It does not publish
/apollo/control; Apollo Control remains the producer of ControlCommand.
"""

import argparse
import math
import os
import sys
import threading
import time

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.canbus_vehicle.yunle.proto import yunle_chassis_receiver_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.planning_msgs import planning_pb2


DEFAULT_PLANNING_TOPIC = "/apollo/planning"
LOCALIZATION_TOPIC = "/apollo/localization/pose"
CHASSIS_TOPIC = "/apollo/canbus/chassis"
CONTROL_TOPIC = "/apollo/control"
DETAIL_TOPIC = "/apollo/canbus/yunle_chassis_detail"

PUBLISH_HZ = 10.0
POINT_INTERVAL_SEC = 0.1
TRAJECTORY_HORIZON_SEC = 6.0
MAX_SPEED_KPH = 1.5
MAX_DURATION_SEC = 8.0
STOP_DURATION_SEC = 1.5
MAX_INITIAL_SPEED_KPH = 0.2
DEFAULT_TURN_RADIUS_M = 10.0
MIN_TURN_RADIUS_M = 10.0
MAX_TURN_RADIUS_M = 100.0


class LatestMessage(object):

    def __init__(self, message_type):
        self._message_type = message_type
        self._lock = threading.Lock()
        self._latest = None

    def callback(self, message):
        copied = self._message_type()
        copied.CopyFrom(message)
        with self._lock:
            self._latest = copied

    def snapshot(self):
        with self._lock:
            if self._latest is None:
                return None
            copied = self._message_type()
            copied.CopyFrom(self._latest)
            return copied


def wait_for(label, monitor, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        message = monitor.snapshot()
        if message is not None:
            return message
        time.sleep(0.02)
    raise RuntimeError("timed out waiting for {}".format(label))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish a bounded straight or curved ADCTrajectory.")
    parser.add_argument("--topic", default=DEFAULT_PLANNING_TOPIC,
                        help="ADCTrajectory topic to publish")
    parser.add_argument("--speed-kph", type=float, default=0.0,
                        help="target speed in kph, limited to [0, 1.5]")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="movement planning duration in seconds")
    parser.add_argument("--trajectory-seconds", type=float,
                        default=TRAJECTORY_HORIZON_SEC,
                        help="future trajectory horizon in seconds")
    parser.add_argument("--publish-hz", type=float, default=PUBLISH_HZ)
    parser.add_argument("--turn", choices=("straight", "left", "right"),
                        default="straight",
                        help="trajectory direction; defaults to straight")
    parser.add_argument("--radius-m", type=float,
                        default=DEFAULT_TURN_RADIUS_M,
                        help="turn radius in metres, limited to [10, 100]")
    parser.add_argument("--confirm-stationary-test", action="store_true",
                        required=True,
                        help="confirm vehicle is secured and E-stop is ready")
    return parser.parse_args()


def validate_args(args):
    values = (args.speed_kph, args.duration, args.trajectory_seconds,
              args.publish_hz, args.radius_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("all numeric arguments must be finite")
    if args.speed_kph < 0.0 or args.speed_kph > MAX_SPEED_KPH:
        raise ValueError("speed-kph must be in [0, 1.5]")
    if args.duration <= 0.0 or args.duration > MAX_DURATION_SEC:
        raise ValueError("duration must be in (0, 8]")
    if args.trajectory_seconds < 1.0 or args.trajectory_seconds > 10.0:
        raise ValueError("trajectory-seconds must be in [1, 10]")
    if args.publish_hz < 2.0 or args.publish_hz > 50.0:
        raise ValueError("publish-hz must be in [2, 50]")
    if args.radius_m < MIN_TURN_RADIUS_M or \
            args.radius_m > MAX_TURN_RADIUS_M:
        raise ValueError("radius-m must be in [10, 100]")


def turn_curvature(turn, radius_m):
    if turn == "left":
        return 1.0 / radius_m
    if turn == "right":
        return -1.0 / radius_m
    return 0.0


def chassis_speed_kph(chassis):
    if chassis is None or not chassis.HasField("speed_mps"):
        return float("nan")
    return chassis.speed_mps * 3.6


def gear_name(gear):
    names = {
        chassis_pb2.Chassis.GEAR_NEUTRAL: "NEUTRAL",
        chassis_pb2.Chassis.GEAR_DRIVE: "DRIVE",
        chassis_pb2.Chassis.GEAR_REVERSE: "REVERSE",
        chassis_pb2.Chassis.GEAR_PARKING: "PARKING",
        chassis_pb2.Chassis.GEAR_INVALID: "INVALID",
        chassis_pb2.Chassis.GEAR_NONE: "NONE",
    }
    return names.get(gear, str(gear))


def print_chassis(label, chassis):
    if chassis is None:
        print("{}: no chassis".format(label))
        return
    print("{}: speed_kph={:.3f} gear={} parking={} driving_mode={}".format(
        label, chassis_speed_kph(chassis), gear_name(chassis.gear_location),
        chassis.parking_brake, chassis.driving_mode))


def print_control(label, control):
    if control is None:
        print("{}: no control".format(label))
        return
    status = control.header.status
    print(("{}: error_code={} msg={!r} gear={} speed_mps={:.3f} "
           "steering={:.2f} brake={:.1f} parking={} safe_mode={} "
           "advice={} reason={!r}").format(
               label, status.error_code, status.msg,
               gear_name(control.gear_location), control.speed,
               control.steering_target, control.brake,
               control.parking_brake, control.is_in_safe_mode,
               control.engage_advice.advice,
               control.engage_advice.reason))


def print_detail(label, detail):
    if detail is None:
        print("{}: no Yunle chassis detail".format(label))
        return
    pad_start_required = getattr(detail, "control_pad_start_required", False)
    pad_started = getattr(detail, "control_pad_started", False)
    last_pad_action = getattr(detail, "last_control_pad_action", 0)
    print(("{}: shift={} parking={} speed_kph={:.3f} scu_brake={} "
           "scu_target_kph={:.3f} vehicle_target_kph={:.3f} "
           "cmd_target_kph={:.3f} cmd_steering={:.2f} fresh={} "
           "interlocks={} failsafe={} sent={} errors={} "
           "pad_start_required={} pad_started={} last_pad_action={} "
           "reason={!r}").format(
               label, detail.shift_status, detail.parking_status,
               detail.vehicle_speed_kph, detail.scu_brake,
               detail.scu_target_speed_kph, detail.vehicle_target_speed_kph,
               detail.commanded_target_speed_kph,
               detail.commanded_steering_percentage,
               detail.control_command_fresh, detail.control_interlocks_ok,
               detail.control_failsafe_active,
               detail.sent_control_frame_count,
               detail.control_send_error_count,
               pad_start_required,
               pad_started,
               last_pad_action,
               detail.control_block_reason))


def make_trajectory(sequence_num, localization, speed_kph, horizon_sec,
                    curvature):
    now_sec = cyber_time.Time.now().to_sec()
    speed_mps = speed_kph / 3.6
    pose = localization.pose
    position = pose.position
    heading = pose.heading if pose.HasField("heading") else 0.0
    cos_heading = math.cos(heading)
    sin_heading = math.sin(heading)

    trajectory = planning_pb2.ADCTrajectory()
    trajectory.header.module_name = "yunle_planning_test_publisher"
    trajectory.header.sequence_num = sequence_num
    trajectory.header.timestamp_sec = now_sec
    trajectory.total_path_time = horizon_sec
    trajectory.total_path_length = speed_mps * horizon_sec
    trajectory.estop.is_estop = False
    trajectory.gear = chassis_pb2.Chassis.GEAR_DRIVE
    trajectory.is_replan = False
    trajectory.trajectory_type = planning_pb2.ADCTrajectory.NORMAL

    point_count = int(horizon_sec / POINT_INTERVAL_SEC) + 1
    for index in range(point_count):
        relative_time = index * POINT_INTERVAL_SEC
        station = speed_mps * relative_time
        if curvature == 0.0:
            local_x = station
            local_y = 0.0
        else:
            heading_delta = curvature * station
            local_x = math.sin(heading_delta) / curvature
            local_y = (1.0 - math.cos(heading_delta)) / curvature
        point = trajectory.trajectory_point.add()
        path_point = point.path_point
        path_point.x = (position.x + cos_heading * local_x -
                        sin_heading * local_y)
        path_point.y = (position.y + sin_heading * local_x +
                        cos_heading * local_y)
        path_point.z = position.z if position.HasField("z") else 0.0
        path_point.theta = heading + curvature * station
        path_point.kappa = curvature
        path_point.dkappa = 0.0
        path_point.ddkappa = 0.0
        path_point.s = station
        point.v = speed_mps
        point.a = 0.0
        point.da = 0.0
        point.relative_time = relative_time
    return trajectory


def publish_planning(writer, localization_monitor, sequence_num, speed_kph,
                     duration_sec, horizon_sec, publish_hz, curvature):
    deadline = time.monotonic() + duration_sec
    period = 1.0 / publish_hz
    while time.monotonic() < deadline:
        localization = localization_monitor.snapshot()
        if localization is None:
            raise RuntimeError("lost localization while publishing planning")
        writer.write(make_trajectory(sequence_num, localization, speed_kph,
                                     horizon_sec, curvature))
        sequence_num += 1
        time.sleep(period)
    return sequence_num


def main():
    args = parse_args()
    validate_args(args)
    curvature = turn_curvature(args.turn, args.radius_m)

    cyber.init()
    node = cyber.Node("yunle_planning_test_publisher")
    localization_monitor = LatestMessage(localization_pb2.LocalizationEstimate)
    chassis_monitor = LatestMessage(chassis_pb2.Chassis)
    control_monitor = LatestMessage(control_cmd_pb2.ControlCommand)
    detail_monitor = LatestMessage(yunle_chassis_receiver_pb2.YunleChassisDetail)

    node.create_reader(LOCALIZATION_TOPIC, localization_pb2.LocalizationEstimate,
                       localization_monitor.callback)
    node.create_reader(CHASSIS_TOPIC, chassis_pb2.Chassis,
                       chassis_monitor.callback)
    node.create_reader(CONTROL_TOPIC, control_cmd_pb2.ControlCommand,
                       control_monitor.callback)
    node.create_reader(DETAIL_TOPIC, yunle_chassis_receiver_pb2.YunleChassisDetail,
                       detail_monitor.callback)
    writer = node.create_writer(args.topic, planning_pb2.ADCTrajectory)

    sequence_num = 1
    exit_code = 0
    try:
        localization = wait_for(LOCALIZATION_TOPIC, localization_monitor, 2.0)
        chassis = wait_for(CHASSIS_TOPIC, chassis_monitor, 2.0)
        initial_speed_kph = abs(chassis_speed_kph(chassis))
        if not math.isfinite(initial_speed_kph) or \
                initial_speed_kph > MAX_INITIAL_SPEED_KPH:
            raise RuntimeError(
                "initial chassis speed is not stationary: {:.3f} kph".format(
                    initial_speed_kph))

        print_chassis("initial chassis", chassis)
        print("Publishing {} planning trajectory on {}".format(
            args.turn, args.topic))
        print(("motion speed_kph={} duration={} horizon={} turn={} "
               "radius_m={} curvature={:.4f}").format(
                   args.speed_kph, args.duration, args.trajectory_seconds,
                   args.turn, args.radius_m, curvature))

        sequence_num = publish_planning(
            writer, localization_monitor, sequence_num, args.speed_kph,
            args.duration, args.trajectory_seconds, args.publish_hz,
            curvature)
        print_control("motion control", control_monitor.snapshot())
        print_detail("motion detail", detail_monitor.snapshot())

        sequence_num = publish_planning(
            writer, localization_monitor, sequence_num, 0.0,
            STOP_DURATION_SEC, args.trajectory_seconds, args.publish_hz, 0.0)
        time.sleep(0.3)
        print_control("stop control", control_monitor.snapshot())
        print_detail("stop detail", detail_monitor.snapshot())
        print_chassis("final chassis", chassis_monitor.snapshot())
        print("Planning command finished; final zero-speed trajectory was published.")
    except Exception as exc:
        exit_code = 1
        print("ERROR: {}".format(exc))
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == "__main__":
    main()
