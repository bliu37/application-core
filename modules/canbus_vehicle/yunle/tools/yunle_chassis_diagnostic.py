#!/usr/bin/env python3

"""Read-only Yunle chassis feedback diagnostic."""

import argparse
import math
import os
import sys
import threading
import time

from cyber.python.cyber_py3 import cyber
from modules.canbus_vehicle.yunle.proto import yunle_chassis_receiver_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2


CHASSIS_TOPIC = "/apollo/canbus/chassis"
DETAIL_TOPIC = "/apollo/canbus/yunle_chassis_detail"


class TopicStats(object):

    def __init__(self, message_type):
        self._message_type = message_type
        self._lock = threading.Lock()
        self._count = 0
        self._first_wall_time = None
        self._last_wall_time = None
        self._first = None
        self._last = None

    def callback(self, message):
        copied = self._message_type()
        copied.CopyFrom(message)
        now = time.monotonic()
        with self._lock:
            if self._count == 0:
                self._first_wall_time = now
                self._first = copied
            self._count += 1
            self._last_wall_time = now
            self._last = copied

    def snapshot(self):
        with self._lock:
            first = None
            last = None
            if self._first is not None:
                first = self._message_type()
                first.CopyFrom(self._first)
            if self._last is not None:
                last = self._message_type()
                last.CopyFrom(self._last)
            return {
                "count": self._count,
                "first_wall_time": self._first_wall_time,
                "last_wall_time": self._last_wall_time,
                "first": first,
                "last": last,
            }


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


def driving_mode_name(mode):
    names = {
        chassis_pb2.Chassis.COMPLETE_MANUAL: "COMPLETE_MANUAL",
        chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE: "COMPLETE_AUTO_DRIVE",
        chassis_pb2.Chassis.AUTO_STEER_ONLY: "AUTO_STEER_ONLY",
        chassis_pb2.Chassis.AUTO_SPEED_ONLY: "AUTO_SPEED_ONLY",
        chassis_pb2.Chassis.EMERGENCY_MODE: "EMERGENCY_MODE",
        chassis_pb2.Chassis.CHASSIS_ERROR: "CHASSIS_ERROR",
    }
    return names.get(mode, str(mode))


def chassis_speed_kph(chassis):
    if chassis is None or not chassis.HasField("speed_mps"):
        return float("nan")
    return chassis.speed_mps * 3.6


def rate(stats):
    if stats["count"] <= 1:
        return 0.0
    span = stats["last_wall_time"] - stats["first_wall_time"]
    if span <= 0.0:
        return 0.0
    return (stats["count"] - 1) / span


def print_chassis(stats):
    chassis = stats["last"]
    print("chassis: messages={} rate={:.3f}Hz".format(
        stats["count"], rate(stats)))
    if chassis is None:
        return
    print(("  speed_kph={:.3f} gear={} parking={} driving_mode={} "
           "error_code={} msg={!r}").format(
               chassis_speed_kph(chassis), gear_name(chassis.gear_location),
               chassis.parking_brake,
               driving_mode_name(chassis.driving_mode),
               chassis.header.status.error_code,
               chassis.header.status.msg))


def print_detail(stats):
    detail = stats["last"]
    print("detail: messages={} rate={:.3f}Hz".format(
        stats["count"], rate(stats)))
    if detail is None:
        return
    pad_start_required = getattr(detail, "control_pad_start_required", False)
    pad_started = getattr(detail, "control_pad_started", False)
    last_pad_action = getattr(detail, "last_control_pad_action", 0)
    terminal_stop_received = getattr(
        detail, "planning_terminal_stop_received", False)
    terminal_stop_fresh = getattr(
        detail, "planning_terminal_stop_fresh", False)
    terminal_stop_reason = getattr(
        detail, "planning_terminal_stop_reason", "")
    terminal_release_ready = getattr(
        detail, "terminal_stop_release_ready", False)
    remote_release_active = getattr(detail, "remote_release_active", False)
    remote_release_frames = getattr(detail, "remote_release_frame_count", 0)
    terminal_released = getattr(
        detail, "terminal_stop_released_to_remote", False)
    print(("  communication_ok={} shift={} parking={} ignition={} "
           "auto_switch={} drive_mode_raw={} speed_kph={:.3f} "
           "warning={} remote_brake={} emergency_brake={} scu_brake={}").format(
               detail.communication_ok, detail.shift_status,
               detail.parking_status, detail.ignition_status,
               detail.drive_mode_shift_button, detail.drive_mode_raw,
               detail.vehicle_speed_kph, detail.max_warning_level,
               detail.remote_brake, detail.emergency_brake,
               detail.scu_brake))
    print(("  steering_mag={:.2f} steer_right={} front_angle={:.2f} "
           "rear_angle={:.2f} target_kph={:.3f} "
           "battery={:.1f}V soc={}%").format(
               detail.steering_magnitude, detail.steering_direction_right,
               detail.front_steering_angle_deg,
               detail.rear_steering_angle_deg,
               detail.vehicle_target_speed_kph,
               detail.battery_voltage_v,
               detail.battery_soc_percentage))
    print(("  control_send_enabled={} command_received={} fresh={} "
           "interlocks={} failsafe={} sent={} errors={} "
           "pad_start_required={} pad_started={} last_pad_action={} "
           "cmd_target_kph={:.3f} cmd_steering={:.2f} cmd_gear={} "
           "reason={!r}").format(
               detail.control_send_enabled,
               detail.control_command_received,
               detail.control_command_fresh,
               detail.control_interlocks_ok,
               detail.control_failsafe_active,
               detail.sent_control_frame_count,
               detail.control_send_error_count,
               pad_start_required,
               pad_started,
               last_pad_action,
               detail.commanded_target_speed_kph,
               detail.commanded_steering_percentage,
               detail.commanded_gear,
               detail.control_block_reason))
    print(("  planning_terminal_stop_received={} fresh={} reason={!r} "
           "release_ready={} remote_release_active={} "
           "remote_release_frames={} terminal_released_to_remote={}").format(
               terminal_stop_received,
               terminal_stop_fresh,
               terminal_stop_reason,
               terminal_release_ready,
               remote_release_active,
               remote_release_frames,
               terminal_released))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Summarize Yunle chassis feedback without publishing control.")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--max-stationary-speed-kph", type=float, default=0.2)
    parser.add_argument("--expect-control-send-disabled",
                        action="store_true", default=True)
    parser.add_argument("--allow-control-send-enabled",
                        action="store_false",
                        dest="expect_control_send_disabled")
    parser.add_argument("--require-control-send-enabled",
                        action="store_true",
                        help="require the commissioning sender path to be enabled")
    parser.add_argument("--require-control-pad-start-required",
                        action="store_true",
                        help="require the Yunle receiver to gate SCU control on Pad START")
    parser.add_argument("--require-control-pad-not-started",
                        action="store_true",
                        help="require the Yunle receiver to still be waiting for Pad START")
    parser.add_argument("--require-control-pad-started",
                        action="store_true",
                        help="require the Yunle receiver to have observed Pad START")
    parser.add_argument("--require-no-control-frames",
                        action="store_true",
                        help="require sent_control_frame_count to remain zero")
    parser.add_argument("--require-auto-switch", action="store_true")
    parser.add_argument("--require-auto-drive-raw", action="store_true",
                        help="require raw chassis drive_mode value 1")
    parser.add_argument("--require-terminal-stop-released-to-remote",
                        action="store_true",
                        help="require terminal-stop release-to-remote latch")
    parser.add_argument("--require-remote-release-frames",
                        action="store_true",
                        help="require at least one terminal-stop release frame")
    return parser.parse_args()


def main():
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise ValueError("--duration must be positive")

    cyber.init()
    node = cyber.Node("yunle_chassis_diagnostic")
    chassis_stats = TopicStats(chassis_pb2.Chassis)
    detail_stats = TopicStats(yunle_chassis_receiver_pb2.YunleChassisDetail)
    node.create_reader(CHASSIS_TOPIC, chassis_pb2.Chassis,
                       chassis_stats.callback)
    node.create_reader(DETAIL_TOPIC, yunle_chassis_receiver_pb2.YunleChassisDetail,
                       detail_stats.callback)

    print("Listening for Yunle chassis feedback for {:.1f}s...".format(
        args.duration))
    time.sleep(args.duration)

    chassis = chassis_stats.snapshot()
    detail = detail_stats.snapshot()
    print_chassis(chassis)
    print_detail(detail)

    problems = []
    if chassis["count"] == 0:
        problems.append("no /apollo/canbus/chassis messages")
    if detail["count"] == 0:
        problems.append("no /apollo/canbus/yunle_chassis_detail messages")

    last_chassis = chassis["last"]
    last_detail = detail["last"]
    if last_chassis is not None:
        speed = abs(chassis_speed_kph(last_chassis))
        if not math.isfinite(speed) or speed > args.max_stationary_speed_kph:
            problems.append("vehicle is not stationary: {:.3f} kph".format(speed))
    if last_detail is not None:
        if not last_detail.communication_ok:
            problems.append("chassis communication is not OK")
        if last_detail.max_warning_level >= 2:
            problems.append("max warning level is {}".format(
                last_detail.max_warning_level))
        if last_detail.remote_brake:
            problems.append("remote brake is active")
        if last_detail.emergency_brake:
            problems.append("emergency brake is active")
        if last_detail.control_send_error_count != 0:
            problems.append("control send error count is {}".format(
                last_detail.control_send_error_count))
        if args.expect_control_send_disabled and last_detail.control_send_enabled:
            problems.append("control send is enabled in a read-only check")
        if args.expect_control_send_disabled and \
                last_detail.sent_control_frame_count != 0:
            problems.append("control frames were sent during read-only check")
        if args.require_control_send_enabled and \
                not last_detail.control_send_enabled:
            problems.append("control send is not enabled")
        pad_start_required = getattr(
            last_detail, "control_pad_start_required", False)
        pad_started = getattr(last_detail, "control_pad_started", False)
        if args.require_control_pad_start_required and \
                not pad_start_required:
            problems.append("control Pad START gate is not required")
        if args.require_control_pad_not_started and pad_started:
            problems.append("control Pad START gate is already started")
        if args.require_control_pad_started and not pad_started:
            problems.append("control Pad START gate has not started")
        if args.require_no_control_frames and \
                last_detail.sent_control_frame_count != 0:
            problems.append("control frames were sent: {}".format(
                last_detail.sent_control_frame_count))
        if args.require_auto_switch and not last_detail.drive_mode_shift_button:
            problems.append("auto drive physical switch is not active")
        if args.require_auto_drive_raw and last_detail.drive_mode_raw != 1:
            problems.append("raw drive mode is {}, expected 1 for Apollo AUTO".format(
                last_detail.drive_mode_raw))
        terminal_released = getattr(
            last_detail, "terminal_stop_released_to_remote", False)
        remote_release_frames = getattr(
            last_detail, "remote_release_frame_count", 0)
        if args.require_terminal_stop_released_to_remote and \
                not terminal_released:
            problems.append("terminal stop has not released to remote")
        if args.require_remote_release_frames and remote_release_frames == 0:
            problems.append("no terminal-stop remote release frames observed")

    if problems:
        print("RESULT: NOT_READY")
        for problem in problems:
            print("  - {}".format(problem))
        exit_code = 1
    else:
        print("RESULT: CHASSIS_READONLY_READY")
        exit_code = 0

    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)


if __name__ == "__main__":
    main()
