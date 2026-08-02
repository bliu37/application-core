#!/usr/bin/env python3

"""Bounded commissioning publisher for the isolated Yunle control topic."""

import argparse
import math
import os
import sys
import threading
import time
import traceback

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.canbus_vehicle.yunle.proto import yunle_chassis_receiver_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2


DEFAULT_TOPIC = "/apollo/canbus/yunle_test_control"
DETAIL_TOPIC = "/apollo/canbus/yunle_chassis_detail"
PUBLISH_HZ = 50.0
MAX_SPEED_KPH = 1.5
MAX_STEERING_PERCENTAGE = 90.0
MAX_DURATION_SEC = 6.0
STOP_DURATION_SEC = 0.5
SETTLE_DURATION_SEC = 0.8
STABILITY_CHECK_SEC = 0.3
GEAR_REQUEST_DURATION_SEC = 1.0
BRAKE_RELEASE_DURATION_SEC = 0.5
SPEED_TOLERANCE_KPH = 0.05
SCU_MAX_SPEED_FEEDBACK_UNDERSHOOT_KPH = 0.15
SCU_FEEDBACK_UNDERSHOOT_MIN_REQUEST_KPH = 1.0

GEARS = {
    "drive": chassis_pb2.Chassis.GEAR_DRIVE,
    "neutral": chassis_pb2.Chassis.GEAR_NEUTRAL,
    "reverse": chassis_pb2.Chassis.GEAR_REVERSE,
}

RAW_FEEDBACK_GEARS = {
    "drive": 1,
    "neutral": 2,
    "reverse": 3,
}


class DetailMonitor(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None

    def callback(self, message):
        copied = yunle_chassis_receiver_pb2.YunleChassisDetail()
        copied.CopyFrom(message)
        with self.lock:
            self.latest = copied

    def snapshot(self):
        with self.lock:
            if self.latest is None:
                return None
            copied = yunle_chassis_receiver_pb2.YunleChassisDetail()
            copied.CopyFrom(self.latest)
            return copied


def print_detail(label, detail):
    if detail is None:
        print("{}: no Yunle chassis detail received".format(label))
        return
    print(
        ("{}: communication_ok={} shift={} parking={} ignition={} "
         "auto_switch={} drive_mode={} speed_kph={} warning={} "
         "remote_brake={} emergency_brake={} scu_brake={} "
         "steer_mag={} steer_right={} front_angle={} rear_angle={} "
         "hardware_target_kph={} scu_target_kph={} vehicle_target_kph={} "
         "real_rpm={} rear_rpm=({},{}) cmd_target_kph={} cmd_steering={} "
         "cmd_gear={} enabled={} received={} fresh={} "
         "interlocks={} failsafe={} sent={} errors={} reason={!r}").format(
             label, detail.communication_ok, detail.shift_status,
             detail.parking_status, detail.ignition_status,
             detail.drive_mode_shift_button, detail.drive_mode_raw,
             detail.vehicle_speed_kph, detail.max_warning_level,
             detail.remote_brake, detail.emergency_brake, detail.scu_brake,
             detail.steering_magnitude, detail.steering_direction_right,
             detail.front_steering_angle_deg,
             detail.rear_steering_angle_deg,
             detail.hardware_target_speed_kph,
             detail.scu_target_speed_kph, detail.vehicle_target_speed_kph,
             detail.real_speed_rpm, detail.rear_left_rpm,
             detail.rear_right_rpm, detail.commanded_target_speed_kph,
             detail.commanded_steering_percentage, detail.commanded_gear,
             detail.control_send_enabled, detail.control_command_received,
             detail.control_command_fresh,
             detail.control_interlocks_ok, detail.control_failsafe_active,
             detail.sent_control_frame_count,
             detail.control_send_error_count,
             detail.control_block_reason))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish a bounded Yunle commissioning ControlCommand.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC,
                        help="ControlCommand topic to publish")
    parser.add_argument("--gear", choices=sorted(GEARS), default="neutral")
    parser.add_argument("--speed-kph", type=float, default=0.0)
    parser.add_argument("--steering", type=float, default=0.0,
                        help="Apollo steering percentage; left is positive")
    parser.add_argument("--brake", type=float, default=100.0,
                        help="brake percentage in [0, 100]")
    parser.add_argument("--duration", type=float, default=1.0)
    safety_mode = parser.add_mutually_exclusive_group()
    safety_mode.add_argument(
        "--expect-parking-interlock", action="store_true",
        help=("abort unless chassis feedback reports the parking brake; use "
              "for the no-send interlock test"))
    safety_mode.add_argument(
        "--allow-stationary-gear-change", action="store_true",
        help=("allow an explicit N-to-D/R request only at zero speed, zero "
              "steering, full brake, and at most one second"))
    safety_mode.add_argument(
        "--shift-then-move", action="store_true",
        help=("continuously request N-to-D, release the brake at zero speed, "
              "then execute one bounded forward movement"))
    parser.add_argument(
        "--preflight-only", action="store_true",
        help="check chassis conditions and exit without publishing a command")
    parser.add_argument(
        "--expect-hardware-brake-interrupt", action="store_true",
        help=("treat remote-brake or emergency-brake feedback during the "
              "motion phase as the expected safety interrupt outcome"))
    parser.add_argument(
        "--confirm-stationary-test", action="store_true", required=True,
        help=("confirm that the vehicle is secured and an emergency stop "
              "is ready"))
    return parser.parse_args()


def validate_args(args):
    values = (args.speed_kph, args.steering, args.brake, args.duration)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("all numeric arguments must be finite")
    if args.speed_kph < 0.0 or args.speed_kph > MAX_SPEED_KPH:
        raise ValueError("speed-kph must be in [0, 1.5]")
    if abs(args.steering) > MAX_STEERING_PERCENTAGE:
        raise ValueError("steering must be in [-90, 90]")
    if args.brake < 0.0 or args.brake > 100.0:
        raise ValueError("brake must be in [0, 100]")
    if args.duration <= 0.0 or args.duration > MAX_DURATION_SEC:
        raise ValueError("duration must be in (0, 6]")
    if args.gear == "neutral" and args.speed_kph > 0.0:
        raise ValueError("nonzero speed is not allowed in neutral")
    if args.brake > 0.0 and args.speed_kph > 0.0:
        raise ValueError("nonzero speed and brake cannot be requested together")
    if args.allow_stationary_gear_change:
        if args.gear == "neutral":
            raise ValueError("stationary gear change target must be drive or reverse")
        if (args.speed_kph != 0.0 or args.steering != 0.0 or
                args.brake != 100.0 or args.duration > 1.0):
            raise ValueError(
                "stationary gear change requires zero speed, zero steering, "
                "full brake, and duration no greater than one second")
    if args.shift_then_move:
        if args.gear != "drive":
            raise ValueError("shift-then-move is restricted to drive gear")
        if (args.speed_kph < 0.0 or args.brake != 0.0 or
                args.duration > MAX_DURATION_SEC):
            raise ValueError(
                "shift-then-move requires bounded non-negative speed, zero "
                "motion brake, and duration no greater than six seconds")
    if args.expect_hardware_brake_interrupt and not args.shift_then_move:
        raise ValueError(
            "hardware brake interrupt validation requires shift-then-move")


def make_command(sequence_num, gear, speed_kph, steering, brake):
    command = control_cmd_pb2.ControlCommand()
    command.header.module_name = "yunle_control_test_publisher"
    command.header.sequence_num = sequence_num
    command.header.timestamp_sec = cyber_time.Time.now().to_sec()
    command.gear_location = GEARS[gear]
    command.speed = speed_kph / 3.6
    command.steering_target = steering
    command.brake = brake
    command.parking_brake = False
    command.is_in_safe_mode = False
    return command


def publish_for(writer, sequence_num, duration, gear, speed_kph, steering,
                brake):
    deadline = time.monotonic() + duration
    period = 1.0 / PUBLISH_HZ
    while time.monotonic() < deadline:
        command = make_command(sequence_num, gear, speed_kph, steering, brake)
        writer.write(command)
        sequence_num += 1
        time.sleep(period)
    return sequence_num


def validate_staged_feedback(label, detail, expected_shift,
                             expected_parking, expected_brake):
    if detail is None:
        raise RuntimeError("{} received no chassis feedback".format(label))
    if (not detail.communication_ok or
            not detail.drive_mode_shift_button or
            detail.max_warning_level >= 2 or
            detail.control_send_error_count != 0):
        raise RuntimeError("{} chassis safety state is invalid".format(label))
    if detail.remote_brake or detail.emergency_brake:
        raise RuntimeError("{} hardware brake input is active".format(label))
    if detail.shift_status != expected_shift:
        raise RuntimeError("{} did not reach the requested gear".format(label))
    if abs(detail.vehicle_speed_kph) > SPEED_TOLERANCE_KPH:
        raise RuntimeError("{} detected nonzero vehicle speed".format(label))
    if detail.parking_status != expected_parking:
        raise RuntimeError("{} parking feedback is unexpected".format(label))
    if detail.scu_brake != expected_brake:
        raise RuntimeError("{} SCU brake feedback is unexpected".format(label))


def is_expected_scu_motion_target(actual_speed_kph, expected_speed_kph):
    if abs(actual_speed_kph - expected_speed_kph) <= SPEED_TOLERANCE_KPH:
        return True
    if (expected_speed_kph <
            SCU_FEEDBACK_UNDERSHOOT_MIN_REQUEST_KPH - SPEED_TOLERANCE_KPH):
        return False
    undershoot = expected_speed_kph - actual_speed_kph
    return (undershoot >= -SPEED_TOLERANCE_KPH and
            undershoot <= SCU_MAX_SPEED_FEEDBACK_UNDERSHOOT_KPH)


def validate_motion_feedback(label, detail, expected_speed_kph,
                             expected_steering):
    if detail is None:
        raise RuntimeError("{} received no chassis feedback".format(label))
    if (not detail.communication_ok or
            not detail.drive_mode_shift_button or
            detail.max_warning_level >= 2 or
            detail.control_send_error_count != 0):
        raise RuntimeError("{} chassis safety state is invalid".format(label))
    if detail.remote_brake or detail.emergency_brake:
        raise RuntimeError("{} hardware brake input is active".format(label))
    if detail.shift_status != RAW_FEEDBACK_GEARS["drive"]:
        raise RuntimeError("{} did not remain in drive gear".format(label))
    if detail.commanded_gear != RAW_FEEDBACK_GEARS["drive"]:
        raise RuntimeError("{} commanded gear is not drive".format(label))
    if detail.parking_status:
        raise RuntimeError("{} parking brake was not released".format(label))
    if detail.scu_brake:
        raise RuntimeError("{} SCU brake was not released".format(label))
    if not detail.control_command_fresh:
        raise RuntimeError("{} control command is not fresh".format(label))
    if not detail.control_interlocks_ok:
        raise RuntimeError("{} control interlocks are not OK".format(label))
    if detail.control_failsafe_active:
        raise RuntimeError("{} control failsafe is active".format(label))
    if (abs(detail.commanded_target_speed_kph - expected_speed_kph) >
            SPEED_TOLERANCE_KPH):
        raise RuntimeError("{} component target speed is unexpected".format(label))
    if (abs(detail.commanded_steering_percentage - expected_steering) >
            SPEED_TOLERANCE_KPH):
        raise RuntimeError(
            "{} component steering is unexpected".format(label))
    if not is_expected_scu_motion_target(
            detail.scu_target_speed_kph, expected_speed_kph):
        raise RuntimeError(
            "{} SCU target speed {} is outside expected {}".format(
                label, detail.scu_target_speed_kph, expected_speed_kph))


def validate_hardware_brake_interrupt(label, detail):
    if detail is None:
        raise RuntimeError("{} received no chassis feedback".format(label))
    if (not detail.communication_ok or
            not detail.drive_mode_shift_button or
            detail.max_warning_level >= 2 or
            detail.control_send_error_count != 0):
        raise RuntimeError("{} chassis safety state is invalid".format(label))
    if not (detail.remote_brake or detail.emergency_brake):
        raise RuntimeError(
            "{} did not observe a hardware brake interrupt".format(label))
    if not detail.parking_status:
        raise RuntimeError("{} parking brake was not applied".format(label))
    if not detail.scu_brake:
        raise RuntimeError("{} SCU brake feedback is not active".format(label))
    if abs(detail.scu_target_speed_kph) > SPEED_TOLERANCE_KPH:
        raise RuntimeError("{} SCU target speed is not zero".format(label))
    if abs(detail.vehicle_target_speed_kph) > SPEED_TOLERANCE_KPH:
        raise RuntimeError("{} vehicle target speed is not zero".format(label))
    if (not detail.control_failsafe_active and
            abs(detail.commanded_target_speed_kph) > SPEED_TOLERANCE_KPH):
        raise RuntimeError("{} component did not suppress motion".format(label))


def validate_stopped_after_interrupt(label, detail):
    if detail is None:
        raise RuntimeError("{} received no chassis feedback".format(label))
    if (not detail.communication_ok or detail.control_send_error_count != 0):
        raise RuntimeError("{} chassis safety state is invalid".format(label))
    if abs(detail.vehicle_speed_kph) > SPEED_TOLERANCE_KPH:
        raise RuntimeError("{} vehicle did not settle to zero speed".format(label))
    if not detail.parking_status:
        raise RuntimeError("{} parking brake is not active".format(label))
    if not detail.scu_brake:
        raise RuntimeError("{} SCU brake feedback is not active".format(label))
    if abs(detail.scu_target_speed_kph) > SPEED_TOLERANCE_KPH:
        raise RuntimeError("{} SCU target speed is not zero".format(label))


def publish_stop_and_report(writer, monitor, sequence_num, gear):
    stop_detail = None
    settled_detail = None
    stability_detail = None
    try:
        publish_for(writer, sequence_num, STOP_DURATION_SEC, gear, 0.0,
                    0.0, 100.0)
        time.sleep(0.05)
        stop_detail = monitor.snapshot()
        print_detail("stop phase", stop_detail)
        time.sleep(SETTLE_DURATION_SEC)
        settled_detail = monitor.snapshot()
        print_detail("settled phase", settled_detail)
        time.sleep(STABILITY_CHECK_SEC)
        stability_detail = monitor.snapshot()
        print_detail("stability check", stability_detail)
    except KeyboardInterrupt:
        print("Stop reporting interrupted; component failsafe remains active.")
    return stop_detail, settled_detail, stability_detail


def run_shift_then_move(writer, monitor, args):
    print("Publishing staged N-to-D and bounded movement on {}".format(
        args.topic))
    print("motion speed_kph={} steering={} brake=0 duration={}".format(
        args.speed_kph, args.steering, args.duration))

    sequence_num = 0
    try:
        sequence_num = publish_for(
            writer, sequence_num, GEAR_REQUEST_DURATION_SEC, "drive", 0.0,
            0.0, 100.0)
        time.sleep(0.05)
        gear_detail = monitor.snapshot()
        print_detail("gear phase", gear_detail)
        validate_staged_feedback(
            "gear phase", gear_detail, RAW_FEEDBACK_GEARS["drive"], True,
            True)

        sequence_num = publish_for(
            writer, sequence_num, BRAKE_RELEASE_DURATION_SEC, "drive", 0.0,
            0.0, 0.0)
        time.sleep(0.05)
        release_detail = monitor.snapshot()
        print_detail("release phase", release_detail)
        validate_staged_feedback(
            "release phase", release_detail, RAW_FEEDBACK_GEARS["drive"],
            False, False)

        sequence_num = publish_for(
            writer, sequence_num, args.duration, "drive", args.speed_kph,
            args.steering, 0.0)
        time.sleep(0.05)
        motion_detail = monitor.snapshot()
        print_detail("motion phase", motion_detail)
        if args.expect_hardware_brake_interrupt:
            validate_hardware_brake_interrupt("motion phase", motion_detail)
        else:
            validate_motion_feedback(
                "motion phase", motion_detail, args.speed_kph, args.steering)
    except KeyboardInterrupt:
        print("Interrupted; publishing the bounded stop command.")
    finally:
        _, settled_detail, stability_detail = publish_stop_and_report(
            writer, monitor, sequence_num, "drive")
    if args.expect_hardware_brake_interrupt:
        validate_stopped_after_interrupt("settled phase", settled_detail)
        validate_stopped_after_interrupt("stability check", stability_detail)
        print("Hardware brake interrupt test passed; vehicle settled stopped.")
    else:
        print("Staged command finished; final zero-speed brake was published.")


def main():
    args = parse_args()
    validate_args(args)

    cyber.init()
    node = cyber.Node("yunle_control_test_publisher_{}".format(os.getpid()))
    writer = node.create_writer(args.topic, control_cmd_pb2.ControlCommand)
    monitor = DetailMonitor()
    node.create_reader(
        DETAIL_TOPIC,
        yunle_chassis_receiver_pb2.YunleChassisDetail,
        monitor.callback)
    time.sleep(0.5)

    initial_detail = monitor.snapshot()
    print_detail("initial", initial_detail)
    if initial_detail is None or not initial_detail.control_send_enabled:
        raise RuntimeError("commissioning component is not enabled")
    if args.expect_parking_interlock:
        if not initial_detail.parking_status:
            raise RuntimeError("parking interlock test requires parking=true")
    elif args.allow_stationary_gear_change or args.shift_then_move:
        if initial_detail.parking_status:
            raise RuntimeError("stationary gear change requires parking=false")
        if initial_detail.shift_status != RAW_FEEDBACK_GEARS["neutral"]:
            raise RuntimeError("stationary gear change must start in neutral")
        if (not initial_detail.communication_ok or
                not initial_detail.drive_mode_shift_button or
                initial_detail.max_warning_level >= 2 or
                initial_detail.vehicle_speed_kph != 0.0):
            raise RuntimeError(
                "stationary gear change safety conditions are not met")
    else:
        if initial_detail.parking_status:
            raise RuntimeError(
                "release parking only after securing the vehicle")
        if initial_detail.shift_status != RAW_FEEDBACK_GEARS[args.gear]:
            raise RuntimeError("requested gear does not match chassis feedback")
        if (not initial_detail.communication_ok or
                not initial_detail.drive_mode_shift_button or
                initial_detail.max_warning_level >= 2 or
                abs(initial_detail.vehicle_speed_kph) > MAX_SPEED_KPH):
            raise RuntimeError("initial chassis safety conditions are not met")
    if args.preflight_only:
        print("Preflight passed; no control command was published.")
        return
    if args.shift_then_move:
        run_shift_then_move(writer, monitor, args)
        return

    print("Publishing bounded Yunle command on {}".format(args.topic))
    print("gear={} speed_kph={} steering={} brake={} duration={}".format(
        args.gear, args.speed_kph, args.steering, args.brake, args.duration))

    sequence_num = 0
    try:
        sequence_num = publish_for(
            writer, sequence_num, args.duration, args.gear, args.speed_kph,
            args.steering, args.brake)
        time.sleep(0.05)
        print_detail("command phase", monitor.snapshot())
    except KeyboardInterrupt:
        print("Interrupted; publishing the bounded stop command.")
    finally:
        publish_stop_and_report(writer, monitor, sequence_num, args.gear)
    print("Command finished; final zero-speed brake command was published.")


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted before command completion.", file=sys.stderr)
        exit_code = 130
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)
