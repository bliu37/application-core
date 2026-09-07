#!/usr/bin/env python3

"""Read-only snapshot for Yunle indoor navigation routing/planning/control."""

import argparse
import math
import os
import sys
import threading
import time

from cyber.python.cyber_py3 import cyber
from modules.canbus_vehicle.yunle.proto import yunle_chassis_receiver_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.planning_msgs import decision_pb2
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.planning_msgs import planning_command_pb2
from modules.common_msgs.routing_msgs import routing_pb2


TOPICS = (
    ("localization", "/apollo/localization/pose",
     localization_pb2.LocalizationEstimate),
    ("planning_localization", "/apollo/yunle/indoor/planning/localization_pose",
     localization_pb2.LocalizationEstimate),
    ("routing_request", "/apollo/routing_request", routing_pb2.RoutingRequest),
    ("routing_response", "/apollo/routing_response",
     routing_pb2.RoutingResponse),
    ("planning_command", "/apollo/planning/command",
     planning_command_pb2.PlanningCommand),
    ("planning", "/apollo/planning", planning_pb2.ADCTrajectory),
    ("control", "/apollo/control", control_cmd_pb2.ControlCommand),
    ("chassis", "/apollo/canbus/chassis", chassis_pb2.Chassis),
    ("yunle_detail", "/apollo/canbus/yunle_chassis_detail",
     yunle_chassis_receiver_pb2.YunleChassisDetail),
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Summarize current indoor navigation topics without "
                    "publishing commands.")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="observation duration in seconds, in (0, 60]")
    return parser.parse_args()


class TopicStats(object):

    def __init__(self, message_type):
        self._message_type = message_type
        self._lock = threading.Lock()
        self._count = 0
        self._first_wall_time = None
        self._last_wall_time = None
        self._last = None

    def callback(self, message):
        copied = self._message_type()
        copied.CopyFrom(message)
        now = time.time()
        with self._lock:
            if self._count == 0:
                self._first_wall_time = now
            self._count += 1
            self._last_wall_time = now
            self._last = copied

    def snapshot(self):
        with self._lock:
            last = None
            if self._last is not None:
                last = self._message_type()
                last.CopyFrom(self._last)
            return {
                "count": self._count,
                "first_wall_time": self._first_wall_time,
                "last_wall_time": self._last_wall_time,
                "last": last,
            }


def rate(stats):
    if stats["count"] <= 1:
        return 0.0
    span = stats["last_wall_time"] - stats["first_wall_time"]
    if span <= 0.0:
        return 0.0
    return (stats["count"] - 1) / span


def age(stats):
    if stats["last_wall_time"] is None:
        return float("nan")
    return time.time() - stats["last_wall_time"]


def header_age(message):
    if (message is not None and message.HasField("header") and
            message.header.HasField("timestamp_sec")):
        return time.time() - message.header.timestamp_sec
    return float("nan")


def stop_reason_name(reason_code):
    names = {
        decision_pb2.STOP_REASON_DESTINATION: "DESTINATION",
        decision_pb2.STOP_REASON_REFERENCE_END: "REFERENCE_END",
        decision_pb2.STOP_REASON_SIGNAL: "SIGNAL",
        decision_pb2.STOP_REASON_STOP_SIGN: "STOP_SIGN",
        decision_pb2.STOP_REASON_YIELD_SIGN: "YIELD_SIGN",
        decision_pb2.STOP_REASON_CLEAR_ZONE: "CLEAR_ZONE",
        decision_pb2.STOP_REASON_CROSSWALK: "CROSSWALK",
        decision_pb2.STOP_REASON_OBSTACLE: "OBSTACLE",
    }
    return names.get(reason_code, str(reason_code))


def driving_action_name(action):
    driving_action = getattr(control_cmd_pb2, "DrivingAction", None)
    if driving_action is None:
        return str(action)
    names = {
        driving_action.START: "START",
        driving_action.STOP: "STOP",
        driving_action.RESET: "RESET",
    }
    return names.get(action, str(action))


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


def route_segments(response):
    segments = []
    for road in response.road:
        for passage in road.passage:
            for segment in passage.segment:
                segments.append((segment.id, segment.start_s, segment.end_s))
    return segments


def print_topic_header(name, topic, stats):
    print("{} {} count={} rate={:.2f}Hz age={:.3f}s".format(
        name, topic, stats["count"], rate(stats), age(stats)))


def print_localization(message):
    pose = message.pose
    heading = pose.heading if pose.HasField("heading") else float("nan")
    print("  x={:.3f} y={:.3f} z={:.3f} heading={:.3f} age_header={:.3f}s".
          format(pose.position.x, pose.position.y, pose.position.z, heading,
                 header_age(message)))


def print_routing_request(request):
    parts = []
    for waypoint in request.waypoint:
        if waypoint.HasField("pose"):
            parts.append("{} s={:.3f} pose=({:.3f},{:.3f})".format(
                waypoint.id or "<no_lane>",
                waypoint.s if waypoint.HasField("s") else float("nan"),
                waypoint.pose.x, waypoint.pose.y))
        else:
            parts.append("{} s={:.3f}".format(
                waypoint.id or "<no_lane>",
                waypoint.s if waypoint.HasField("s") else float("nan")))
    print("  waypoints={} is_start_pose_set={} {}".format(
        len(request.waypoint), request.is_start_pose_set, "; ".join(parts)))


def print_routing_response(response):
    segments = route_segments(response)
    segment_text = ", ".join(
        "{}[{:.3f},{:.3f}]".format(lane_id, start_s, end_s)
        for lane_id, start_s, end_s in segments)
    distance = response.measurement.distance if response.HasField(
        "measurement") else float("nan")
    print("  roads={} segments={} distance={:.3f}".format(
        len(response.road), segment_text or "none", distance))
    if response.HasField("status"):
        print("  status_code={} status_msg={!r}".format(
            response.status.error_code, response.status.msg))


def print_planning_command(command):
    speed = command.target_speed if command.HasField("target_speed") else (
        float("nan"))
    print("  command_id={} target_speed={:.3f} motion={}".format(
        command.command_id, speed, command.is_motion_command))
    if command.HasField("lane_follow_command"):
        print_routing_response(command.lane_follow_command)


def main_decision_summary(decision):
    if not decision.HasField("main_decision"):
        return "main=none"
    main = decision.main_decision
    if main.HasField("mission_complete"):
        return "main=mission_complete"
    if main.HasField("stop"):
        stop = main.stop
        reason = stop_reason_name(stop.reason_code) if stop.HasField(
            "reason_code") else "none"
        return "main=stop reason={} point=({:.3f},{:.3f}) heading={:.3f}".format(
            reason, stop.stop_point.x, stop.stop_point.y, stop.stop_heading)
    if main.HasField("not_ready"):
        return "main=not_ready reason={!r}".format(main.not_ready.reason)
    return "main=other"


def terminal_object_stops(decision):
    stops = []
    if not decision.HasField("object_decision"):
        return stops
    for object_decision in decision.object_decision.decision:
        for object_decision_type in object_decision.object_decision:
            if not object_decision_type.HasField("stop"):
                continue
            stop = object_decision_type.stop
            reason = stop_reason_name(stop.reason_code) if stop.HasField(
                "reason_code") else "none"
            if reason in ("DESTINATION", "REFERENCE_END"):
                stops.append("{}:{}".format(object_decision.id, reason))
    return stops


def print_planning(planning):
    speeds = [point.v for point in planning.trajectory_point
              if point.HasField("v") and math.isfinite(point.v)]
    min_v = min(speeds) if speeds else float("nan")
    max_v = max(speeds) if speeds else float("nan")
    total_len = planning.total_path_length if planning.HasField(
        "total_path_length") else float("nan")
    total_time = planning.total_path_time if planning.HasField(
        "total_path_time") else float("nan")
    lane_ids = ",".join(lane.id for lane in planning.lane_id) or "none"
    print(("  traj_pts={} path_pts={} total_len={:.3f} total_time={:.3f} "
           "v=[{:.3f},{:.3f}] lanes={} age_header={:.3f}s").format(
               len(planning.trajectory_point), len(planning.path_point),
               total_len, total_time, min_v, max_v, lane_ids,
               header_age(planning)))
    if planning.HasField("estop"):
        print("  estop={} reason={!r}".format(
            planning.estop.is_estop, planning.estop.reason))
    if planning.HasField("decision"):
        print("  " + main_decision_summary(planning.decision))
        stops = terminal_object_stops(planning.decision)
        print("  terminal_object_stops={}".format(
            ",".join(stops) if stops else "none"))


def print_control(control):
    speed = control.speed if control.HasField("speed") else float("nan")
    steering = (control.steering_target
                if control.HasField("steering_target") else float("nan"))
    brake = control.brake if control.HasField("brake") else float("nan")
    pad = "none"
    if control.HasField("pad_msg") and control.pad_msg.HasField("action"):
        pad = driving_action_name(control.pad_msg.action)
    print("  speed={:.3f} steering={:.3f} brake={:.3f} pad_action={} "
          "age_header={:.3f}s".format(
              speed, steering, brake, pad, header_age(control)))


def print_chassis(chassis):
    speed_kph = chassis.speed_mps * 3.6 if chassis.HasField(
        "speed_mps") else float("nan")
    print("  speed_kph={:.3f} gear={} parking={} driving_mode={} "
          "error={} msg={!r}".format(
              speed_kph, gear_name(chassis.gear_location),
              chassis.parking_brake, driving_mode_name(chassis.driving_mode),
              chassis.header.status.error_code, chassis.header.status.msg))


def print_detail(detail):
    print(("  control_send_enabled={} fresh={} interlocks={} sent={} "
           "errors={} pad_started={} reason={!r}").format(
               detail.control_send_enabled, detail.control_command_fresh,
               detail.control_interlocks_ok, detail.sent_control_frame_count,
               detail.control_send_error_count,
               getattr(detail, "control_pad_started", False),
               detail.control_block_reason))
    print(("  terminal_stop_received={} fresh={} reason={!r} "
           "release_ready={} remote_release_active={} "
           "remote_release_frames={} terminal_released={}").format(
               getattr(detail, "planning_terminal_stop_received", False),
               getattr(detail, "planning_terminal_stop_fresh", False),
               getattr(detail, "planning_terminal_stop_reason", ""),
               getattr(detail, "terminal_stop_release_ready", False),
               getattr(detail, "remote_release_active", False),
               getattr(detail, "remote_release_frame_count", 0),
               getattr(detail, "terminal_stop_released_to_remote", False)))


def safe_shutdown(timeout_sec=2.0):
    finished = []

    def do_shutdown():
        try:
            cyber.shutdown()
        finally:
            finished.append(True)

    thread = threading.Thread(target=do_shutdown)
    thread.daemon = True
    thread.start()
    thread.join(timeout_sec)
    if not finished:
        print("WARNING: cyber.shutdown did not finish within {:.1f}s".format(
            timeout_sec))


def main():
    args = parse_args()
    if args.duration <= 0.0 or args.duration > 60.0:
        raise ValueError("--duration must be in (0, 60]")

    node_name = "yunle_indoor_navigation_snapshot_{}".format(os.getpid())
    cyber.init(node_name)
    node = cyber.Node(node_name)
    records = {}
    readers = []
    for name, topic, message_type in TOPICS:
        stats = TopicStats(message_type)
        records[name] = (topic, stats)
        readers.append(node.create_reader(topic, message_type, stats.callback))

    print("Listening for Yunle indoor navigation topics for {:.1f}s...".format(
        args.duration), flush=True)
    time.sleep(args.duration)

    for name, topic, _ in TOPICS:
        topic, stats = records[name]
        snapshot = stats.snapshot()
        message = snapshot["last"]
        print_topic_header(name, topic, snapshot)
        if message is None:
            continue
        if name in ("localization", "planning_localization"):
            print_localization(message)
        elif name == "routing_request":
            print_routing_request(message)
        elif name == "routing_response":
            print_routing_response(message)
        elif name == "planning_command":
            print_planning_command(message)
        elif name == "planning":
            print_planning(message)
        elif name == "control":
            print_control(message)
        elif name == "chassis":
            print_chassis(message)
        elif name == "yunle_detail":
            print_detail(message)

    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
