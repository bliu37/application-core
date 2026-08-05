#!/usr/bin/env python3

"""Run an indoor Apollo Planning preflight without Dreamview or Control."""

import argparse
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import threading
import time

from google.protobuf import text_format

from yunle_indoor_navigation_readonly_preflight import (
    CHASSIS_DIAG,
    DEFAULT_MAP_DIR as DEFAULT_NDT_MAP_DIR,
    LOCALIZATION_DIAG,
    MODULES,
    apollo_process_env,
    existing_mainboards,
    launch_module,
    run_diagnostic,
    run_text,
    stop_started,
    validate_paths,
)
from yunle_indoor_routing_preflight import (
    load_request,
    summarize_response,
    write_routing_runtime_files,
)

APOLLO_DISTRIBUTION_HOME = "/opt/apollo/neo"
APOLLO_PYTHON = os.path.join(APOLLO_DISTRIBUTION_HOME, "python")
if os.path.isdir(APOLLO_PYTHON) and APOLLO_PYTHON not in sys.path:
    sys.path.insert(0, APOLLO_PYTHON)

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.planning_msgs import planning_command_pb2
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.prediction_msgs import prediction_obstacle_pb2
from modules.common_msgs.routing_msgs import routing_pb2


DEFAULT_HD_MAP_DIR = Path("/apollo/modules/map/data/yunle_indoor_map03_minimal")
DEFAULT_MAINBOARD = "/opt/apollo/neo/bin/mainboard"
DEFAULT_LOG_DIR = Path("/apollo_workspace/data/log/yunle_planning_preflight")
PLANNING_COMMAND_TOPIC = "/apollo/planning/command"
PLANNING_TOPIC = "/apollo/planning"
PREDICTION_TOPIC = "/apollo/prediction"
LOCALIZATION_TOPIC = "/apollo/localization/pose"
PLANNING_LOCALIZATION_TOPIC = "/apollo/yunle/indoor/planning/localization_pose"
CHASSIS_TOPIC = "/apollo/canbus/chassis"
OLD_ROUTING_REQUEST_TOPIC = "/apollo/routing_request"
ROUTING_RESPONSE_TOPIC = "/apollo/routing_response"
HEADING_ADAPTER_DAG = (
    "/apollo_workspace/profiles/current/modules/tools/"
    "yunle_dreamview_bridge/dag/indoor_planning_heading_adapter.dag"
)

EXPECTED_NDT_PROCESS = (
    "yunle_indoor_pclomp_ndt_localization_map03_measured_z_stable"
)
BLOCKING_PROCESSES = [
    "control",
    "planning",
    "yunle_indoor_planning_preflight",
    "yunle_indoor_planning_heading_adapter",
    "yunle_indoor_routing_preflight",
    "routing",
    "yunle_chassis_apollo_control_test",
    "yunle_chassis_control_test",
]

LANE_FOLLOW_TASK_CONFIGS = [
    ("lane_change_path", "lane_change_path"),
    ("lane_follow_path", "lane_follow_path"),
    ("lane_borrow_path", "lane_borrow_path"),
    ("fallback_path", "fallback_path"),
    ("path_decider", "path_decider"),
    ("rule_based_stop_decider", "rule_based_stop_decider"),
    ("speed_bounds_priori_decider", "speed_bounds_decider"),
    ("speed_heuristic_optimizer", "path_time_heuristic"),
    ("speed_decider", "speed_decider"),
    ("speed_bounds_final_decider", "speed_bounds_decider"),
    ("piecewise_jerk_speed", "piecewise_jerk_speed"),
]


class TopicMonitor(object):

    def __init__(self, message_type):
        self._message_type = message_type
        self._lock = threading.Lock()
        self._count = 0
        self._first_wall_time = None
        self._last_wall_time = None
        self._latest = None
        self._valid_planning = []

    def callback(self, message):
        copied = self._message_type()
        copied.CopyFrom(message)
        now = time.monotonic()
        with self._lock:
            if self._count == 0:
                self._first_wall_time = now
            self._count += 1
            self._last_wall_time = now
            self._latest = copied
            if isinstance(copied, planning_pb2.ADCTrajectory):
                if planning_is_valid(copied):
                    saved = planning_pb2.ADCTrajectory()
                    saved.CopyFrom(copied)
                    self._valid_planning.append(saved)

    def snapshot(self):
        with self._lock:
            latest = None
            if self._latest is not None:
                latest = self._message_type()
                latest.CopyFrom(self._latest)
            valid = []
            for item in self._valid_planning[-5:]:
                copied = planning_pb2.ADCTrajectory()
                copied.CopyFrom(item)
                valid.append(copied)
            return {
                "count": self._count,
                "first_wall_time": self._first_wall_time,
                "last_wall_time": self._last_wall_time,
                "latest": latest,
                "valid_planning": valid,
            }


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Start read-only indoor localization/chassis inputs, Routing and "
            "Planning, then verify that Apollo Planning publishes a non-empty "
            "ADCTrajectory. This script never starts Control."
        ))
    parser.add_argument("--hd-map-dir", type=Path, default=DEFAULT_HD_MAP_DIR)
    parser.add_argument("--ndt-map-dir", default=DEFAULT_NDT_MAP_DIR)
    parser.add_argument("--routing-test-file", type=Path,
                        help="RoutingRequest text proto; defaults to "
                        "<hd-map-dir>/routing_test.pb.txt")
    parser.add_argument("--localization-duration", type=float, default=10.0)
    parser.add_argument("--chassis-duration", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--publish-hz", type=float, default=10.0)
    parser.add_argument("--target-speed-mps", type=float, default=0.5)
    parser.add_argument("--direct-planning-command", action="store_true",
                        help="debug path: start Routing and publish "
                        "PlanningCommand directly from Python. The default "
                        "uses Apollo old_routing_adapter plus "
                        "external_command_process.")
    parser.add_argument("--skip-start-inputs", action="store_true",
                        help="do not start lidar/gnss/tf/liorf/ndt/chassis")
    parser.add_argument("--skip-input-diagnostics", action="store_true")
    parser.add_argument("--synthetic-localization", action="store_true",
                        help="publish a static localization/chassis pose at "
                        "the route start. This isolates HDMap/Routing/"
                        "Planning and does not start physical input modules.")
    parser.add_argument("--route-heading-localization", action="store_true",
                        help="Planning-only diagnostic: keep live localization "
                        "X/Y/Z but publish a private localization topic whose "
                        "heading follows the route direction. This does not "
                        "modify /apollo/localization/pose.")
    parser.add_argument("--use-heading-adapter", action="store_true",
                        help="Start the Yunle C++ route-heading adapter and "
                        "make Planning read its private localization topic.")
    parser.add_argument("--synthetic-forward-offset", type=float, default=0.7,
                        help="meters to move the synthetic pose forward from "
                        "the first routing waypoint")
    parser.add_argument("--keep-running", action="store_true",
                        help="leave modules started by this script running")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--mainboard",
                        default=os.environ.get("MAINBOARD_BIN",
                                               DEFAULT_MAINBOARD))
    return parser.parse_args()


def validate_extra_paths(args):
    validate_paths(args.mainboard)
    hd_map_dir = args.hd_map_dir.resolve()
    request_file = (args.routing_test_file or
                    (hd_map_dir / "routing_test.pb.txt")).resolve()
    missing = []
    for name in ("base_map.bin", "routing_map.bin", "sim_map.bin"):
        if not (hd_map_dir / name).is_file():
            missing.append(str(hd_map_dir / name))
    if not request_file.is_file():
        missing.append(str(request_file))
    planning_dag = "/apollo/modules/planning/planning_component/dag/planning.dag"
    planning_config = (
        "/apollo/modules/planning/planning_component/conf/"
        "planning_config.pb.txt"
    )
    planning_conf = (
        "/apollo/modules/planning/planning_component/conf/planning.conf"
    )
    for path in (planning_dag, planning_config, planning_conf, args.mainboard):
        if not os.path.exists(path):
            missing.append(path)
    if args.use_heading_adapter and not os.path.exists(HEADING_ADAPTER_DAG):
        missing.append(HEADING_ADAPTER_DAG)
    if missing:
        raise RuntimeError("missing required files:\n  " + "\n  ".join(missing))
    return hd_map_dir, request_file


def process_group_in_line(line, process_group):
    return "-p {}".format(process_group) in line


def find_blocking_processes():
    blockers = []
    lines = existing_mainboards()
    for line in lines:
        if "pgrep -af mainboard" in line:
            continue
        for process_group in BLOCKING_PROCESSES:
            if process_group == "planning":
                if (process_group_in_line(line, "planning") or
                        process_group_in_line(line,
                                              "yunle_indoor_planning_preflight")):
                    blockers.append(line)
                    break
                continue
            if process_group_in_line(line, process_group):
                blockers.append(line)
                break
    return blockers


def find_wrong_indoor_ndt_processes():
    wrong = []
    for line in existing_mainboards():
        if "yunle_indoor_ndt_localization" not in line:
            continue
        if EXPECTED_NDT_PROCESS in line:
            continue
        wrong.append(line)
    return wrong


def write_planning_runtime_files(temp_dir, hd_map_dir, target_speed_mps,
                                 localization_topic):
    flagfile = temp_dir / "planning_yunle_indoor.conf"
    speed = max(0.05, min(float(target_speed_mps), 0.8))
    flagfile.write_text(
        "--flagfile=/apollo/modules/planning/planning_component/conf/"
        "planning.conf\n"
        "--map_dir={}\n"
        "--base_map_filename=base_map.bin|base_map.txt\n"
        "--planning_upper_speed_limit={:.3f}\n"
        "--default_cruise_speed={:.3f}\n"
        "--destination_check_distance=20.0\n".format(
            hd_map_dir, speed, speed),
        encoding="utf-8")

    dag = temp_dir / "planning_yunle_indoor.dag"
    dag.write_text(
        "module_config {\n"
        "  module_library: \"modules/planning/planning_component/"
        "libplanning_component.so\"\n"
        "  components {\n"
        "    class_name: \"PlanningComponent\"\n"
        "    config {\n"
        "      name: \"planning\"\n"
        "      config_file_path: \"/apollo/modules/planning/"
        "planning_component/conf/planning_config.pb.txt\"\n"
        "      flag_file_path: \"" + str(flagfile) + "\"\n"
        "      readers: [\n"
        "        {\n"
        "          channel: \"" + PREDICTION_TOPIC + "\"\n"
        "        },\n"
        "        {\n"
        "          channel: \"/apollo/canbus/chassis\"\n"
        "          qos_profile: { depth: 15 }\n"
        "          pending_queue_size: 50\n"
        "        },\n"
        "        {\n"
        "          channel: \"" + localization_topic + "\"\n"
        "          qos_profile: { depth: 15 }\n"
        "          pending_queue_size: 50\n"
        "        }\n"
        "      ]\n"
        "    }\n"
        "  }\n"
        "}\n",
        encoding="utf-8")
    return dag, flagfile


def write_command_runtime_files(temp_dir, hd_map_dir):
    flagfile = temp_dir / "command_yunle_indoor.conf"
    flagfile.write_text(
        "--flagfile=/apollo/modules/common/data/global_flagfile.txt\n"
        "--map_dir={}\n"
        "--base_map_filename=base_map.bin|base_map.txt\n"
        "--routing_response_topic={}\n".format(
            hd_map_dir, ROUTING_RESPONSE_TOPIC),
        encoding="utf-8")

    external_dag = temp_dir / "external_command_yunle_indoor.dag"
    external_dag.write_text(
        "module_config {\n"
        "  module_library: \"modules/external_command/process_component/"
        "libexternal_command_process_component.so\"\n"
        "  components {\n"
        "    class_name: \"ExternalCommandProcessComponent\"\n"
        "    config {\n"
        "      name: \"external_command_process\"\n"
        "      config_file_path: \"/apollo/modules/external_command/"
        "process_component/conf/config.pb.txt\"\n"
        "      flag_file_path: \"" + str(flagfile) + "\"\n"
        "    }\n"
        "  }\n"
        "}\n",
        encoding="utf-8")

    adapter_dag = temp_dir / "old_routing_adapter_yunle_indoor.dag"
    adapter_dag.write_text(
        "module_config {\n"
        "  module_library: \"modules/external_command/old_routing_adapter/"
        "libold_routing_adapter.so\"\n"
        "  components {\n"
        "    class_name: \"OldRoutingAdapter\"\n"
        "    config {\n"
        "      name: \"old_routing_adapter\"\n"
        "      config_file_path: \"/apollo/modules/external_command/"
        "old_routing_adapter/conf/config.pb.txt\"\n"
        "      flag_file_path: \"" + str(flagfile) + "\"\n"
        "    }\n"
        "  }\n"
        "}\n",
        encoding="utf-8")
    return [external_dag, adapter_dag], flagfile


def prepare_planning_conf_overlay(temp_dir):
    overlay = temp_dir / "conf_overlay"
    lane_follow_stage = (
        overlay / "modules/planning/scenarios/lane_follow/conf/"
        "lane_follow_stage"
    )
    lane_follow_stage.mkdir(parents=True, exist_ok=True)
    for expected_name, task_name in LANE_FOLLOW_TASK_CONFIGS:
        source = Path(
            "/apollo/modules/planning/tasks/{}/conf/default_conf.pb.txt".
            format(task_name))
        if not source.is_file():
            source = Path(
                "/opt/apollo/neo/share/modules/planning/tasks/{}/conf/"
                "default_conf.pb.txt".format(task_name))
        if not source.is_file():
            raise RuntimeError("missing task config for {}: {}".format(
                expected_name, source))
        shutil.copyfile(str(source),
                        str(lane_follow_stage /
                            "{}.pb.txt".format(expected_name)))
    return overlay


def planning_process_env(conf_overlay):
    env = apollo_process_env()
    existing = env.get("APOLLO_CONF_PATH", "/apollo")
    share = os.path.join(APOLLO_DISTRIBUTION_HOME, "share")
    paths = [str(conf_overlay), existing, share]
    env["APOLLO_CONF_PATH"] = ":".join(paths)
    return env


def launch_mainboard(process_group, dag, log_path, mainboard, env=None):
    log_file = log_path.open("w")
    command = [mainboard, "-p", process_group]
    dags = dag if isinstance(dag, list) else [dag]
    for one_dag in dags:
        command.extend(["-d", str(one_dag)])
    print("start {}: {}".format(process_group, " ".join(command)))
    print("  log={}".format(log_path))
    process = subprocess.Popen(
        command, stdout=log_file, stderr=subprocess.STDOUT,
        cwd="/apollo_workspace", env=env or apollo_process_env(),
        preexec_fn=os.setsid)
    return {
        "process": process,
        "log_file": log_file,
        "process_group": process_group,
    }


def stop_process_group(item):
    process = item["process"]
    if process.poll() is None:
        print("stop: {}".format(item["process_group"]))
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                pass
            try:
                process.wait(timeout=5.0)
            except Exception:
                pass
    item["log_file"].close()


def launch_routing(args, hd_map_dir):
    temp_dir = Path(tempfile.mkdtemp(prefix="yunle_planning_routing_"))
    dag, flagfile = write_routing_runtime_files(temp_dir, hd_map_dir)
    print("routing_flagfile={}".format(flagfile))
    item = launch_mainboard(
        "yunle_indoor_routing_preflight",
        dag,
        args.log_dir / "routing_mainboard.log",
        args.mainboard)
    item["temp_dir"] = temp_dir
    time.sleep(2.0)
    if item["process"].poll() is not None:
        raise RuntimeError("Routing exited early; see {}".format(
            args.log_dir / "routing_mainboard.log"))
    return item


def launch_planning(args, hd_map_dir, localization_topic):
    temp_dir = Path(tempfile.mkdtemp(prefix="yunle_planning_preflight_"))
    conf_overlay = prepare_planning_conf_overlay(temp_dir)
    dag, flagfile = write_planning_runtime_files(
        temp_dir, hd_map_dir, args.target_speed_mps, localization_topic)
    print("planning_flagfile={}".format(flagfile))
    print("planning_conf_overlay={}".format(conf_overlay))
    item = launch_mainboard(
        "yunle_indoor_planning_preflight",
        dag,
        args.log_dir / "planning_mainboard.log",
        args.mainboard,
        env=planning_process_env(conf_overlay))
    item["temp_dir"] = temp_dir
    time.sleep(2.0)
    if item["process"].poll() is not None:
        raise RuntimeError("Planning exited early; see {}".format(
            args.log_dir / "planning_mainboard.log"))
    return item


def launch_command_process(args, hd_map_dir):
    temp_dir = Path(tempfile.mkdtemp(prefix="yunle_command_preflight_"))
    dags, flagfile = write_command_runtime_files(temp_dir, hd_map_dir)
    print("command_flagfile={}".format(flagfile))
    item = launch_mainboard(
        "yunle_indoor_command_preflight",
        dags,
        args.log_dir / "command_mainboard.log",
        args.mainboard)
    item["temp_dir"] = temp_dir
    time.sleep(2.0)
    if item["process"].poll() is not None:
        raise RuntimeError("Command process exited early; see {}".format(
            args.log_dir / "command_mainboard.log"))
    return item


def launch_heading_adapter(args):
    item = launch_mainboard(
        "yunle_indoor_planning_heading_adapter",
        HEADING_ADAPTER_DAG,
        args.log_dir / "heading_adapter_mainboard.log",
        args.mainboard)
    time.sleep(1.0)
    if item["process"].poll() is not None:
        raise RuntimeError("Heading adapter exited early; see {}".format(
            args.log_dir / "heading_adapter_mainboard.log"))
    return item


def fill_header(header, module_name, sequence):
    header.module_name = module_name
    header.sequence_num = sequence
    header.timestamp_sec = time.time()


def make_planning_command(response, target_speed_mps, sequence):
    command = planning_command_pb2.PlanningCommand()
    fill_header(command.header, "yunle_indoor_planning_preflight", sequence)
    command.command_id = int(time.time() * 1000)
    command.lane_follow_command.CopyFrom(response)
    command.target_speed = target_speed_mps
    command.is_motion_command = True
    return command


def make_prediction(sequence):
    prediction = prediction_obstacle_pb2.PredictionObstacles()
    fill_header(prediction.header, "yunle_indoor_planning_preflight", sequence)
    return prediction


def waypoint_pose(waypoint):
    if waypoint.HasField("pose"):
        return waypoint.pose.x, waypoint.pose.y
    raise RuntimeError("routing waypoint has no pose: {}".format(waypoint))


def route_start_pose(routing_request, forward_offset):
    if len(routing_request.waypoint) < 2:
        raise RuntimeError("routing request needs at least two waypoints")
    start = routing_request.waypoint[0]
    end = routing_request.waypoint[1]
    start_x, start_y = waypoint_pose(start)
    end_x, end_y = waypoint_pose(end)
    if start.HasField("heading"):
        heading = start.heading
    else:
        heading = math.atan2(end_y - start_y, end_x - start_x)
    distance = max(0.0, min(float(forward_offset), 2.0))
    return (start_x + math.cos(heading) * distance,
            start_y + math.sin(heading) * distance,
            heading)


def route_heading(routing_request):
    if len(routing_request.waypoint) < 2:
        raise RuntimeError("routing request needs at least two waypoints")
    start = routing_request.waypoint[0]
    end = routing_request.waypoint[1]
    if start.HasField("heading"):
        return start.heading
    start_x, start_y = waypoint_pose(start)
    end_x, end_y = waypoint_pose(end)
    return math.atan2(end_y - start_y, end_x - start_x)


def make_synthetic_localization(x, y, heading, sequence):
    localization = localization_pb2.LocalizationEstimate()
    fill_header(localization.header, "yunle_indoor_planning_preflight",
                sequence)
    localization.measurement_time = localization.header.timestamp_sec
    pose = localization.pose
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    pose.heading = heading
    pose.orientation.qx = 0.0
    pose.orientation.qy = 0.0
    pose.orientation.qz = math.sin(heading * 0.5)
    pose.orientation.qw = math.cos(heading * 0.5)
    pose.linear_velocity.x = 0.0
    pose.linear_velocity.y = 0.0
    pose.linear_velocity.z = 0.0
    pose.linear_acceleration.x = 0.0
    pose.linear_acceleration.y = 0.0
    pose.linear_acceleration.z = 0.0
    pose.angular_velocity.x = 0.0
    pose.angular_velocity.y = 0.0
    pose.angular_velocity.z = 0.0
    pose.linear_acceleration_vrf.x = 0.0
    pose.linear_acceleration_vrf.y = 0.0
    pose.linear_acceleration_vrf.z = 0.0
    pose.angular_velocity_vrf.x = 0.0
    pose.angular_velocity_vrf.y = 0.0
    pose.angular_velocity_vrf.z = 0.0
    pose.euler_angles.x = 0.0
    pose.euler_angles.y = 0.0
    pose.euler_angles.z = heading
    return localization


def set_localization_heading(localization, heading):
    localization.pose.heading = heading
    localization.pose.orientation.qx = 0.0
    localization.pose.orientation.qy = 0.0
    localization.pose.orientation.qz = math.sin(heading * 0.5)
    localization.pose.orientation.qw = math.cos(heading * 0.5)
    localization.pose.euler_angles.x = 0.0
    localization.pose.euler_angles.y = 0.0
    localization.pose.euler_angles.z = heading


def make_synthetic_chassis(sequence):
    chassis = chassis_pb2.Chassis()
    fill_header(chassis.header, "yunle_indoor_planning_preflight", sequence)
    chassis.engine_started = True
    chassis.speed_mps = 0.0
    chassis.throttle_percentage = 0.0
    chassis.brake_percentage = 0.0
    chassis.steering_percentage = 0.0
    chassis.parking_brake = False
    chassis.driving_mode = chassis_pb2.Chassis.COMPLETE_MANUAL
    chassis.error_code = chassis_pb2.Chassis.NO_ERROR
    chassis.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    return chassis


def safe_cyber_shutdown(timeout_sec=2.0):
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


def planning_not_ready_reason(planning):
    if planning is None:
        return ""
    try:
        main = planning.decision.main_decision
        if main.HasField("not_ready"):
            return main.not_ready.reason
    except Exception:
        return ""
    return ""


def planning_is_valid(planning):
    if planning is None:
        return False
    if len(planning.trajectory_point) < 2:
        return False
    if planning_not_ready_reason(planning):
        return False
    if planning.HasField("estop") and planning.estop.is_estop:
        return False
    if (planning.HasField("header") and
            planning.header.HasField("status") and
            planning.header.status.error_code != 0):
        return False
    return True


def trajectory_speed_range(planning):
    values = []
    for point in planning.trajectory_point:
        if point.HasField("v"):
            values.append(point.v)
    if not values:
        return float("nan"), float("nan")
    return min(values), max(values)


def trajectory_summary(planning):
    if planning is None:
        return "no /apollo/planning message"
    status_code = 0
    status_msg = ""
    if planning.HasField("header") and planning.header.HasField("status"):
        status_code = planning.header.status.error_code
        status_msg = planning.header.status.msg
    min_v, max_v = trajectory_speed_range(planning)
    not_ready = planning_not_ready_reason(planning)
    estop = ""
    if planning.HasField("estop") and planning.estop.is_estop:
        estop = planning.estop.reason
    return (
        "points={} path_points={} total_length={:.3f}m "
        "total_time={:.3f}s v_range={:.3f}..{:.3f}m/s "
        "trajectory_type={} status={} msg={!r} not_ready={!r} estop={!r}"
    ).format(
        len(planning.trajectory_point), len(planning.path_point),
        planning.total_path_length, planning.total_path_time,
        min_v if math.isfinite(min_v) else float("nan"),
        max_v if math.isfinite(max_v) else float("nan"),
        planning.trajectory_type, status_code, status_msg, not_ready, estop)


def publish_and_monitor(request_file, timeout, publish_hz, target_speed_mps,
                        synthetic_localization, synthetic_forward_offset,
                        route_heading_localization, direct_planning_command):
    routing_request = load_request(request_file)
    synthetic_pose = None
    if synthetic_localization:
        synthetic_pose = route_start_pose(
            routing_request, synthetic_forward_offset)
        print(("synthetic_localization: x={:.3f} y={:.3f} "
               "heading={:.3f}rad").format(
                   synthetic_pose[0], synthetic_pose[1], synthetic_pose[2]))
    route_heading_value = None
    if route_heading_localization:
        route_heading_value = route_heading(routing_request)
        print(("route_heading_localization: source={} output={} "
               "heading={:.3f}rad ({:.2f}deg)").format(
                   LOCALIZATION_TOPIC, PLANNING_LOCALIZATION_TOPIC,
                   route_heading_value, math.degrees(route_heading_value)))
    cyber.init("yunle_indoor_planning_preflight")
    node = cyber.Node("yunle_indoor_planning_preflight")
    routing_monitor = TopicMonitor(routing_pb2.RoutingResponse)
    planning_monitor = TopicMonitor(planning_pb2.ADCTrajectory)
    live_localization_monitor = None
    response_topic = ("/apollo/raw_routing_response"
                      if direct_planning_command else ROUTING_RESPONSE_TOPIC)
    request_topic = ("/apollo/raw_routing_request"
                     if direct_planning_command else OLD_ROUTING_REQUEST_TOPIC)
    node.create_reader(response_topic, routing_pb2.RoutingResponse,
                       routing_monitor.callback)
    node.create_reader(PLANNING_TOPIC, planning_pb2.ADCTrajectory,
                       planning_monitor.callback)
    routing_writer = node.create_writer(request_topic,
                                        routing_pb2.RoutingRequest)
    planning_writer = None
    if direct_planning_command:
        planning_writer = node.create_writer(
            PLANNING_COMMAND_TOPIC, planning_command_pb2.PlanningCommand)
    prediction_writer = node.create_writer(PREDICTION_TOPIC,
                                           prediction_obstacle_pb2
                                           .PredictionObstacles)
    localization_writer = None
    chassis_writer = None
    if synthetic_localization:
        localization_writer = node.create_writer(
            LOCALIZATION_TOPIC, localization_pb2.LocalizationEstimate)
        chassis_writer = node.create_writer(CHASSIS_TOPIC, chassis_pb2.Chassis)
    elif route_heading_localization:
        live_localization_monitor = TopicMonitor(
            localization_pb2.LocalizationEstimate)
        node.create_reader(LOCALIZATION_TOPIC,
                           localization_pb2.LocalizationEstimate,
                           live_localization_monitor.callback)
        localization_writer = node.create_writer(
            PLANNING_LOCALIZATION_TOPIC,
            localization_pb2.LocalizationEstimate)

    period = 1.0 / publish_hz
    deadline = time.monotonic() + timeout
    sequence = 0
    planning_command = None
    planning_command_sent = False
    routing_summary_printed = False
    try:
        while time.monotonic() < deadline:
            fill_header(routing_request.header,
                        "yunle_indoor_planning_preflight", sequence)
            routing_writer.write(routing_request)

            routing_snapshot = routing_monitor.snapshot()
            response = routing_snapshot["latest"]
            if planning_command is None and response is not None:
                if not routing_summary_printed:
                    print("\n=== routing_response ===")
                    summarize_response(response)
                    routing_summary_printed = True
                if direct_planning_command:
                    planning_command = make_planning_command(
                        response, target_speed_mps, sequence)
                    planning_writer.write(planning_command)
                    planning_command_sent = True
                else:
                    planning_command_sent = True

            if synthetic_localization:
                localization_writer.write(make_synthetic_localization(
                    synthetic_pose[0], synthetic_pose[1], synthetic_pose[2],
                    sequence))
                chassis_writer.write(make_synthetic_chassis(sequence))
            elif route_heading_localization:
                localization_snapshot = live_localization_monitor.snapshot()
                live_localization = localization_snapshot["latest"]
                if live_localization is not None:
                    corrected = localization_pb2.LocalizationEstimate()
                    corrected.CopyFrom(live_localization)
                    set_localization_heading(corrected, route_heading_value)
                    localization_writer.write(corrected)

            prediction_writer.write(make_prediction(sequence))

            planning_snapshot = planning_monitor.snapshot()
            if planning_command_sent and planning_snapshot["valid_planning"]:
                return routing_monitor.snapshot(), planning_snapshot
            time.sleep(period)
            sequence += 1
    finally:
        safe_cyber_shutdown()
    return routing_monitor.snapshot(), planning_monitor.snapshot()


def summarize_planning(snapshot):
    count = snapshot["count"]
    latest = snapshot["latest"]
    valid = snapshot["valid_planning"]
    print("\n=== planning_output ===")
    print("planning_messages={} valid_messages={}".format(
        count, len(valid)))
    print("latest: {}".format(trajectory_summary(latest)))
    if valid:
        print("valid_last: {}".format(trajectory_summary(valid[-1])))


def main():
    args = parse_args()
    if args.timeout <= 0.0 or args.timeout > 120.0:
        raise ValueError("--timeout must be in (0, 120]")
    if args.publish_hz <= 0.0 or args.publish_hz > 30.0:
        raise ValueError("--publish-hz must be in (0, 30]")
    if args.target_speed_mps <= 0.0 or args.target_speed_mps > 1.0:
        raise ValueError("--target-speed-mps must be in (0, 1.0]")
    if args.synthetic_localization and args.route_heading_localization:
        raise ValueError("--synthetic-localization and "
                         "--route-heading-localization are mutually exclusive")
    if args.synthetic_localization and args.use_heading_adapter:
        raise ValueError("--synthetic-localization and "
                         "--use-heading-adapter are mutually exclusive")
    if args.route_heading_localization and args.use_heading_adapter:
        raise ValueError("--route-heading-localization and "
                         "--use-heading-adapter are mutually exclusive")

    args.log_dir.mkdir(parents=True, exist_ok=True)
    hd_map_dir, request_file = validate_extra_paths(args)

    blockers = find_blocking_processes()
    if blockers:
        print("ERROR: blocking mainboard is already running:")
        for line in blockers:
            print("  {}".format(line))
        print("Stop these before Planning preflight. Control is intentionally "
              "not allowed in this test.")
        print("RESULT: YUNLE_INDOOR_PLANNING_PREFLIGHT_FAIL")
        return 1

    wrong_ndt = find_wrong_indoor_ndt_processes()
    if wrong_ndt:
        print("ERROR: non-ZStable indoor NDT localization is running:")
        for line in wrong_ndt:
            print("  {}".format(line))
        print("Use IndoorPclOmpNdtLocalizationMap03MeasuredZStable for this "
              "baseline.")
        print("RESULT: YUNLE_INDOOR_PLANNING_PREFLIGHT_FAIL")
        return 1

    started_inputs = []
    started_mainboards = []
    success = False
    try:
        if args.synthetic_localization:
            print("synthetic-localization: physical input modules are not "
                  "started")
        elif args.skip_start_inputs:
            print("skip-start-inputs: using already-running inputs")
        else:
            for module in MODULES:
                item = launch_module(module, str(args.log_dir), args.mainboard)
                if item is not None:
                    started_inputs.append(item)

        if args.synthetic_localization:
            print("skip input diagnostics: using synthetic localization/chassis")
        elif not args.skip_input_diagnostics:
            localization_ok = run_diagnostic(
                "localization",
                [
                    "python3", LOCALIZATION_DIAG,
                    "--duration", str(args.localization_duration),
                    "--skip-log-summary",
                    "--map-dir", args.ndt_map_dir,
                ],
                "RESULT: CPU-NDT fixed-map localization topics are active",
            )
            chassis_ok = run_diagnostic(
                "chassis",
                [
                    "python3", CHASSIS_DIAG,
                    "--duration", str(args.chassis_duration),
                ],
                "RESULT: CHASSIS_READONLY_READY",
            )
            if not localization_ok or not chassis_ok:
                print("RESULT: YUNLE_INDOOR_PLANNING_PREFLIGHT_FAIL")
                return 1

        if args.direct_planning_command:
            started_mainboards.append(launch_routing(args, hd_map_dir))
        else:
            started_mainboards.append(launch_command_process(args, hd_map_dir))
        if args.use_heading_adapter:
            started_mainboards.append(launch_heading_adapter(args))
        planning_localization_topic = LOCALIZATION_TOPIC
        if args.route_heading_localization or args.use_heading_adapter:
            planning_localization_topic = PLANNING_LOCALIZATION_TOPIC
        started_mainboards.append(launch_planning(
            args, hd_map_dir, planning_localization_topic))

        routing_snapshot, planning_snapshot = publish_and_monitor(
            request_file, args.timeout, args.publish_hz,
            args.target_speed_mps, args.synthetic_localization,
            args.synthetic_forward_offset, args.route_heading_localization,
            args.direct_planning_command)
        if routing_snapshot["latest"] is None:
            print("\n=== routing_response ===")
            print("routing_messages={}".format(routing_snapshot["count"]))
            summarize_response(routing_snapshot["latest"])
        summarize_planning(planning_snapshot)

        success = bool(planning_snapshot["valid_planning"])
        print("\nRESULT: {}".format(
            "YUNLE_INDOOR_PLANNING_PREFLIGHT_PASS"
            if success else "YUNLE_INDOOR_PLANNING_PREFLIGHT_FAIL"))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as exc:
        print("ERROR: {}".format(exc))
        print("RESULT: YUNLE_INDOOR_PLANNING_PREFLIGHT_FAIL")
    finally:
        if args.keep_running:
            print("keep-running: leaving started modules active")
            for item in started_inputs:
                item["log_file"].close()
            for item in started_mainboards:
                item["log_file"].close()
        else:
            for item in reversed(started_mainboards):
                stop_process_group(item)
            stop_started(started_inputs)
    return 0 if success else 1


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.default_int_handler)
    sys.exit(main())
