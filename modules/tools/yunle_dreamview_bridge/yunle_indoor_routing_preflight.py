#!/usr/bin/env python3

"""Start Apollo Routing on the Yunle indoor HDMap and verify a route response."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time

from google.protobuf import text_format

APOLLO_DISTRIBUTION_HOME = "/opt/apollo/neo"
APOLLO_PYTHON = os.path.join(APOLLO_DISTRIBUTION_HOME, "python")
if os.path.isdir(APOLLO_PYTHON) and APOLLO_PYTHON not in sys.path:
    sys.path.insert(0, APOLLO_PYTHON)

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.routing_msgs import routing_pb2


DEFAULT_MAP_DIR = Path("/apollo/modules/map/data/yunle_indoor_map03_minimal")
DEFAULT_MAINBOARD = "/opt/apollo/neo/bin/mainboard"
DEFAULT_LOG_DIR = Path("/apollo_workspace/data/log/yunle_routing_preflight")
APOLLO_ROOT = "/apollo"
RAW_ROUTING_REQUEST_TOPIC = "/apollo/raw_routing_request"
ROUTING_RESPONSE_TOPIC = "/apollo/raw_routing_response"
ROUTING_RESPONSE_HISTORY_TOPIC = "/apollo/raw_rrouting_response_history"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run a non-Dreamview Routing preflight against the generated "
            "Yunle indoor HDMap. This does not start Planning, Control, or "
            "any vehicle receiver."))
    parser.add_argument("--map-dir", type=Path, default=DEFAULT_MAP_DIR)
    parser.add_argument("--routing-test-file", type=Path,
                        help="RoutingRequest text proto; defaults to "
                        "<map-dir>/routing_test.pb.txt")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--publish-hz", type=float, default=2.0)
    parser.add_argument("--routing-response-topic",
                        default=ROUTING_RESPONSE_TOPIC)
    parser.add_argument("--mainboard",
                        default=os.environ.get("MAINBOARD_BIN",
                                               DEFAULT_MAINBOARD))
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--keep-running", action="store_true",
                        help="leave the Routing mainboard running")
    parser.add_argument("--skip-start", action="store_true",
                        help="only publish/listen against an already-running "
                        "Routing module")
    return parser.parse_args()


def apollo_process_env():
    env = os.environ.copy()
    env.setdefault("APOLLO_PATH", APOLLO_ROOT)
    env.setdefault("APOLLO_ROOT_DIR", APOLLO_ROOT)
    env.setdefault("CYBER_PATH", os.path.join(APOLLO_ROOT, "cyber"))
    env.setdefault("APOLLO_DISTRIBUTION_HOME", APOLLO_DISTRIBUTION_HOME)
    env.setdefault("APOLLO_LIB_PATH",
                   os.path.join(APOLLO_DISTRIBUTION_HOME, "lib"))
    env.setdefault("APOLLO_CONF_PATH", APOLLO_ROOT)
    env.setdefault("APOLLO_FLAG_PATH", APOLLO_ROOT)
    env.setdefault("APOLLO_DAG_PATH", APOLLO_ROOT)
    env.setdefault("APOLLO_LAUNCH_PATH", APOLLO_ROOT)
    env.setdefault("APOLLO_RUNTIME_PATH", APOLLO_ROOT)
    env.setdefault("APOLLO_PLUGIN_SEARCH_IN_BAZEL_OUTPUT", "0")
    env.setdefault(
        "APOLLO_PLUGIN_INDEX_PATH",
        os.path.join(APOLLO_DISTRIBUTION_HOME, "share/cyber_plugin_index"))
    env.setdefault("APOLLO_PLUGIN_LIB_PATH",
                   os.path.join(APOLLO_DISTRIBUTION_HOME, "lib"))
    env.setdefault("APOLLO_PLUGIN_DESCRIPTION_PATH",
                   "/apollo_workspace:" + APOLLO_DISTRIBUTION_HOME)
    env["PATH"] = (os.path.join(APOLLO_DISTRIBUTION_HOME, "bin") + ":" +
                   env.get("PATH", ""))
    env["PYTHONPATH"] = (
        os.path.join(APOLLO_DISTRIBUTION_HOME, "python") + ":" +
        env.get("PYTHONPATH", ""))
    return env


def run_text(command):
    result = subprocess.run(command, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            universal_newlines=True)
    return result.returncode, result.stdout


def existing_mainboards():
    _, output = run_text(["pgrep", "-af", "mainboard"])
    lines = []
    for line in output.splitlines():
        if "pgrep -af mainboard" in line:
            continue
        lines.append(line)
    return lines


def process_running(process_group):
    needle = "-p {}".format(process_group)
    return any(needle in line for line in existing_mainboards())


def validate_paths(args):
    map_dir = args.map_dir.resolve()
    request_file = (args.routing_test_file or
                    (map_dir / "routing_test.pb.txt")).resolve()
    missing = []
    for name in ("base_map.bin", "routing_map.bin"):
        if not (map_dir / name).is_file():
            missing.append(str(map_dir / name))
    if not request_file.is_file():
        missing.append(str(request_file))
    if not os.path.exists(args.mainboard):
        missing.append(args.mainboard)
    if missing:
        raise RuntimeError("missing required files:\n  " +
                           "\n  ".join(missing))
    return map_dir, request_file


def write_routing_runtime_files(temp_dir, map_dir):
    flagfile = temp_dir / "routing_yunle_indoor.conf"
    flagfile.write_text(
        "--routing_conf_file=/apollo/modules/routing/conf/routing_config.pb.txt\n"
        "--map_dir={}\n"
        "--base_map_filename=base_map.bin|base_map.txt\n"
        "--routing_map_filename=routing_map.bin|routing_map.txt\n"
        "--routing_response_topic={}\n"
        "--routing_response_history_topic={}\n".format(
            map_dir, ROUTING_RESPONSE_TOPIC, ROUTING_RESPONSE_HISTORY_TOPIC),
        encoding="utf-8")
    dag = temp_dir / "routing_yunle_indoor.dag"
    dag.write_text(
        "module_config {\n"
        "  module_library: \"modules/routing/librouting_component.so\"\n"
        "  components {\n"
        "    class_name: \"RoutingComponent\"\n"
        "    config {\n"
        "      name: \"routing\"\n"
        "      config_file_path: \"/apollo/modules/routing/conf/"
        "routing_config.pb.txt\"\n"
        "      flag_file_path: \"" + str(flagfile) + "\"\n"
        "      readers: [\n"
        "        {\n"
        "          channel: \"" + RAW_ROUTING_REQUEST_TOPIC + "\"\n"
        "          qos_profile: { depth: 10 }\n"
        "        }\n"
        "      ]\n"
        "    }\n"
        "  }\n"
        "}\n",
        encoding="utf-8")
    return dag, flagfile


def launch_routing(args, map_dir, log_dir):
    temp_dir = Path(tempfile.mkdtemp(prefix="yunle_routing_preflight_"))
    dag, flagfile = write_routing_runtime_files(temp_dir, map_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = (log_dir / "routing_mainboard.log").open("w")
    command = [
        args.mainboard,
        "-p", "yunle_indoor_routing_preflight",
        "-d", str(dag),
    ]
    print("start Routing: {}".format(" ".join(command)))
    print("routing_flagfile={}".format(flagfile))
    process = subprocess.Popen(
        command, stdout=log_file, stderr=subprocess.STDOUT,
        env=apollo_process_env(), preexec_fn=os.setsid)
    return {
        "process": process,
        "log_file": log_file,
        "temp_dir": temp_dir,
    }


def stop_started(started):
    if started is None:
        return
    process = started["process"]
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                pass
    started["log_file"].close()


class RoutingResponseMonitor(object):

    def __init__(self):
        self._lock = threading.Lock()
        self._latest = None
        self._count = 0

    def callback(self, message):
        copied = routing_pb2.RoutingResponse()
        copied.CopyFrom(message)
        with self._lock:
            self._latest = copied
            self._count += 1

    def snapshot(self):
        with self._lock:
            if self._latest is None:
                return self._count, None
            copied = routing_pb2.RoutingResponse()
            copied.CopyFrom(self._latest)
            return self._count, copied


def load_request(request_file):
    request = routing_pb2.RoutingRequest()
    text_format.Merge(request_file.read_text(encoding="utf-8"), request)
    if len(request.waypoint) < 2:
        raise RuntimeError("{} has fewer than two waypoints".format(
            request_file))
    request.header.module_name = "yunle_indoor_routing_preflight"
    return request


def status_ok(response):
    if response is None:
        return False
    if response.HasField("status"):
        return response.status.error_code == 0
    return len(response.road) > 0


def summarize_response(response):
    if response is None:
        print("routing_response: no response")
        return
    if response.HasField("status"):
        print("routing_status: error_code={} msg={!r}".format(
            response.status.error_code, response.status.msg))
    else:
        print("routing_status: no status field")
    if response.HasField("measurement"):
        print("routing_distance={:.3f}m".format(
            response.measurement.distance))
    print("routing_roads={} request_waypoints={}".format(
        len(response.road), len(response.routing_request.waypoint)
        if response.HasField("routing_request") else 0))
    for road_index, road in enumerate(response.road):
        print("  road[{}] id={!r} passages={}".format(
            road_index, road.id, len(road.passage)))
        for passage_index, passage in enumerate(road.passage):
            lane_ids = [
                "{}:{:.3f}->{:.3f}".format(
                    segment.id, segment.start_s, segment.end_s)
                for segment in passage.segment
            ]
            print("    passage[{}] {}".format(
                passage_index, ", ".join(lane_ids)))


def publish_and_wait(request_file, response_topic, timeout, publish_hz):
    request = load_request(request_file)
    cyber.init("yunle_indoor_routing_preflight")
    monitor = RoutingResponseMonitor()
    node = cyber.Node("yunle_indoor_routing_preflight")
    node.create_reader(response_topic, routing_pb2.RoutingResponse,
                       monitor.callback)
    writer = node.create_writer(RAW_ROUTING_REQUEST_TOPIC,
                                routing_pb2.RoutingRequest)
    period = 1.0 / publish_hz
    deadline = time.monotonic() + timeout
    sequence = 0
    try:
        while time.monotonic() < deadline:
            request.header.sequence_num = sequence
            request.header.timestamp_sec = time.time()
            writer.write(request)
            sequence += 1
            time.sleep(period)
            count, response = monitor.snapshot()
            if count > 0:
                return count, response
    finally:
        cyber.shutdown()
    count, response = monitor.snapshot()
    return count, response


def main():
    args = parse_args()
    if args.timeout <= 0.0 or args.timeout > 60.0:
        raise ValueError("--timeout must be in (0, 60]")
    if args.publish_hz <= 0.0 or args.publish_hz > 20.0:
        raise ValueError("--publish-hz must be in (0, 20]")

    try:
        map_dir, request_file = validate_paths(args)
        if not args.skip_start and process_running(
                "yunle_indoor_routing_preflight"):
            raise RuntimeError(
                "yunle_indoor_routing_preflight is already running")
        started = None
        if not args.skip_start:
            started = launch_routing(args, map_dir, args.log_dir)
            time.sleep(2.0)
            if started["process"].poll() is not None:
                raise RuntimeError(
                    "Routing mainboard exited early; see {}".format(
                        args.log_dir / "routing_mainboard.log"))
        count, response = publish_and_wait(
            request_file, args.routing_response_topic, args.timeout,
            args.publish_hz)
        print("responses={}".format(count))
        summarize_response(response)
        if status_ok(response):
            print("RESULT: YUNLE_INDOOR_ROUTING_PREFLIGHT_PASS")
            return 0
        print("RESULT: YUNLE_INDOOR_ROUTING_PREFLIGHT_FAIL")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1
    except Exception as exc:
        print("ERROR: {}".format(exc))
        print("RESULT: YUNLE_INDOOR_ROUTING_PREFLIGHT_FAIL")
        return 1
    finally:
        if "started" in locals() and started is not None:
            if args.keep_running:
                print("keep-running: leaving Routing active")
                started["log_file"].close()
            else:
                stop_started(started)


if __name__ == "__main__":
    sys.exit(main())
