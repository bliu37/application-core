#!/usr/bin/env python3

"""Start read-only indoor navigation inputs and run localization/chassis checks."""

import argparse
import os
import signal
import subprocess
import sys
import time


DEFAULT_MAP_DIR = (
    "/apollo_workspace/data/map_work/yunle_indoor/"
    "jd03_indoor_20260803_03/ndt_map/local_map_float_safe"
)
DEFAULT_MAINBOARD = "/opt/apollo/neo/bin/mainboard"
APOLLO_DISTRIBUTION_HOME = "/opt/apollo/neo"
APOLLO_ROOT = "/apollo"

LOCALIZATION_DIAG = (
    "/apollo_workspace/modules/tools/yunle_dreamview_bridge/"
    "yunle_indoor_ndt_localization_diagnostic.py"
)
CHASSIS_DIAG = (
    "/apollo_workspace/modules/canbus_vehicle/yunle/tools/"
    "yunle_chassis_diagnostic.py"
)

MODULES = [
    {
        "name": "LslidarC16V4",
        "process": "lslidar",
        "dags": ["/apollo/modules/drivers/lidar/lslidar/dag/lslidar.dag"],
        "settle_sec": 3.0,
    },
    {
        "name": "N100Imu",
        "process": "n100_imu",
        "dags": [
            "/apollo/modules/tools/yunle_dreamview_bridge/dag/"
            "n100_imu_driver.dag"
        ],
        "settle_sec": 2.0,
    },
    {
        "name": "IndoorTestTf",
        "process": "yunle_indoor_test_static_transform",
        "dags": [
            "/apollo/modules/transform/dag/"
            "yunle_indoor_test_static_transform.dag"
        ],
        "settle_sec": 1.0,
    },
    {
        "name": "IndoorLiorfOdometry",
        "process": "yunle_indoor_liorf_odometry",
        "dags": [
            "/apollo/modules/loam_velodyne_indoor/dag/"
            "laser_multiscan_registration.dag",
            "/apollo/modules/loam_velodyne_indoor/dag/laser_odometry.dag",
            "/apollo/modules/loam_velodyne_indoor/dag/localization_mapping.dag",
        ],
        "settle_sec": 8.0,
    },
    {
        "name": "IndoorPclOmpNdtLocalizationMap03MeasuredZStable",
        "process": (
            "yunle_indoor_pclomp_ndt_localization_map03_measured_z_stable"
        ),
        "dags": [
            "/apollo/modules/tools/yunle_dreamview_bridge/dag/"
            "indoor_pclomp_ndt_map03_measured_zstable_tools.dag",
        ],
        "settle_sec": 12.0,
        "start_retries": 2,
    },
    {
        "name": "YunleChassisPreview",
        "process": "yunle_chassis_receiver",
        "dags": [
            "/apollo/modules/canbus_vehicle/yunle/dag/"
            "yunle_chassis_receiver.dag"
        ],
        "settle_sec": 2.0,
    },
]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Launch read-only indoor localization/chassis modules without "
            "Dreamview and run diagnostics. This script does not start "
            "Control or YunleApolloControlTest."
        ))
    parser.add_argument("--map-dir", default=DEFAULT_MAP_DIR)
    parser.add_argument("--localization-duration", type=float, default=30.0)
    parser.add_argument("--chassis-duration", type=float, default=10.0)
    parser.add_argument("--startup-wait", type=float, default=2.0,
                        help="extra wait after all modules are launched")
    parser.add_argument("--skip-start", action="store_true",
                        help="only run diagnostics against already-running modules")
    parser.add_argument("--keep-running", action="store_true",
                        help="do not stop modules started by this script")
    parser.add_argument("--log-dir",
                        default="/apollo_workspace/data/log/yunle_preflight")
    parser.add_argument("--mainboard",
                        default=os.environ.get("MAINBOARD_BIN",
                                               DEFAULT_MAINBOARD))
    return parser.parse_args()


def run_text(command, check=False, env=None):
    result = subprocess.run(command, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, universal_newlines=True,
                            env=env)
    if check and result.returncode != 0:
        raise RuntimeError("{} failed:\n{}".format(
            " ".join(command), result.stdout))
    return result.returncode, result.stdout


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
    for line in existing_mainboards():
        if needle in line:
            return True
    return False


def validate_paths(mainboard):
    missing = []
    if not os.path.exists(mainboard):
        missing.append(mainboard)
    for module in MODULES:
        for dag in module["dags"]:
            if not os.path.exists(dag):
                missing.append(dag)
    for path in (LOCALIZATION_DIAG, CHASSIS_DIAG):
        if not os.path.exists(path):
            missing.append(path)
    if missing:
        raise RuntimeError("missing required files:\n  " + "\n  ".join(missing))


def launch_module(module, log_dir, mainboard):
    if process_running(module["process"]):
        print("reuse: {} ({})".format(module["name"], module["process"]))
        return None

    attempts = int(module.get("start_retries", 0)) + 1
    command = [mainboard, "-p", module["process"]]
    for dag in module["dags"]:
        command.extend(["-d", dag])
    log_path = os.path.join(log_dir, module["process"] + ".log")

    last_returncode = None
    for attempt in range(1, attempts + 1):
        log_file = open(log_path, "a")
        suffix = "" if attempts == 1 else " attempt {}/{}".format(
            attempt, attempts)
        print("start: {} ({}){}".format(
            module["name"], module["process"], suffix))
        print("  log={}".format(log_path))
        process = subprocess.Popen(command, stdout=log_file,
                                   stderr=subprocess.STDOUT,
                                   cwd="/apollo_workspace",
                                   env=apollo_process_env())
        time.sleep(module["settle_sec"])
        if process.poll() is None:
            return {"module": module, "process": process, "log_file": log_file}

        last_returncode = process.returncode
        log_file.close()
        if attempt < attempts:
            print("retry: {} exited early with code {}".format(
                module["name"], last_returncode))
            time.sleep(1.0)

    raise RuntimeError("{} exited early with code {}".format(
        module["name"], last_returncode))


def stop_started(started):
    for item in reversed(started):
        process = item["process"]
        module = item["module"]
        if process.poll() is None:
            print("stop: {} ({})".format(module["name"], module["process"]))
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
        item["log_file"].close()


def run_diagnostic(label, command, expected_result):
    print("\n=== {} ===".format(label))
    print("cmd: {}".format(" ".join(command)))
    returncode, output = run_text(command, env=apollo_process_env())
    sys.stdout.write(output)
    if output and not output.endswith("\n"):
        print("")
    ok = returncode == 0 and expected_result in output
    if ok:
        print("{}: PASS".format(label))
    else:
        print("{}: FAIL".format(label))
    return ok


def main():
    args = parse_args()
    validate_paths(args.mainboard)
    os.makedirs(args.log_dir, exist_ok=True)

    started = []
    success = False
    try:
        if args.skip_start:
            print("skip-start: diagnostics only")
        else:
            for module in MODULES:
                item = launch_module(module, args.log_dir, args.mainboard)
                if item is not None:
                    started.append(item)
            if args.startup_wait > 0.0:
                print("wait: {:.1f}s for topic settling".format(
                    args.startup_wait))
                time.sleep(args.startup_wait)

        localization_ok = run_diagnostic(
            "localization",
            [
                "python3", LOCALIZATION_DIAG,
                "--duration", str(args.localization_duration),
                "--skip-log-summary",
                "--map-dir", args.map_dir,
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
        success = localization_ok and chassis_ok
        print("\nRESULT: {}".format(
            "READONLY_PREFLIGHT_PASS" if success else "READONLY_PREFLIGHT_FAIL"))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as exc:
        print("ERROR: {}".format(exc))
    finally:
        if args.keep_running:
            print("keep-running: leaving started modules active")
            for item in started:
                item["log_file"].close()
        else:
            stop_started(started)
    return 0 if success else 1


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.default_int_handler)
    sys.exit(main())
