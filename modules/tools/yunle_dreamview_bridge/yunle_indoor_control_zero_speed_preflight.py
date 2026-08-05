#!/usr/bin/env python3

"""Run a zero-speed Apollo Control chain preflight without Dreamview."""

import argparse
import os
import re
import subprocess
import sys
import time

from yunle_indoor_navigation_readonly_preflight import (
    CHASSIS_DIAG,
    DEFAULT_MAP_DIR,
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


PLANNING_PUBLISHER = (
    "/apollo_workspace/modules/canbus_vehicle/yunle/tools/"
    "yunle_planning_test_publisher.py"
)

YUNLE_APOLLO_CONTROL_TEST_MODULE = {
    "name": "YunleApolloControlTest",
    "process": "yunle_chassis_apollo_control_test",
    "dags": [
        "/apollo/modules/canbus_vehicle/yunle/dag/"
        "yunle_chassis_apollo_control_test.dag"
    ],
    "settle_sec": 2.0,
}

CONTROL_MODULE = {
    "name": "Control",
    "process": "control",
    "dags": ["/apollo/modules/control/control_component/dag/control.dag"],
    "settle_sec": 0.5,
}

CONFLICT_PROCESSES = [
    "control",
    "yunle_chassis_receiver",
    "yunle_chassis_control_test",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Start indoor localization plus Apollo Control commissioning "
            "modules, publish only a zero-speed planning trajectory, and "
            "verify that /apollo/planning -> /apollo/control -> Yunle receiver "
            "is connected."
        ))
    parser.add_argument("--map-dir", default=DEFAULT_MAP_DIR)
    parser.add_argument("--localization-duration", type=float, default=20.0)
    parser.add_argument("--chassis-duration", type=float, default=5.0)
    parser.add_argument("--planning-duration", type=float, default=3.0)
    parser.add_argument("--startup-wait", type=float, default=2.0)
    parser.add_argument("--keep-running", action="store_true")
    parser.add_argument("--log-dir",
                        default="/apollo_workspace/data/log/yunle_control_zero")
    parser.add_argument("--mainboard",
                        default=os.environ.get(
                            "MAINBOARD_BIN", "/opt/apollo/neo/bin/mainboard"))
    parser.add_argument(
        "--allow-manual-mode-check-only", action="store_true",
        help=("deprecated compatibility flag; the script now stops before "
              "publishing control unless --confirm-zero-speed-control is set"))
    parser.add_argument(
        "--confirm-zero-speed-control", action="store_true",
        help=("allow this wrapper to start Apollo Control and publish a "
              "zero-speed planning trajectory after all pre-command gates pass"))
    return parser.parse_args()


def control_modules():
    return MODULES


def validate_extra_paths(mainboard):
    validate_paths(mainboard)
    missing = []
    for module in (YUNLE_APOLLO_CONTROL_TEST_MODULE, CONTROL_MODULE):
        for dag in module["dags"]:
            if not os.path.exists(dag):
                missing.append(dag)
    if not os.path.exists(PLANNING_PUBLISHER):
        missing.append(PLANNING_PUBLISHER)
    if missing:
        raise RuntimeError("missing required files:\n  " + "\n  ".join(missing))


def find_conflicts():
    conflicts = []
    lines = existing_mainboards()
    for process_group in CONFLICT_PROCESSES:
        needle = "-p {}".format(process_group)
        for line in lines:
            if needle in line:
                conflicts.append(line)
    return conflicts


def run_chassis_gate(args):
    command = [
        "python3", CHASSIS_DIAG,
        "--duration", str(args.chassis_duration),
        "--allow-control-send-enabled",
        "--require-control-send-enabled",
        "--require-no-control-frames",
        "--require-auto-switch",
    ]
    print("\n=== chassis_control_gate ===")
    print("cmd: {}".format(" ".join(command)))
    returncode, output = run_text(command, env=apollo_process_env())
    sys.stdout.write(output)
    if output and not output.endswith("\n"):
        print("")
    ok = returncode == 0 and "RESULT: CHASSIS_READONLY_READY" in output
    if ok:
        print("chassis_control_gate: PASS")
    else:
        print("chassis_control_gate: FAIL")
    return ok, output


def detail_sent_count(label, output):
    match = re.search(r"{} detail: .* sent=([0-9]+) ".format(label), output)
    if not match:
        return None
    return int(match.group(1))


def control_speed_is_zero(label, output):
    match = re.search(
        r"{} control: .* speed_mps=([-+]?[0-9]+(?:\.[0-9]+)?)".format(label),
        output)
    if not match:
        return False
    return abs(float(match.group(1))) <= 0.02


def run_zero_planning(args, started):
    command = [
        "python3", PLANNING_PUBLISHER,
        "--speed-kph", "0.0",
        "--duration", str(args.planning_duration),
        "--trajectory-seconds", "3.0",
        "--confirm-stationary-test",
    ]
    print("\n=== zero_speed_planning ===")
    print("cmd: {}".format(" ".join(command)))
    publisher = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        universal_newlines=True, env=apollo_process_env(),
        cwd="/apollo_workspace")
    time.sleep(1.0)
    item = launch_module(CONTROL_MODULE, args.log_dir, args.mainboard)
    if item is not None:
        started.append(item)
    try:
        output, _ = publisher.communicate(timeout=args.planning_duration + 8.0)
        returncode = publisher.returncode
    except subprocess.TimeoutExpired:
        publisher.kill()
        output, _ = publisher.communicate(timeout=5.0)
        returncode = publisher.returncode if publisher.returncode is not None else 124
    sys.stdout.write(output)
    if output and not output.endswith("\n"):
        print("")
    motion_sent = detail_sent_count("motion", output)
    stop_sent = detail_sent_count("stop", output)
    ok = (
        returncode == 0 and
        "motion control: error_code=0" in output and
        "stop control: error_code=0" in output and
        control_speed_is_zero("motion", output) and
        control_speed_is_zero("stop", output) and
        "motion detail:" in output and
        "stop detail:" in output and
        "fresh=True" in output and
        "interlocks=True" in output and
        "failsafe=False" in output and
        "cmd_target_kph=0.000" in output and
        "errors=0" in output and
        motion_sent is not None and motion_sent > 0 and
        stop_sent is not None and stop_sent >= motion_sent and
        "motion control: no control" not in output and
        "stop control: no control" not in output and
        "motion detail: no Yunle chassis detail" not in output and
        "stop detail: no Yunle chassis detail" not in output and
        "Planning command finished" in output
    )
    if ok:
        print("zero_speed_planning: PASS")
    else:
        print("zero_speed_planning: FAIL")
    return ok


def main():
    args = parse_args()
    validate_extra_paths(args.mainboard)
    conflicts = find_conflicts()
    if conflicts:
        print("ERROR: conflicting chassis mainboard is already running:")
        for conflict in conflicts:
            print("  {}".format(conflict))
        print("Stop it before running the Apollo Control zero-speed preflight.")
        return 1

    os.makedirs(args.log_dir, exist_ok=True)
    started = []
    preview_receiver = None
    success = False
    try:
        for module in control_modules():
            item = launch_module(module, args.log_dir, args.mainboard)
            if item is not None:
                started.append(item)
                if module["process"] == "yunle_chassis_receiver":
                    preview_receiver = item
        if args.startup_wait > 0.0:
            print("wait: {:.1f}s for topic settling".format(args.startup_wait))
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
        if not localization_ok:
            print("\nRESULT: CONTROL_ZERO_SPEED_PREFLIGHT_FAIL")
            return 1

        if preview_receiver is not None:
            print("\nSwitching chassis receiver from read-only preview to "
                  "Apollo control commissioning mode.")
            stop_started([preview_receiver])
            started.remove(preview_receiver)
            preview_receiver = None
            time.sleep(1.0)

        item = launch_module(
            YUNLE_APOLLO_CONTROL_TEST_MODULE, args.log_dir, args.mainboard)
        if item is not None:
            started.append(item)

        chassis_gate_ok, chassis_output = run_chassis_gate(args)
        if not chassis_gate_ok:
            if "raw drive mode is" in chassis_output:
                print("\nRESULT: CONTROL_ZERO_SPEED_WAITING_FOR_AUTO_MODE")
                print("The pre-command gate should not require raw AUTO. "
                      "Review the diagnostic arguments.")
                return 0 if args.allow_manual_mode_check_only else 1
            print("\nRESULT: CONTROL_ZERO_SPEED_PREFLIGHT_FAIL")
            return 1

        if not args.confirm_zero_speed_control:
            success = True
            print("\nRESULT: CONTROL_ZERO_SPEED_READY_TO_COMMAND")
            print("No Control module was started and no zero-speed control "
                  "frame was sent. Rerun with --confirm-zero-speed-control "
                  "when the vehicle is secured and E-stop is ready.")
            return 0

        planning_ok = run_zero_planning(args, started)
        success = planning_ok
        print("\nRESULT: {}".format(
            "CONTROL_ZERO_SPEED_PREFLIGHT_PASS"
            if success else "CONTROL_ZERO_SPEED_PREFLIGHT_FAIL"))
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
    sys.exit(main())
