#!/usr/bin/env python3

"""Save the current Yunle indoor LIORF map into a protected test directory."""

import argparse
import datetime
import os

from cyber.python.cyber_py3 import cyber
from modules.loam_velodyne_indoor.proto import slam_service_pb2


ALLOWED_ROOT = "/apollo_workspace/data/map_work/yunle_indoor"
ALLOWED_PRECREATED_ENTRIES = ("record",)


def default_output_dir():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(ALLOWED_ROOT, "test_save_{}".format(timestamp))


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Ask the already-running indoor LIORF slam_server to save its "
            "current accumulated map. This does not stop the LIORF process."))
    parser.add_argument("--output-dir", default=default_output_dir(),
                        help="new output directory below {}".format(
                            ALLOWED_ROOT))
    parser.add_argument("--confirm-test-save", action="store_true",
                        help="required acknowledgement that this is a test map")
    return parser.parse_args()


def validate_output_dir(path):
    if not os.path.isabs(path):
        raise ValueError("output-dir must be an absolute path")

    normalized_root = os.path.realpath(ALLOWED_ROOT)
    normalized_path = os.path.realpath(path)
    try:
        common = os.path.commonpath((normalized_root, normalized_path))
    except ValueError:
        raise ValueError("output-dir is not below the allowed root")

    if common != normalized_root or normalized_path == normalized_root:
        raise ValueError(
            "output-dir must be a child directory below {}".format(
                normalized_root))
    if os.path.exists(normalized_path):
        if not os.path.isdir(normalized_path):
            raise ValueError(
                "output-dir already exists and is not a directory: {}".format(
                    normalized_path))
        unexpected_entries = []
        for name in sorted(os.listdir(normalized_path)):
            path = os.path.join(normalized_path, name)
            if name in ALLOWED_PRECREATED_ENTRIES and os.path.isdir(path):
                continue
            unexpected_entries.append(name)
        if unexpected_entries:
            raise ValueError(
                "output-dir already exists with map or unexpected content; "
                "refusing to overwrite: {} entries={}".format(
                    normalized_path, ",".join(unexpected_entries)))
    return normalized_path


def response_status_name(status):
    try:
        return slam_service_pb2.SlamResponse.Status.Name(status)
    except ValueError:
        return "UNKNOWN_{}".format(status)


def list_saved_files(output_dir):
    files = []
    if not os.path.isdir(output_dir):
        return files
    for current_dir, _, names in os.walk(output_dir):
        for name in sorted(names):
            path = os.path.join(current_dir, name)
            files.append((path, os.path.getsize(path)))
    return files


def main():
    args = parse_args()
    if not args.confirm_test_save:
        raise ValueError(
            "--confirm-test-save is required; no save request was sent")
    output_dir = validate_output_dir(args.output_dir)

    cyber.init()
    node = cyber.Node("yunle_indoor_map_save_test_{}".format(os.getpid()))
    client = node.create_client(
        "slam_server", slam_service_pb2.SlamRequest,
        slam_service_pb2.SlamResponse)

    request = slam_service_pb2.SlamRequest()
    request.command = slam_service_pb2.SlamRequest.STOP
    request.pose_path = output_dir

    print("Sending LIORF test-save request to slam_server...", flush=True)
    print("output_dir={}".format(output_dir), flush=True)
    print("The service may take some time to write the accumulated clouds.",
          flush=True)
    response = client.send_request(request)

    if response is None:
        print("RESULT: no response from slam_server.", flush=True)
        exit_code = 2
    else:
        status_name = response_status_name(response.status)
        print("response: status={} message={!r}".format(
            status_name, response.message), flush=True)
        files = list_saved_files(output_dir)
        if files:
            print("saved files:", flush=True)
            for path, size in files:
                print("  {} {} bytes".format(path, size), flush=True)
        else:
            print("saved files: none", flush=True)

        if response.status == slam_service_pb2.SlamResponse.OK and files:
            print(("RESULT: test map save completed. The LIORF process remains "
                   "running."), flush=True)
            exit_code = 0
        else:
            print("RESULT: test map save did not complete successfully.",
                  flush=True)
            exit_code = 3

    cyber.shutdown()
    os._exit(exit_code)


if __name__ == "__main__":
    main()
