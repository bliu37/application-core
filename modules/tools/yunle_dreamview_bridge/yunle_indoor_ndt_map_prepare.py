#!/usr/bin/env python3

"""Prepare a saved Yunle LIORF map as input for Apollo's NDT map creator."""

import argparse
import math
import os
from pathlib import Path


DEFAULT_MAP_DIR = Path(
    "/apollo_workspace/data/map_work/yunle_indoor/jd03_indoor_20260803_01")
NDT_MAP_OFFSET = (10000.0, 10000.0, 0.0)
NDT_VARIANT = "float_safe"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Create an idempotent one-frame PCD/pose input set for Apollo's "
            "CPU NDT map creator. The source global.pcd is not modified."))
    parser.add_argument("--map-dir", type=Path, default=DEFAULT_MAP_DIR,
                        help="saved LIORF map directory")
    return parser.parse_args()


def read_first_timestamp(path):
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if fields:
            timestamp = float(fields[0])
            if not math.isfinite(timestamp) or timestamp <= 0.0:
                break
            return timestamp
    raise ValueError("{} has no valid timestamp".format(path))


def validate_pcd(path):
    with path.open("rb") as stream:
        header = stream.read(4096)
    if b"FIELDS x y z" not in header or b"DATA binary" not in header:
        raise ValueError(
            "{} is not the expected binary x/y/z PCD".format(path))


def ensure_symlink(link, target):
    if link.is_symlink():
        if link.resolve() != target.resolve():
            raise RuntimeError(
                "{} already points to {}".format(link, link.resolve()))
        return
    if link.exists():
        raise RuntimeError(
            "{} already exists and is not a symlink".format(link))
    link.symlink_to(target)


def write_if_same_or_missing(path, content):
    if path.exists():
        existing = path.read_text(encoding="utf-8")
        if existing != content:
            raise RuntimeError(
                "{} already exists with different content".format(path))
        return
    path.write_text(content, encoding="utf-8")


def main():
    args = parse_args()
    map_dir = args.map_dir.resolve()
    source_pcd = map_dir / "global.pcd"
    pose_file = map_dir / "6D-Pose.txt"
    for required in (source_pcd, pose_file):
        if not required.is_file():
            raise FileNotFoundError(str(required))

    validate_pcd(source_pcd)
    offset_x, offset_y, offset_z = NDT_MAP_OFFSET
    timestamp = read_first_timestamp(pose_file)

    input_dir = map_dir / "ndt_input_{}".format(NDT_VARIANT)
    pcd_dir = input_dir / "pcds"
    pcd_dir.mkdir(parents=True, exist_ok=True)
    pcd_link = pcd_dir / "0.pcd"
    ensure_symlink(pcd_link, source_pcd)

    # global.pcd is already an accumulated LIORF map in its local frame. One
    # identity-orientation pose applies the saved artificial map translation
    # to every point without resampling or rewriting the binary PCD.
    poses_path = input_dir / "poses.txt"
    poses_line = ("0 {:.9f} {:.9f} {:.9f} {:.9f} "
                  "0.000000000 0.000000000 0.000000000 1.000000000\n").format(
                      timestamp, offset_x, offset_y, offset_z)
    write_if_same_or_missing(poses_path, poses_line)

    output_map = map_dir / "ndt_map" / "local_map_{}".format(NDT_VARIANT)
    # ndt_map_creator uses boost::create_directory (one level only), so create
    # the complete parent chain here before invoking the preinstalled binary.
    output_map.mkdir(parents=True, exist_ok=True)
    print("RESULT: NDT map-creator input is ready.")
    print("source_pcd={}".format(source_pcd))
    print("pcd_folder={}".format(pcd_dir))
    print("pose_file={}".format(poses_path))
    print("map_offset=({:.3f}, {:.3f}, {:.3f})".format(
        offset_x, offset_y, offset_z))
    print("recommended_output={}".format(output_map))
    print()
    print("Run Apollo's preinstalled CPU map creator next:")
    print("ndt_map_creator \\")
    print("  --pcd_folders {} \\".format(pcd_dir))
    print("  --pose_files {} \\".format(poses_path))
    print("  --map_folder {} \\".format(output_map))
    print("  --resolution_type single \\")
    print("  --resolution 0.25 \\")
    print("  --resolution_z 0.25 \\")
    print("  --zone_id 50 \\")
    print("  --set_road_cells false \\")
    print("  --pool_size 20")


if __name__ == "__main__":
    main()
