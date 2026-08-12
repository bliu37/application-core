#!/usr/bin/env python3

"""Create a tile_map_images_creator config for a saved Yunle indoor map."""

import argparse
from pathlib import Path


DEFAULT_POINT_CLOUD_CHANNEL = "/apollo/sensor/lslidar16v4/PointCloud2"
DEFAULT_LOCALIZATION_CHANNEL = "/apollo/localization/pose"
DEFAULT_BASE_MAP_ROOT = "modules/map_creator/map_editor/data/base_map"

KNOWN_WORKSPACE_PREFIXES = (
    "/apollo_workspace",
    "/home/yunle/application-core",
)


def find_workspace_root():
    script_path = Path(__file__).resolve()
    for parent in (script_path.parent,) + tuple(script_path.parents):
        if (parent / "modules").is_dir() and (parent / "profiles").is_dir():
            return parent
    return Path.cwd().resolve()


def workspace_relative(path_text):
    for prefix in KNOWN_WORKSPACE_PREFIXES:
        if path_text == prefix:
            return Path()
        if path_text.startswith(prefix + "/"):
            return Path(path_text[len(prefix) + 1:])
    return None


def resolve_path(path_text, workspace_root):
    path = Path(path_text).expanduser()
    if path.is_absolute():
        if path.exists():
            return path.resolve()
        relative = workspace_relative(str(path))
        if relative is not None:
            return (workspace_root / relative).resolve()
        return path

    cwd_candidate = (Path.cwd() / path).resolve()
    if cwd_candidate.exists():
        return cwd_candidate
    return (workspace_root / path).resolve()


def apollo_workspace_path(path, workspace_root):
    path = Path(path).resolve()
    try:
        relative = path.relative_to(workspace_root)
    except ValueError:
        return str(path)
    return "/apollo_workspace/{}".format(relative.as_posix())


def parse_args():
    workspace_root = find_workspace_root()
    parser = argparse.ArgumentParser(
        description=(
            "Write an image_creator_conf.pb.txt for the map_creator tile "
            "image generator. The default config uses SLAM pose mode: point "
            "clouds are read from record files and poses are read from "
            "slam_pose_result.bin."
        )
    )
    parser.add_argument(
        "--map-dir",
        required=True,
        help=(
            "saved map work directory containing record/ and "
            "slam_pose_result.bin."
        ),
    )
    parser.add_argument(
        "--map-name",
        default="",
        help="base-map name shown in map_creator. Defaults to map-dir name.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="output pb.txt path. Defaults to <map-dir>/image_creator_conf.pb.txt.",
    )
    parser.add_argument(
        "--record-dir",
        default="",
        help="record directory. Defaults to <map-dir>/record.",
    )
    parser.add_argument(
        "--slam-pose",
        default="",
        help="slam_pose_result.bin path. Defaults to <map-dir>/slam_pose_result.bin.",
    )
    parser.add_argument(
        "--base-map-root",
        default=DEFAULT_BASE_MAP_ROOT,
        help="map_editor data/base_map root.",
    )
    parser.add_argument(
        "--image-dir",
        default="",
        help=(
            "tile image output directory. Defaults to "
            "<base-map-root>/<map-name>/map_images."
        ),
    )
    parser.add_argument(
        "--bin-dir",
        default="",
        help="temporary tile matrix directory. Defaults to <map-dir>/map_bin.",
    )
    parser.add_argument(
        "--point-cloud-channel",
        default=DEFAULT_POINT_CLOUD_CHANNEL,
        help="record point-cloud channel.",
    )
    parser.add_argument(
        "--localization-channel",
        default=DEFAULT_LOCALIZATION_CHANNEL,
        help="localization channel for non-SLAM mode compatibility.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print paths and generated content without writing the file.",
    )
    parser.set_defaults(workspace_root=workspace_root)
    return parser.parse_args()


def validate_paths(map_dir, record_dir, slam_pose):
    if not map_dir.is_dir():
        raise FileNotFoundError("map-dir not found: {}".format(map_dir))
    if not record_dir.is_dir():
        raise FileNotFoundError("record directory not found: {}".format(record_dir))
    if not slam_pose.is_file():
        raise FileNotFoundError("slam pose file not found: {}".format(slam_pose))


def build_config(args, map_dir, record_dir, slam_pose, bin_dir, image_dir):
    workspace_root = args.workspace_root
    return """reader_conf {{
   point_cloud_channel: "{point_cloud_channel}"
   localization_channel: "{localization_channel}"
}}

traffic_light_detection_conf {{
    camera_frame_id: "front_6mm"
    camera_channel: "/apollo/sensor/camera/front_6mm/image"
    camera_intrinsic_file_path: "/apollo/modules/perception/data/params/front_6mm_intrinsics.yaml"
    car2stopline_max_distance: 5
    car2stopline_max_deg: 45
}}

point_cloud_processing_conf {{
    intensity_converter_conf {{
        enable_sigmoid_converter: true
        intensity_sigmoid_k: 0.35
        intensity_sigmoid_x0: 15.0
    }}
    vertical_segmentation {{
        enable_height_relative_converter: false
        height_var_k: 250.0
        height_count_threshold: 10
        height_var_threshold: 0.1
    }}

    filter_conf {{
        enable_height_filter: true
        upper_height_limit_relative_to_pose: 1.5
        lower_height_limit_relative_to_pose: -1.0

        enable_distance_filter: true
        upper_distance_limit: 20.0
        lower_distance_limit: 0.3
    }}
}}

matrix_generator_conf {{
    matrix_resolution: 0.03125
    matrix_id: 4
}}

input_output_conf {{
    input_dir: "{record_dir}"
    bin_output_dir: "{bin_dir}"
    images_output_dir: "{image_dir}"
    use_LRU_cache: true
    LRU_cache_size: 20
}}

slam_mode_selection_conf {{
    enable_slam_mode: true
    slam_pose_path: "{slam_pose}"
}}

coordinate_transformer_conf {{
    worker_num: 12
}}

sample_distance: 0.0

debug_conf {{
    enable_pcd_file_output: false
    debug_pcd_file_path: "{map_dir}/debug_tile_points.pcd"
}}
""".format(
        point_cloud_channel=args.point_cloud_channel,
        localization_channel=args.localization_channel,
        record_dir=apollo_workspace_path(record_dir, workspace_root),
        bin_dir=apollo_workspace_path(bin_dir, workspace_root),
        image_dir=apollo_workspace_path(image_dir, workspace_root),
        slam_pose=apollo_workspace_path(slam_pose, workspace_root),
        map_dir=apollo_workspace_path(map_dir, workspace_root),
    )


def main():
    args = parse_args()
    workspace_root = args.workspace_root
    map_dir = resolve_path(args.map_dir, workspace_root)
    map_name = args.map_name or map_dir.name
    record_dir = resolve_path(args.record_dir, workspace_root) if args.record_dir else map_dir / "record"
    slam_pose = resolve_path(args.slam_pose, workspace_root) if args.slam_pose else map_dir / "slam_pose_result.bin"
    output = resolve_path(args.output, workspace_root) if args.output else map_dir / "image_creator_conf.pb.txt"
    base_map_root = resolve_path(args.base_map_root, workspace_root)
    image_dir = resolve_path(args.image_dir, workspace_root) if args.image_dir else base_map_root / map_name / "map_images"
    bin_dir = resolve_path(args.bin_dir, workspace_root) if args.bin_dir else map_dir / "map_bin"

    validate_paths(map_dir, record_dir, slam_pose)
    config = build_config(args, map_dir, record_dir, slam_pose, bin_dir, image_dir)

    if args.dry_run:
        print("RESULT: dry run completed; no files were changed.")
        print("workspace_root={}".format(workspace_root))
        print("map_dir={}".format(map_dir))
        print("map_name={}".format(map_name))
        print("output={}".format(output))
        print("image_dir={}".format(image_dir))
        print()
        print(config.rstrip())
        return

    output.parent.mkdir(parents=True, exist_ok=True)
    image_dir.mkdir(parents=True, exist_ok=True)
    bin_dir.mkdir(parents=True, exist_ok=True)
    output.write_text(config, encoding="utf-8")

    print("RESULT: wrote tile_map_images_creator config.")
    print("config={}".format(output))
    print("map_name={}".format(map_name))
    print("image_dir={}".format(image_dir))
    print()
    print("Run with Apollo static TF modules active:")
    print("tile_map_images_creator \\")
    print("  -c {} \\".format(apollo_workspace_path(output, workspace_root)))
    print("  -i {} \\".format(apollo_workspace_path(record_dir, workspace_root)))
    print("  -o {}".format(apollo_workspace_path(image_dir, workspace_root)))


if __name__ == "__main__":
    main()
