#!/usr/bin/env python3

"""Create a tile_map_images_creator config for a saved Yunle indoor map."""

import argparse
import io
import sys
from pathlib import Path


DEFAULT_POINT_CLOUD_CHANNEL = "/apollo/sensor/lslidar16v4/PointCloud2"
DEFAULT_LOCALIZATION_CHANNEL = "/apollo/localization/pose"
DEFAULT_BASE_MAP_ROOT = "modules/map_creator/map_editor/data/base_map"
DEFAULT_UPPER_HEIGHT_LIMIT_RELATIVE_TO_POSE = 1.5
DEFAULT_LOWER_HEIGHT_LIMIT_RELATIVE_TO_POSE = -1.0
DEFAULT_UPPER_DISTANCE_LIMIT = 20.0
DEFAULT_LOWER_DISTANCE_LIMIT = 0.3
DEFAULT_MATRIX_RESOLUTION = 0.03125
DEFAULT_MATRIX_ID = 4
DEFAULT_WORKER_NUM = 12

KNOWN_WORKSPACE_PREFIXES = (
    "/apollo_workspace",
    "/home/yunle/application-core",
)


def configure_text_output():
    for name in ("stdout", "stderr"):
        stream = getattr(sys, name)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
        elif hasattr(stream, "buffer"):
            setattr(sys, name, io.TextIOWrapper(
                stream.buffer, encoding="utf-8", errors="replace",
                line_buffering=True))


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
        "--upper-height-limit-relative-to-pose",
        type=float,
        default=DEFAULT_UPPER_HEIGHT_LIMIT_RELATIVE_TO_POSE,
        help="upper point height filter relative to pose, in meters.",
    )
    parser.add_argument(
        "--lower-height-limit-relative-to-pose",
        type=float,
        default=DEFAULT_LOWER_HEIGHT_LIMIT_RELATIVE_TO_POSE,
        help="lower point height filter relative to pose, in meters.",
    )
    parser.add_argument(
        "--upper-distance-limit",
        type=float,
        default=DEFAULT_UPPER_DISTANCE_LIMIT,
        help="far distance filter for point cloud points, in meters.",
    )
    parser.add_argument(
        "--lower-distance-limit",
        type=float,
        default=DEFAULT_LOWER_DISTANCE_LIMIT,
        help="near distance filter for point cloud points, in meters.",
    )
    parser.add_argument(
        "--matrix-resolution",
        type=float,
        default=DEFAULT_MATRIX_RESOLUTION,
        help="tile-map matrix resolution, in meters per cell.",
    )
    parser.add_argument(
        "--matrix-id",
        type=int,
        default=DEFAULT_MATRIX_ID,
        help="tile-map matrix id.",
    )
    parser.add_argument(
        "--worker-num",
        type=int,
        default=DEFAULT_WORKER_NUM,
        help="coordinate transformer worker thread count.",
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
        raise FileNotFoundError(
            "record directory not found: {}".format(record_dir))
    if not slam_pose.is_file():
        raise FileNotFoundError("slam pose file not found: {}".format(slam_pose))


def validate_numeric_args(args):
    if args.upper_height_limit_relative_to_pose <= (
            args.lower_height_limit_relative_to_pose):
        raise ValueError(
            "upper-height-limit-relative-to-pose must be greater than "
            "lower-height-limit-relative-to-pose")
    if args.upper_distance_limit <= args.lower_distance_limit:
        raise ValueError(
            "upper-distance-limit must be greater than lower-distance-limit")
    if args.lower_distance_limit < 0.0:
        raise ValueError("lower-distance-limit must be non-negative")
    if args.matrix_resolution <= 0.0:
        raise ValueError("matrix-resolution must be positive")
    if args.matrix_id < 0:
        raise ValueError("matrix-id must be non-negative")
    if args.worker_num <= 0:
        raise ValueError("worker-num must be positive")


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
        upper_height_limit_relative_to_pose: {upper_height_limit}
        lower_height_limit_relative_to_pose: {lower_height_limit}

        enable_distance_filter: true
        upper_distance_limit: {upper_distance_limit}
        lower_distance_limit: {lower_distance_limit}
    }}
}}

matrix_generator_conf {{
    matrix_resolution: {matrix_resolution}
    matrix_id: {matrix_id}
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
    worker_num: {worker_num}
}}

sample_distance: 0.0

debug_conf {{
    enable_pcd_file_output: false
    debug_pcd_file_path: "{map_dir}/debug_tile_points.pcd"
}}
""".format(
        point_cloud_channel=args.point_cloud_channel,
        localization_channel=args.localization_channel,
        upper_height_limit=args.upper_height_limit_relative_to_pose,
        lower_height_limit=args.lower_height_limit_relative_to_pose,
        upper_distance_limit=args.upper_distance_limit,
        lower_distance_limit=args.lower_distance_limit,
        matrix_resolution=args.matrix_resolution,
        matrix_id=args.matrix_id,
        worker_num=args.worker_num,
        record_dir=apollo_workspace_path(record_dir, workspace_root),
        bin_dir=apollo_workspace_path(bin_dir, workspace_root),
        image_dir=apollo_workspace_path(image_dir, workspace_root),
        slam_pose=apollo_workspace_path(slam_pose, workspace_root),
        map_dir=apollo_workspace_path(map_dir, workspace_root),
    )


def print_change_hints(args, map_dir, map_name, record_dir, slam_pose, bin_dir,
                       image_dir, output, workspace_root):
    print()
    print("后续如果目录或硬件变化，优先改这些参数：")
    print("  --map-dir: 当前建图工作目录，现在是 {}".format(
        apollo_workspace_path(map_dir, workspace_root)))
    print("  --map-name: map_creator 里显示/选择的底图名，现在是 {}".format(
        map_name))
    print("  --point-cloud-channel: 雷达点云 topic，现在是 {}".format(
        args.point_cloud_channel))
    print("  --localization-channel: 定位 topic，现在是 {}".format(
        args.localization_channel))
    print("  --record-dir: record 输入目录，现在是 {}".format(
        apollo_workspace_path(record_dir, workspace_root)))
    print("  --slam-pose: SLAM 位姿文件，现在是 {}".format(
        apollo_workspace_path(slam_pose, workspace_root)))
    print("  --image-dir: map_creator 底图瓦片输出目录，现在是 {}".format(
        apollo_workspace_path(image_dir, workspace_root)))
    print("  --bin-dir: 中间 map_bin 输出目录，现在是 {}".format(
        apollo_workspace_path(bin_dir, workspace_root)))
    print("  --upper-height-limit-relative-to-pose / "
          "--lower-height-limit-relative-to-pose: 点云高度过滤，当前 {:.3f} / "
          "{:.3f} m".format(args.upper_height_limit_relative_to_pose,
                            args.lower_height_limit_relative_to_pose))
    print("  --upper-distance-limit / --lower-distance-limit: 点云距离过滤，"
          "当前 {:.3f} / {:.3f} m".format(args.upper_distance_limit,
                                        args.lower_distance_limit))
    print()
    print("本次生成的配置文件：")
    print("  {}".format(apollo_workspace_path(output, workspace_root)))
    print("map_creator 后端会从这个底图目录读取瓦片：")
    print("  {}".format(apollo_workspace_path(image_dir.parent,
                                             workspace_root)))


def print_tile_command(output, record_dir, image_dir, workspace_root):
    print()
    print("下一步生成底图瓦片，保持 IndoorTestTf 或对应静态 TF 模块运行后执行：")
    print("/opt/apollo/neo/bin/tile_map_images_creator \\")
    print("  -c {} \\".format(apollo_workspace_path(output, workspace_root)))
    print("  -i {} \\".format(apollo_workspace_path(record_dir, workspace_root)))
    print("  -o {}".format(apollo_workspace_path(image_dir, workspace_root)))


def main():
    configure_text_output()
    args = parse_args()
    workspace_root = args.workspace_root
    map_dir = resolve_path(args.map_dir, workspace_root)
    map_name = args.map_name or map_dir.name
    record_dir = (resolve_path(args.record_dir, workspace_root)
                  if args.record_dir else map_dir / "record")
    slam_pose = (resolve_path(args.slam_pose, workspace_root)
                 if args.slam_pose else map_dir / "slam_pose_result.bin")
    output = (resolve_path(args.output, workspace_root)
              if args.output else map_dir / "image_creator_conf.pb.txt")
    base_map_root = resolve_path(args.base_map_root, workspace_root)
    image_dir = (resolve_path(args.image_dir, workspace_root)
                 if args.image_dir else base_map_root / map_name / "map_images")
    bin_dir = (resolve_path(args.bin_dir, workspace_root)
               if args.bin_dir else map_dir / "map_bin")

    validate_numeric_args(args)
    validate_paths(map_dir, record_dir, slam_pose)
    config = build_config(args, map_dir, record_dir, slam_pose, bin_dir, image_dir)

    if args.dry_run:
        print("RESULT: dry run completed; no files were changed.")
        print("workspace_root={}".format(workspace_root))
        print("map_dir={}".format(map_dir))
        print("map_name={}".format(map_name))
        print("output={}".format(output))
        print("image_dir={}".format(image_dir))
        print_change_hints(args, map_dir, map_name, record_dir, slam_pose,
                           bin_dir, image_dir, output, workspace_root)
        print_tile_command(output, record_dir, image_dir, workspace_root)
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
    print_change_hints(args, map_dir, map_name, record_dir, slam_pose, bin_dir,
                       image_dir, output, workspace_root)
    print_tile_command(output, record_dir, image_dir, workspace_root)


if __name__ == "__main__":
    main()
