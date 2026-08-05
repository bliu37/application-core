#!/usr/bin/env python3

"""Generate a minimal Apollo HDMap from a saved Yunle indoor LIORF map."""

import argparse
import datetime
import math
import os
import shutil
import subprocess
from pathlib import Path


DEFAULT_MAP_DIR = Path(
    "/apollo_workspace/data/map_work/yunle_indoor/jd03_indoor_20260803_03")
DEFAULT_MAP_NAME = "yunle_indoor_map03_minimal"
DEFAULT_NDT_MAP_OFFSET = (10000.0, 10000.0, 0.0)
DEFAULT_DREAMVIEW_MAP_ROOT = Path("/apollo/modules/map/data")
DEFAULT_APOLLO_BIN = Path("/opt/apollo/neo/bin")
DEFAULT_ROUTING_CONF = Path("/apollo/modules/routing/conf/routing_config.pb.txt")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Create a one-lane Apollo HDMap from a saved indoor LIORF "
            "6D-Pose.txt and generate base/sim/routing map binaries."))
    parser.add_argument("--map-dir", type=Path, default=DEFAULT_MAP_DIR,
                        help="saved LIORF map directory")
    parser.add_argument("--map-name", default=DEFAULT_MAP_NAME,
                        help="Dreamview map directory name")
    parser.add_argument("--output-dir", type=Path,
                        help="HDMap output directory; defaults to "
                        "<map-dir>/hdmap/<map-name>")
    parser.add_argument("--dreamview-map-root", type=Path,
                        default=DEFAULT_DREAMVIEW_MAP_ROOT,
                        help="Dreamview maps_data_path root")
    parser.add_argument("--no-dreamview-sync", action="store_true",
                        help="do not copy the generated map under "
                        "Dreamview's map data root")
    parser.add_argument("--lane-id", default="yunle_lane_1")
    parser.add_argument("--road-id", default="yunle_road_1")
    parser.add_argument("--lane-width", type=float, default=3.0,
                        help="lane width in meters")
    parser.add_argument("--centerline-lateral-shift", type=float, default=0.0,
                        help="meters to shift the fitted centerline along "
                        "the lane normal before writing the map; positive is "
                        "left of the generated lane heading")
    parser.add_argument("--centerline-shift-x", type=float, default=0.0,
                        help="meters to translate the fitted centerline in "
                        "map X before writing the map")
    parser.add_argument("--centerline-shift-y", type=float, default=0.0,
                        help="meters to translate the fitted centerline in "
                        "map Y before writing the map")
    parser.add_argument("--speed-limit", type=float, default=0.8,
                        help="lane speed limit in m/s")
    parser.add_argument("--endpoint-extension", type=float, default=0.5,
                        help="meters to extend both ends of the fitted lane")
    parser.add_argument("--min-lane-length", type=float, default=0.0,
                        help="minimum generated lane length in meters; extra "
                        "length is added evenly to both ends")
    parser.add_argument("--point-spacing", type=float, default=0.4,
                        help="maximum spacing between generated lane points")
    parser.add_argument("--apollo-bin", type=Path, default=DEFAULT_APOLLO_BIN)
    parser.add_argument("--routing-conf", type=Path,
                        default=DEFAULT_ROUTING_CONF)
    parser.add_argument("--skip-tools", action="store_true",
                        help="only write base_map.txt, do not run Apollo map "
                        "generators")
    return parser.parse_args()


def parse_float_tuple(line, count, path):
    fields = line.split()
    if len(fields) < count:
        raise ValueError("{} must contain at least {} numbers".format(
            path, count))
    return tuple(float(fields[i]) for i in range(count))


def read_map_offset(path):
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            return parse_float_tuple(line, 3, path)
    raise ValueError("{} is empty".format(path))


def read_pose_points(path, source_offset, target_offset):
    points = []
    src_x, src_y, src_z = source_offset
    dst_x, dst_y, dst_z = target_offset
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        try:
            timestamp = float(fields[0])
            x = float(fields[1]) - src_x + dst_x
            y = float(fields[2]) - src_y + dst_y
            z = float(fields[3]) - src_z + dst_z
        except ValueError:
            continue
        if all(math.isfinite(value) for value in (timestamp, x, y, z)):
            points.append((x, y, z))
    if len(points) < 2:
        raise ValueError("{} does not contain enough valid poses".format(path))
    return points


def median(values):
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def fit_centerline(points, endpoint_extension, point_spacing,
                   min_lane_length=0.0):
    mean_x = sum(p[0] for p in points) / len(points)
    mean_y = sum(p[1] for p in points) / len(points)
    cov_xx = sum((p[0] - mean_x) * (p[0] - mean_x) for p in points)
    cov_xy = sum((p[0] - mean_x) * (p[1] - mean_y) for p in points)
    cov_yy = sum((p[1] - mean_y) * (p[1] - mean_y) for p in points)

    if cov_xx + cov_yy < 1e-8:
        raise ValueError("pose span is too small to create a lane")
    heading = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
    direction = (math.cos(heading), math.sin(heading))
    if direction[1] < 0.0 or (abs(direction[1]) < 1e-6 and direction[0] < 0.0):
        direction = (-direction[0], -direction[1])
        heading = math.atan2(direction[1], direction[0])

    projections = [
        (p[0] - mean_x) * direction[0] + (p[1] - mean_y) * direction[1]
        for p in points
    ]
    start_s = min(projections) - endpoint_extension
    end_s = max(projections) + endpoint_extension
    length = end_s - start_s
    if length <= 0.5:
        raise ValueError("generated lane would be too short")
    if min_lane_length > length:
        extra = 0.5 * (min_lane_length - length)
        start_s -= extra
        end_s += extra
        length = end_s - start_s

    z = median([p[2] for p in points])
    count = max(2, int(math.ceil(length / point_spacing)) + 1)
    centerline = []
    for index in range(count):
        along = start_s + length * index / (count - 1)
        centerline.append((
            mean_x + direction[0] * along,
            mean_y + direction[1] * along,
            z,
        ))
    return centerline, heading, length


def offset_points(points, heading, lateral_offset):
    normal = (-math.sin(heading), math.cos(heading))
    return [
        (x + normal[0] * lateral_offset,
         y + normal[1] * lateral_offset,
         z)
        for x, y, z in points
    ]


def shift_points_laterally(points, heading, lateral_shift):
    if abs(lateral_shift) < 1e-9:
        return points
    return offset_points(points, heading, lateral_shift)


def translate_points(points, dx, dy, dz=0.0):
    if abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9:
        return points
    return [(x + dx, y + dy, z + dz) for x, y, z in points]


def fmt(value):
    return "{:.6f}".format(value)


def point_block(point, indent):
    pad = " " * indent
    return (
        "{}point {{\n"
        "{}  x: {}\n"
        "{}  y: {}\n"
        "{}  z: {}\n"
        "{}}}\n").format(
            pad, pad, fmt(point[0]), pad, fmt(point[1]), pad, fmt(point[2]),
            pad)


def curve_block(name, points, heading, length, indent):
    pad = " " * indent
    inner = " " * (indent + 4)
    text = "{}{} {{\n{}segment {{\n{}  line_segment {{\n".format(
        pad, name, pad + "  ", inner)
    for point in points:
        text += point_block(point, indent + 6)
    text += (
        "{}  }}\n"
        "{}  s: 0.000000\n"
        "{}  start_position {{\n"
        "{}    x: {}\n"
        "{}    y: {}\n"
        "{}    z: {}\n"
        "{}  }}\n"
        "{}  heading: {}\n"
        "{}  length: {}\n"
        "{}}}\n"
        "{}}}\n").format(
            inner, inner, inner, inner, fmt(points[0][0]), inner,
            fmt(points[0][1]), inner, fmt(points[0][2]), inner, inner,
            fmt(heading), inner, fmt(length), pad + "  ", pad)
    return text


def boundary_block(name, points, heading, length, indent):
    pad = " " * indent
    text = "{}{} {{\n".format(pad, name)
    text += curve_block("curve", points, heading, length, indent + 2)
    text += "{}  length: {}\n".format(pad, fmt(length))
    text += "{}  virtual: false\n".format(pad)
    text += "{}  boundary_type {{\n".format(pad)
    text += "{}    s: 0.000000\n".format(pad)
    text += "{}    types: SOLID_WHITE\n".format(pad)
    text += "{}  }}\n".format(pad)
    text += "{}}}\n".format(pad)
    return text


def sample_blocks(name, length, width, indent):
    pad = " " * indent
    return (
        "{0}{1} {{ s: 0.000000 width: {2} }}\n"
        "{0}{1} {{ s: {3} width: {2} }}\n").format(
            pad, name, fmt(width), fmt(length))


def road_boundary_curve(edge_type, points, heading, length, indent):
    pad = " " * indent
    text = "{}edge {{\n".format(pad)
    text += curve_block("curve", points, heading, length, indent + 2)
    text += "{}  type: {}\n".format(pad, edge_type)
    text += "{}}}\n".format(pad)
    return text


def generate_base_map(args, centerline, heading, length):
    half_width = args.lane_width / 2.0
    left_boundary = offset_points(centerline, heading, half_width)
    right_boundary = offset_points(centerline, heading, -half_width)
    left = min(p[0] for p in left_boundary + right_boundary) - 1.0
    right = max(p[0] for p in left_boundary + right_boundary) + 1.0
    bottom = min(p[1] for p in left_boundary + right_boundary) - 1.0
    top = max(p[1] for p in left_boundary + right_boundary) + 1.0
    today = datetime.datetime.now().strftime("%Y%m%d")

    text = (
        "header {{\n"
        "  version: \"yunle_indoor_map03_minimal_{}\"\n"
        "  date: \"{}\"\n"
        "  projection {{\n"
        "    proj: \"+proj=tmerc +lat_0={{{{0}}}} +lon_0={{{{0}}}} +k={{{{1}}}} "
        "+ellps=WGS84 +no_defs\"\n"
        "  }}\n"
        "  district: \"yunle_indoor\"\n"
        "  left: {}\n"
        "  top: {}\n"
        "  right: {}\n"
        "  bottom: {}\n"
        "  vendor: \"yunle\"\n"
        "}}\n").format(today, today, fmt(left), fmt(top), fmt(right),
                      fmt(bottom))

    text += "lane {\n"
    text += "  id { id: \"" + args.lane_id + "\" }\n"
    text += curve_block("central_curve", centerline, heading, length, 2)
    text += boundary_block("left_boundary", left_boundary, heading, length, 2)
    text += boundary_block("right_boundary", right_boundary, heading, length, 2)
    text += "  length: {}\n".format(fmt(length))
    text += "  speed_limit: {}\n".format(fmt(args.speed_limit))
    text += "  type: CITY_DRIVING\n"
    text += "  turn: NO_TURN\n"
    text += sample_blocks("left_sample", length, half_width, 2)
    text += sample_blocks("right_sample", length, half_width, 2)
    text += "  direction: FORWARD\n"
    text += sample_blocks("left_road_sample", length, half_width, 2)
    text += sample_blocks("right_road_sample", length, half_width, 2)
    text += "}\n"

    text += "road {\n"
    text += "  id { id: \"" + args.road_id + "\" }\n"
    text += "  section {\n"
    text += "    id { id: \"" + args.road_id + "_section_1\" }\n"
    text += "    lane_id { id: \"" + args.lane_id + "\" }\n"
    text += "    boundary {\n"
    text += "      outer_polygon {\n"
    text += road_boundary_curve("LEFT_BOUNDARY", left_boundary, heading,
                                length, 8)
    text += road_boundary_curve("RIGHT_BOUNDARY", list(reversed(right_boundary)),
                                heading + math.pi, length, 8)
    text += "      }\n"
    text += "    }\n"
    text += "  }\n"
    text += "  type: PARK\n"
    text += "}\n"
    return text


def write_text(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def run_command(command):
    result = subprocess.run(command, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            universal_newlines=True)
    if result.returncode != 0:
        raise RuntimeError("{} failed:\n{}".format(
            " ".join(str(part) for part in command), result.stdout))
    return result.stdout


def run_apollo_generators(args, output_dir):
    bin_map_generator = args.apollo_bin / "bin_map_generator"
    sim_map_generator = args.apollo_bin / "sim_map_generator"
    topo_creator = args.apollo_bin / "topo_creator"
    for tool in (bin_map_generator, sim_map_generator, topo_creator):
        if not tool.exists():
            raise FileNotFoundError(str(tool))

    logs = []
    logs.append(run_command([
        str(bin_map_generator),
        "--map_dir={}".format(output_dir),
        "--base_map_filename=base_map.txt",
        "--output_dir={}".format(output_dir),
    ]))
    logs.append(run_command([
        str(sim_map_generator),
        "--map_dir={}".format(output_dir),
        "--base_map_filename=base_map.bin|base_map.txt",
        "--output_dir={}".format(output_dir),
    ]))
    logs.append(run_command([
        str(topo_creator),
        "--routing_conf_file={}".format(args.routing_conf),
        "--map_dir={}".format(output_dir),
        "--base_map_filename=base_map.bin|base_map.txt",
        "--routing_map_filename=routing_map.bin",
    ]))
    return logs


def sync_to_dreamview(output_dir, dreamview_dir):
    if dreamview_dir.exists():
        if dreamview_dir.is_symlink() or not dreamview_dir.is_dir():
            raise RuntimeError(
                "{} exists and is not a regular directory".format(
                    dreamview_dir))
        shutil.rmtree(str(dreamview_dir))
    shutil.copytree(str(output_dir), str(dreamview_dir))


def validate_outputs(output_dir, require_generated):
    required = ["base_map.txt"]
    if require_generated:
        required.extend([
            "base_map.bin",
            "sim_map.txt",
            "sim_map.bin",
            "routing_map.txt",
            "routing_map.bin",
        ])
    missing = [name for name in required if not (output_dir / name).is_file()]
    if missing:
        raise RuntimeError("missing generated files: {}".format(
            ", ".join(missing)))


def point_at_s(centerline, length, s):
    ratio = min(1.0, max(0.0, s / length))
    start = centerline[0]
    end = centerline[-1]
    return (
        start[0] + (end[0] - start[0]) * ratio,
        start[1] + (end[1] - start[1]) * ratio,
        start[2] + (end[2] - start[2]) * ratio,
    )


def routing_waypoint_block(lane_id, s, point):
    return (
        "waypoint {\n"
        "  id: \"" + lane_id + "\"\n"
        "  s: " + fmt(s) + "\n"
        "  pose {\n"
        "    x: " + fmt(point[0]) + "\n"
        "    y: " + fmt(point[1]) + "\n"
        "  }\n"
        "}\n")


def main():
    args = parse_args()
    if args.lane_width <= 0.5:
        raise ValueError("--lane-width must be greater than 0.5m")
    if args.speed_limit <= 0.0:
        raise ValueError("--speed-limit must be positive")

    map_dir = args.map_dir.resolve()
    pose_path = map_dir / "6D-Pose.txt"
    source_offset_path = map_dir / "gnss-map-offset.txt"
    if not pose_path.is_file():
        raise FileNotFoundError(str(pose_path))
    if not source_offset_path.is_file():
        raise FileNotFoundError(str(source_offset_path))

    output_dir = (args.output_dir or
                  (map_dir / "hdmap" / args.map_name)).resolve()
    source_offset = read_map_offset(source_offset_path)
    points = read_pose_points(pose_path, source_offset, DEFAULT_NDT_MAP_OFFSET)
    centerline, heading, length = fit_centerline(
        points, args.endpoint_extension, args.point_spacing,
        args.min_lane_length)
    centerline = shift_points_laterally(
        centerline, heading, args.centerline_lateral_shift)
    centerline = translate_points(
        centerline, args.centerline_shift_x, args.centerline_shift_y)
    base_map = generate_base_map(args, centerline, heading, length)

    output_dir.mkdir(parents=True, exist_ok=True)
    write_text(output_dir / "base_map.txt", base_map)
    write_text(output_dir / "default_end_way_point.txt",
               "landmark {\n"
               "  name: \"" + args.lane_id + "\"\n"
               "  waypoint {\n"
               "    id: \"" + args.lane_id + "\"\n"
               "    s: " + fmt(length) + "\n"
               "    pose {\n"
               "      x: " + fmt(centerline[-1][0]) + "\n"
               "      y: " + fmt(centerline[-1][1]) + "\n"
               "    }\n"
               "  }\n"
               "}\n")
    start_s = min(0.5, length * 0.1)
    end_s = max(length - start_s, length * 0.9)
    start_point = point_at_s(centerline, length, start_s)
    end_point = point_at_s(centerline, length, end_s)
    write_text(output_dir / "routing_test.pb.txt",
               "header {\n"
               "  module_name: \"routing\"\n"
               "  sequence_num: 1\n"
               "  timestamp_sec: 0.0\n"
               "}\n" +
               routing_waypoint_block(args.lane_id, start_s, start_point) +
               routing_waypoint_block(args.lane_id, end_s, end_point))

    logs = []
    if not args.skip_tools:
        logs = run_apollo_generators(args, output_dir)
    validate_outputs(output_dir, not args.skip_tools)

    dreamview_dir = None
    if not args.no_dreamview_sync:
        dreamview_dir = (args.dreamview_map_root / args.map_name).resolve()
        sync_to_dreamview(output_dir, dreamview_dir)
        validate_outputs(dreamview_dir, not args.skip_tools)

    print("RESULT: YUNLE_INDOOR_HDMAP_GENERATED")
    print("source_map_dir={}".format(map_dir))
    print("source_offset=({:.3f}, {:.3f}, {:.3f})".format(*source_offset))
    print("target_offset=({:.3f}, {:.3f}, {:.3f})".format(
        *DEFAULT_NDT_MAP_OFFSET))
    print("output_dir={}".format(output_dir))
    if dreamview_dir is not None:
        print("dreamview_map_dir={}".format(dreamview_dir))
        print("dreamview_map_name={}".format(args.map_name))
    print("lane_id={}".format(args.lane_id))
    print("lane_length={:.3f}m lane_width={:.3f}m speed_limit={:.3f}mps".
          format(length, args.lane_width, args.speed_limit))
    print("centerline_lateral_shift={:.3f}m".format(
        args.centerline_lateral_shift))
    print("centerline_translation=({:.3f}, {:.3f})m".format(
        args.centerline_shift_x, args.centerline_shift_y))
    print("start=({:.3f}, {:.3f}, {:.3f})".format(*centerline[0]))
    print("end=({:.3f}, {:.3f}, {:.3f})".format(*centerline[-1]))
    print("heading_deg={:.3f}".format(math.degrees(heading)))
    print("generated_files:")
    for path in sorted(output_dir.iterdir()):
        if path.is_file():
            print("  {} {} bytes".format(path, path.stat().st_size))
    if logs:
        print("apollo_tools=ok")


if __name__ == "__main__":
    main()
