#!/usr/bin/env python3

"""Convert a map_creator released HDMap to float-safe coordinates and install it."""

import argparse
import datetime
import json
import math
import re
import shutil
import subprocess
import tempfile
from pathlib import Path


DEFAULT_SOURCE = (
    "modules/map_creator/map_editor/data/released_map/"
    "yunle_indoor_new_planning"
)
DEFAULT_TARGET = (
    "profiles/yunle/modules/map/data/yunle_indoor_map03_planning"
)
DEFAULT_CONVERTER = "/opt/apollo/neo/bin/editor_map_converter"
DEFAULT_VEHICLE_CONFIG = "/apollo/modules/common/data/vehicle_param.pb.txt"

REQUIRED_RELEASE_FILES = (
    "editor_map.json",
)

REQUIRED_CONVERTED_FILES = (
    "base_map.bin",
    "base_map.txt",
    "sim_map.bin",
    "sim_map.txt",
    "routing_map.bin",
    "routing_map.txt",
)

OPTIONAL_CONVERTED_FILES = (
    "editor_map.json",
    "default_end_way_point.txt",
    "routing_test.pb.txt",
)

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


def resolve_binary(binary_text, workspace_root):
    path = resolve_path(binary_text, workspace_root)
    if path.exists():
        return str(path)
    found = shutil.which(binary_text)
    return found or binary_text


def parse_args():
    workspace_root = find_workspace_root()
    parser = argparse.ArgumentParser(
        description=(
            "Take a map_creator released map, shift its editor-map base "
            "center from the original large indoor coordinates to the "
            "float-safe NDT coordinate range, regenerate Apollo HDMap files, "
            "backup the current Yunle profile map slot, and install the result."
        )
    )
    parser.add_argument(
        "--src",
        default=DEFAULT_SOURCE,
        help=(
            "map_creator released map directory containing editor_map.json. "
            "Accepts a host path, a container /apollo_workspace path, or a "
            "path relative to the workspace."
        ),
    )
    parser.add_argument(
        "--dst",
        default=DEFAULT_TARGET,
        help="target profile map slot to install into.",
    )
    parser.add_argument(
        "--converted-dir",
        default="",
        help=(
            "directory for the float-safe regenerated map. Defaults to a "
            "temporary directory under /tmp."
        ),
    )
    parser.add_argument(
        "--target-basemap-center-y",
        type=float,
        default=10000.0,
        help="new editor_map basemapCenter.y; matches local_map_float_safe.",
    )
    parser.add_argument(
        "--target-basemap-center-x",
        type=float,
        default=None,
        help=(
            "optional new editor_map basemapCenter.x. If omitted, x is kept "
            "from the released editor_map.json."
        ),
    )
    parser.add_argument(
        "--version-suffix",
        default="_float_safe",
        help="suffix appended to editor_map header.version in the converted map.",
    )
    parser.add_argument(
        "--rotate-deg",
        type=float,
        default=0.0,
        help=(
            "optional 2D rotation in degrees applied to editor geometry "
            "after the float-safe base-center shift."
        ),
    )
    parser.add_argument(
        "--rotate-origin-x",
        type=float,
        default=None,
        help=(
            "absolute map X for the rotation origin. Defaults to the target "
            "basemapCenter.x."
        ),
    )
    parser.add_argument(
        "--rotate-origin-y",
        type=float,
        default=None,
        help=(
            "absolute map Y for the rotation origin. Defaults to the target "
            "basemapCenter.y."
        ),
    )
    parser.add_argument(
        "--translate-x",
        type=float,
        default=0.0,
        help="optional final absolute map X translation in meters.",
    )
    parser.add_argument(
        "--translate-y",
        type=float,
        default=0.0,
        help="optional final absolute map Y translation in meters.",
    )
    parser.add_argument(
        "--converter-binary",
        default=DEFAULT_CONVERTER,
        help="editor_map_converter binary path.",
    )
    parser.add_argument(
        "--vehicle-config-path",
        default=DEFAULT_VEHICLE_CONFIG,
        help=(
            "vehicle_param.pb.txt passed through to editor_map_converter. "
            "Set to an empty string to skip this flag."
        ),
    )
    parser.add_argument(
        "--backup-dir",
        default="",
        help="backup directory. Defaults to <dst>.backup_YYYYmmdd_HHMMSS.",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="install without backing up the current target directory.",
    )
    parser.add_argument(
        "--skip-waypoints",
        action="store_true",
        help="do not generate routing_test.pb.txt/default_end_way_point.txt.",
    )
    parser.add_argument(
        "--no-install",
        action="store_true",
        help="only create the converted map directory; do not install to dst.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print planned paths and checks without writing files.",
    )
    parser.set_defaults(workspace_root=workspace_root)
    return parser.parse_args()


def validate_source(src):
    if not src.is_dir():
        raise FileNotFoundError("source directory not found: {}".format(src))
    missing = [name for name in REQUIRED_RELEASE_FILES if not (src / name).is_file()]
    if missing:
        raise FileNotFoundError(
            "source released map is missing: {}".format(", ".join(missing))
        )


def validate_target(dst):
    if not dst.is_dir():
        raise FileNotFoundError("target directory not found: {}".format(dst))


def load_editor_map(path):
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    center = data.get("basemapCenter")
    if not isinstance(center, dict):
        raise ValueError("{} has no basemapCenter object".format(path))
    for name in ("x", "y"):
        if name not in center:
            raise ValueError("{} basemapCenter has no {}".format(path, name))
        center[name] = float(center[name])
    return data


def convert_editor_map(data, target_x, target_y, version_suffix):
    converted = json.loads(json.dumps(data))
    center = converted["basemapCenter"]
    source_x = float(center["x"])
    source_y = float(center["y"])
    new_x = source_x if target_x is None else float(target_x)
    new_y = float(target_y)
    center["x"] = new_x
    center["y"] = new_y

    header = converted.setdefault("header", {})
    version = header.get("version")
    if version_suffix and isinstance(version, str) and not version.endswith(version_suffix):
        header["version"] = version + version_suffix

    return converted, (source_x, source_y), (new_x, new_y)


def is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def transform_xy(x, y, origin_x, origin_y, cos_theta, sin_theta, translate_x, translate_y):
    dx = x - origin_x
    dy = y - origin_y
    return (
        origin_x + cos_theta * dx - sin_theta * dy + translate_x,
        origin_y + sin_theta * dx + cos_theta * dy + translate_y,
    )


def transform_position(pos, center, origin_x, origin_y, cos_theta, sin_theta, translate_x, translate_y):
    if not isinstance(pos, dict) or not is_number(pos.get("x")) or not is_number(pos.get("y")):
        return
    abs_x = center["x"] + float(pos["x"])
    abs_y = center["y"] + float(pos["y"])
    new_abs_x, new_abs_y = transform_xy(
        abs_x,
        abs_y,
        origin_x,
        origin_y,
        cos_theta,
        sin_theta,
        translate_x,
        translate_y,
    )
    pos["x"] = new_abs_x - center["x"]
    pos["y"] = new_abs_y - center["y"]


def transform_heading(value, rotate_rad):
    if not is_number(value):
        return value
    return float(value) + rotate_rad


def apply_editor_transform(data, rotate_deg, rotate_origin_x, rotate_origin_y, translate_x, translate_y):
    if abs(rotate_deg) < 1e-12 and abs(translate_x) < 1e-12 and abs(translate_y) < 1e-12:
        return
    center = data["basemapCenter"]
    origin_x = float(center["x"] if rotate_origin_x is None else rotate_origin_x)
    origin_y = float(center["y"] if rotate_origin_y is None else rotate_origin_y)
    rotate_rad = math.radians(float(rotate_deg))
    cos_theta = math.cos(rotate_rad)
    sin_theta = math.sin(rotate_rad)

    for point in data.get("point", []):
        transform_position(
            point.get("position"),
            center,
            origin_x,
            origin_y,
            cos_theta,
            sin_theta,
            translate_x,
            translate_y,
        )

    for key in ("boundary", "roadBoundary", "road_boundary"):
        for boundary in data.get(key, []):
            controls = (
                boundary.get("controlsPosition")
                or boundary.get("controls_position")
                or []
            )
            for control in controls:
                transform_position(
                    control,
                    center,
                    origin_x,
                    origin_y,
                    cos_theta,
                    sin_theta,
                    translate_x,
                    translate_y,
                )

    for signal_key in ("trafficSignal", "traffic_signal"):
        for signal in data.get(signal_key, []):
            transform_position(
                signal.get("center"),
                center,
                origin_x,
                origin_y,
                cos_theta,
                sin_theta,
                translate_x,
                translate_y,
            )
            if "heading" in signal:
                signal["heading"] = transform_heading(signal["heading"], rotate_rad)

    for parking_key in ("parkingSpace", "parking_space"):
        for parking_space in data.get(parking_key, []):
            if "heading" in parking_space:
                parking_space["heading"] = transform_heading(
                    parking_space["heading"], rotate_rad
                )


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False)
        stream.write("\n")


def run_command(command):
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "{} failed with code {}:\n{}".format(
                " ".join(str(part) for part in command),
                result.returncode,
                result.stdout,
            )
        )
    return result.stdout


def regenerate_map(converter_binary, input_json, output_dir, vehicle_config_path):
    command = [
        converter_binary,
        "--input_json={}".format(input_json),
        "--output_dir={}".format(output_dir),
    ]
    if vehicle_config_path:
        command.append("--vehicle_config_path={}".format(vehicle_config_path))
    print("run: {}".format(" ".join(command)))
    return run_command(command)


def validate_converted_map(output_dir, target_y):
    missing = [
        name for name in REQUIRED_CONVERTED_FILES
        if not (output_dir / name).is_file()
    ]
    if missing:
        raise FileNotFoundError(
            "converted map is missing: {}".format(", ".join(missing))
        )

    xs, ys = read_xy_values(output_dir / "base_map.txt")
    if not xs or not ys:
        raise ValueError("converted base_map.txt contains no x/y values")
    max_y_delta = max(abs(value - target_y) for value in ys)
    if max_y_delta > 100000.0:
        raise ValueError(
            "converted map y coordinates still look unshifted: "
            "range=({:.3f}, {:.3f}), target_y={:.3f}".format(
                min(ys), max(ys), target_y
            )
        )
    return {
        "x_min": min(xs),
        "x_max": max(xs),
        "y_min": min(ys),
        "y_max": max(ys),
    }


def read_xy_values(path):
    xs = []
    ys = []
    pattern = re.compile(r"^\s*([xy]):\s*([-+0-9.eE]+)\s*$")
    for line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if not match:
            continue
        value = float(match.group(2))
        if match.group(1) == "x":
            xs.append(value)
        else:
            ys.append(value)
    return xs, ys


def extract_first_block(text, block_name):
    pattern = re.compile(r"(^|\n)(\s*){} \{{".format(re.escape(block_name)))
    match = pattern.search(text)
    if not match:
        return ""
    start = match.start(0) + (1 if match.group(1) else 0)
    brace_index = text.find("{", start)
    depth = 0
    for index in range(brace_index, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start:index + 1]
    return ""


def parse_first_lane_centerline(base_map_txt):
    text = base_map_txt.read_text(encoding="utf-8")
    lane_block = extract_first_block(text, "lane")
    if not lane_block:
        raise ValueError("base_map.txt has no lane block")
    id_match = re.search(r'id:\s*"([^"]+)"', lane_block)
    if not id_match:
        raise ValueError("first lane has no id")
    lane_id = id_match.group(1)
    central_curve = extract_first_block(lane_block, "central_curve")
    if not central_curve:
        raise ValueError("first lane has no central_curve")
    points = []
    point_pattern = re.compile(
        r"point\s*\{[^{}]*?x:\s*([-+0-9.eE]+)"
        r"[^{}]*?y:\s*([-+0-9.eE]+)",
        re.DOTALL,
    )
    for match in point_pattern.finditer(central_curve):
        points.append((float(match.group(1)), float(match.group(2))))
    if len(points) < 2:
        raise ValueError("first lane central_curve has fewer than two points")
    return lane_id, points


def polyline_length(points):
    return sum(
        math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
        for i in range(1, len(points))
    )


def point_at_s(points, s):
    if s <= 0.0:
        return points[0]
    remaining = s
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        segment_length = math.hypot(x1 - x0, y1 - y0)
        if segment_length <= 1e-9:
            continue
        if remaining <= segment_length:
            ratio = remaining / segment_length
            return (x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio)
        remaining -= segment_length
    return points[-1]


def write_waypoint_files(output_dir):
    lane_id, points = parse_first_lane_centerline(output_dir / "base_map.txt")
    length = polyline_length(points)
    if length <= 1e-6:
        raise ValueError("first lane length is too short for waypoints")
    start_s = 0.0 if length <= 1.0 else 0.5
    end_s = length if length <= 1.0 else max(start_s, length - 0.5)
    start_x, start_y = point_at_s(points, start_s)
    end_x, end_y = point_at_s(points, end_s)

    (output_dir / "default_end_way_point.txt").write_text(
        (
            "landmark {{\n"
            "  name: \"{lane_id}\"\n"
            "  waypoint {{\n"
            "    id: \"{lane_id}\"\n"
            "    s: {end_s:.6f}\n"
            "    pose {{\n"
            "      x: {end_x:.6f}\n"
            "      y: {end_y:.6f}\n"
            "    }}\n"
            "  }}\n"
            "}}\n"
        ).format(lane_id=lane_id, end_s=end_s, end_x=end_x, end_y=end_y),
        encoding="utf-8",
    )
    (output_dir / "routing_test.pb.txt").write_text(
        (
            "header {{\n"
            "  module_name: \"routing\"\n"
            "  sequence_num: 1\n"
            "  timestamp_sec: 0.0\n"
            "}}\n"
            "waypoint {{\n"
            "  id: \"{lane_id}\"\n"
            "  s: {start_s:.6f}\n"
            "  pose {{\n"
            "    x: {start_x:.6f}\n"
            "    y: {start_y:.6f}\n"
            "  }}\n"
            "}}\n"
            "waypoint {{\n"
            "  id: \"{lane_id}\"\n"
            "  s: {end_s:.6f}\n"
            "  pose {{\n"
            "    x: {end_x:.6f}\n"
            "    y: {end_y:.6f}\n"
            "  }}\n"
            "}}\n"
        ).format(
            lane_id=lane_id,
            start_s=start_s,
            start_x=start_x,
            start_y=start_y,
            end_s=end_s,
            end_x=end_x,
            end_y=end_y,
        ),
        encoding="utf-8",
    )
    return lane_id, length, start_s, end_s


def make_backup(dst, backup_dir, dry_run):
    if backup_dir.exists():
        raise FileExistsError("backup directory already exists: {}".format(backup_dir))
    print("backup: {} -> {}".format(dst, backup_dir))
    if not dry_run:
        shutil.copytree(str(dst), str(backup_dir), symlinks=True)


def copy_converted_files(converted_dir, dst, dry_run):
    copied = []
    for name in REQUIRED_CONVERTED_FILES + OPTIONAL_CONVERTED_FILES:
        source_file = converted_dir / name
        if not source_file.exists():
            continue
        target_file = dst / name
        print("copy: {} -> {}".format(source_file, target_file))
        if not dry_run:
            shutil.copy2(str(source_file), str(target_file))
        copied.append(name)
    return copied


def main():
    args = parse_args()
    workspace_root = args.workspace_root
    src = resolve_path(args.src, workspace_root)
    dst = resolve_path(args.dst, workspace_root)
    converter_binary = resolve_binary(args.converter_binary, workspace_root)
    vehicle_config_path = args.vehicle_config_path

    validate_source(src)
    if not args.no_install:
        validate_target(dst)

    if args.converted_dir:
        converted_dir = resolve_path(args.converted_dir, workspace_root)
    else:
        converted_dir = Path(tempfile.mkdtemp(
            prefix="yunle_hdmap_float_safe_"))

    editor_map = load_editor_map(src / "editor_map.json")
    converted_map, source_center, target_center = convert_editor_map(
        editor_map,
        args.target_basemap_center_x,
        args.target_basemap_center_y,
        args.version_suffix,
    )
    apply_editor_transform(
        converted_map,
        args.rotate_deg,
        args.rotate_origin_x,
        args.rotate_origin_y,
        args.translate_x,
        args.translate_y,
    )

    if args.dry_run:
        print("RESULT: dry run completed; no files were changed.")
        print("workspace_root={}".format(workspace_root))
        print("source={}".format(src))
        print("target={}".format(dst))
        print("converted_dir={}".format(converted_dir))
        print("converter_binary={}".format(converter_binary))
        print("source_basemap_center=({:.3f}, {:.3f})".format(*source_center))
        print("target_basemap_center=({:.3f}, {:.3f})".format(*target_center))
        print("rotate_deg={:.6f}".format(args.rotate_deg))
        print("translate=({:.6f}, {:.6f})".format(args.translate_x, args.translate_y))
        return

    converted_dir.mkdir(parents=True, exist_ok=True)
    converted_json = converted_dir / "editor_map.json"
    write_json(converted_json, converted_map)
    converter_output = regenerate_map(
        converter_binary,
        converted_json,
        converted_dir,
        vehicle_config_path,
    )
    if converter_output.strip():
        print(converter_output.strip())

    waypoint_summary = None
    if not args.skip_waypoints:
        waypoint_summary = write_waypoint_files(converted_dir)

    bounds = validate_converted_map(converted_dir, args.target_basemap_center_y)

    backup_dir = None
    if not args.no_install:
        if not args.no_backup:
            if args.backup_dir:
                backup_dir = resolve_path(args.backup_dir, workspace_root)
            else:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                backup_dir = dst.with_name(dst.name + ".backup_" + timestamp)
            make_backup(dst, backup_dir, False)
        copied = copy_converted_files(converted_dir, dst, False)
    else:
        copied = []

    print()
    if args.no_install:
        print("RESULT: converted released HDMap to float-safe coordinates.")
    else:
        print("RESULT: converted and installed released HDMap.")
    print("workspace_root={}".format(workspace_root))
    print("source={}".format(src))
    print("converted_dir={}".format(converted_dir))
    print("target={}".format(dst))
    if backup_dir is not None:
        print("backup={}".format(backup_dir))
    print("source_basemap_center=({:.3f}, {:.3f})".format(*source_center))
    print("target_basemap_center=({:.3f}, {:.3f})".format(*target_center))
    print("rotate_deg={:.6f}".format(args.rotate_deg))
    print("translate=({:.6f}, {:.6f})".format(args.translate_x, args.translate_y))
    print("converted_bounds=x[{x_min:.3f},{x_max:.3f}] y[{y_min:.3f},{y_max:.3f}]".format(**bounds))
    if waypoint_summary is not None:
        lane_id, lane_length, start_s, end_s = waypoint_summary
        print("waypoints=lane_id={} lane_length={:.3f} start_s={:.3f} end_s={:.3f}".format(
            lane_id, lane_length, start_s, end_s))
    if copied:
        print("copied_files={}".format(", ".join(copied)))
    print("runtime_map_dir=/apollo/modules/map/data/yunle_indoor_map03_planning")
    print("restart Dreamview/Routing/Planning before using the updated map.")


if __name__ == "__main__":
    main()
