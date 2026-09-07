#!/usr/bin/env python3

"""Estimate map_creator HDMap 2D alignment from a saved Yunle pose trajectory."""

import argparse
import json
import math
from pathlib import Path


DEFAULT_INSTALL_SCRIPT = (
    "modules/tools/yunle_dreamview_bridge/"
    "yunle_install_released_hdmap_float_safe.py"
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
            "Estimate --rotate-deg/--translate-x/--translate-y for "
            "yunle_install_released_hdmap_float_safe.py by matching a saved "
            "6D-Pose.txt or 3D-Pose.txt trajectory to map_creator lane "
            "centerlines."
        )
    )
    parser.add_argument(
        "--map-dir",
        default="",
        help="saved map work directory containing 6D-Pose.txt/3D-Pose.txt.",
    )
    parser.add_argument(
        "--released-map",
        required=True,
        help="map_creator released map directory containing editor_map.json.",
    )
    parser.add_argument(
        "--pose-file",
        default="",
        help="pose file. Defaults to <map-dir>/6D-Pose.txt, then 3D-Pose.txt.",
    )
    parser.add_argument(
        "--source-offset-file",
        default="",
        help="source offset file. Defaults to <map-dir>/gnss-map-offset.txt.",
    )
    parser.add_argument(
        "--pose-target-offset-x",
        type=float,
        default=10000.0,
        help="target X offset used by the float-safe NDT map.",
    )
    parser.add_argument(
        "--pose-target-offset-y",
        type=float,
        default=10000.0,
        help="target Y offset used by the float-safe NDT map.",
    )
    parser.add_argument(
        "--target-basemap-center-y",
        type=float,
        default=10000.0,
        help="float-safe basemapCenter.y used by the HDMap installer.",
    )
    parser.add_argument(
        "--target-basemap-center-x",
        type=float,
        default=None,
        help=(
            "float-safe basemapCenter.x used by the HDMap installer. If "
            "omitted, the released editor_map basemapCenter.x is kept."
        ),
    )
    parser.add_argument(
        "--lane-id",
        default="",
        help="only match this map_creator lane id. Defaults to all lanes.",
    )
    parser.add_argument(
        "--samples-per-lane",
        type=int,
        default=200,
        help="number of centerline samples used per lane.",
    )
    parser.add_argument(
        "--min-pose-step",
        type=float,
        default=0.05,
        help="drop consecutive pose samples closer than this many meters.",
    )
    parser.add_argument(
        "--min-pose-span",
        type=float,
        default=1.0,
        help="warn when first-last trajectory span is shorter than this.",
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=30,
        help="ICP refinement iterations per candidate.",
    )
    parser.add_argument(
        "--max-abs-rotate-deg",
        type=float,
        default=45.0,
        help=(
            "prefer candidates whose absolute rotation is within this range. "
            "This suppresses 180-degree false matches on nearly straight lanes."
        ),
    )
    parser.add_argument(
        "--max-abs-translate",
        type=float,
        default=10.0,
        help="prefer candidates whose |translate_x/y| are within this range.",
    )
    parser.add_argument(
        "--top-candidates",
        type=int,
        default=5,
        help="number of candidate lane matches to print.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="write JSON summary. Defaults to <map-dir>/hdmap_alignment.json.",
    )
    parser.add_argument(
        "--no-write",
        action="store_true",
        help="do not write the JSON summary.",
    )
    parser.add_argument(
        "--install-script",
        default=DEFAULT_INSTALL_SCRIPT,
        help="HDMap install script path used when printing the command.",
    )
    parser.set_defaults(workspace_root=workspace_root)
    return parser.parse_args()


def read_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def read_offset(path):
    if not path or not path.is_file():
        return None
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) >= 3:
            return tuple(float(value) for value in fields[:3])
    raise ValueError("{} has no valid offset line".format(path))


def choose_pose_file(map_dir, pose_file):
    if pose_file:
        return pose_file
    if not map_dir:
        raise ValueError("--map-dir or --pose-file is required")
    for name in ("6D-Pose.txt", "3D-Pose.txt"):
        candidate = map_dir / name
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("no 6D-Pose.txt or 3D-Pose.txt under {}".format(map_dir))


def parse_pose_line(line):
    fields = line.split()
    if len(fields) < 3:
        return None
    try:
        values = [float(field) for field in fields]
    except ValueError:
        return None
    if len(values) >= 4 and values[0] > 1.0e8:
        return values[1], values[2], values[3]
    return values[0], values[1], values[2]


def convert_pose_xy(x, y, source_offset, target_x, target_y):
    if source_offset is not None:
        return x - source_offset[0] + target_x, y - source_offset[1] + target_y
    if abs(x) < 1000.0 and abs(y) < 1000.0:
        return x + target_x, y + target_y
    return x, y


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def filter_pose_points(points, min_step):
    filtered = []
    for point in points:
        if not filtered or distance(point, filtered[-1]) >= min_step:
            filtered.append(point)
    if len(filtered) >= 2:
        return filtered
    return points


def read_pose_points(path, source_offset, target_x, target_y, min_step):
    points = []
    for line in path.read_text(encoding="utf-8").splitlines():
        parsed = parse_pose_line(line)
        if parsed is None:
            continue
        x, y, _ = parsed
        converted = convert_pose_xy(x, y, source_offset, target_x, target_y)
        if all(math.isfinite(value) for value in converted):
            points.append(converted)
    points = filter_pose_points(points, min_step)
    if len(points) < 2:
        raise ValueError("{} has fewer than two usable pose points".format(path))
    return points


def polyline_length(points):
    return sum(distance(points[index - 1], points[index])
               for index in range(1, len(points)))


def point_at_s(points, s):
    if s <= 0.0:
        return points[0]
    remaining = s
    for index in range(1, len(points)):
        start = points[index - 1]
        end = points[index]
        segment_length = distance(start, end)
        if segment_length <= 1e-12:
            continue
        if remaining <= segment_length:
            ratio = remaining / segment_length
            return (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
            )
        remaining -= segment_length
    return points[-1]


def resample_polyline(points, count):
    if len(points) < 2:
        return list(points)
    length = polyline_length(points)
    if length <= 1e-12:
        return [points[0] for _ in range(count)]
    if count <= 1:
        return [points[0]]
    return [point_at_s(points, length * index / (count - 1))
            for index in range(count)]


def bezier(points, t):
    if len(points) == 3:
        p0, p1, p2 = points
        u = 1.0 - t
        return (
            u * u * p0[0] + 2.0 * u * t * p1[0] + t * t * p2[0],
            u * u * p0[1] + 2.0 * u * t * p1[1] + t * t * p2[1],
        )
    if len(points) == 4:
        p0, p1, p2, p3 = points
        u = 1.0 - t
        return (
            u * u * u * p0[0] + 3.0 * u * u * t * p1[0] +
            3.0 * u * t * t * p2[0] + t * t * t * p3[0],
            u * u * u * p0[1] + 3.0 * u * u * t * p1[1] +
            3.0 * u * t * t * p2[1] + t * t * t * p3[1],
        )
    raise ValueError("unsupported bezier control count")


def normalize_angle(angle):
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def heading(points):
    if len(points) < 2:
        return 0.0
    return math.atan2(points[-1][1] - points[0][1],
                      points[-1][0] - points[0][0])


def trajectory_turn_degrees(points):
    headings = []
    for index in range(1, len(points)):
        if distance(points[index - 1], points[index]) > 1e-6:
            headings.append(heading([points[index - 1], points[index]]))
    if len(headings) < 2:
        return 0.0
    return math.degrees(
        sum(abs(normalize_angle(headings[index] - headings[index - 1]))
            for index in range(1, len(headings)))
    )


def transform_point(point, theta, bx, by):
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    x, y = point
    return (
        cos_theta * x - sin_theta * y + bx,
        sin_theta * x + cos_theta * y + by,
    )


def centroid(points):
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def estimate_rigid_transform(source_points, target_points):
    if len(source_points) != len(target_points) or len(source_points) < 2:
        raise ValueError("need matching point pairs")
    source_center = centroid(source_points)
    target_center = centroid(target_points)
    cross = 0.0
    dot = 0.0
    variance = 0.0
    for source, target in zip(source_points, target_points):
        sx = source[0] - source_center[0]
        sy = source[1] - source_center[1]
        tx = target[0] - target_center[0]
        ty = target[1] - target_center[1]
        cross += sx * ty - sy * tx
        dot += sx * tx + sy * ty
        variance += sx * sx + sy * sy
    if variance <= 1e-12:
        raise ValueError("matched map points have too little variance")
    theta = math.atan2(cross, dot)
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    bx = target_center[0] - (
        cos_theta * source_center[0] - sin_theta * source_center[1])
    by = target_center[1] - (
        sin_theta * source_center[0] + cos_theta * source_center[1])
    return theta, bx, by


def nearest_indices(transformed_lane, pose_points):
    indices = []
    distances = []
    for pose in pose_points:
        best_index = 0
        best_distance = float("inf")
        for index, point in enumerate(transformed_lane):
            current = distance(pose, point)
            if current < best_distance:
                best_index = index
                best_distance = current
        indices.append(best_index)
        distances.append(best_distance)
    return indices, distances


def score_transform(lane_points, pose_points, theta, bx, by):
    transformed = [
        transform_point(point, theta, bx, by)
        for point in lane_points
    ]
    _, distances = nearest_indices(transformed, pose_points)
    rms = math.sqrt(sum(value * value for value in distances) / len(distances))
    return {
        "rms": rms,
        "mean": sum(distances) / len(distances),
        "max": max(distances),
    }


def run_icp(lane_points, pose_points, initial_theta, initial_bx, initial_by,
            max_iterations):
    theta = initial_theta
    bx = initial_bx
    by = initial_by
    for _ in range(max_iterations):
        transformed = [
            transform_point(point, theta, bx, by)
            for point in lane_points
        ]
        indices, _ = nearest_indices(transformed, pose_points)
        matched_source = [lane_points[index] for index in indices]
        try:
            new_theta, new_bx, new_by = estimate_rigid_transform(
                matched_source, pose_points)
        except ValueError:
            break
        delta = (
            abs(normalize_angle(new_theta - theta)) +
            abs(new_bx - bx) + abs(new_by - by)
        )
        theta, bx, by = new_theta, new_bx, new_by
        if delta < 1e-9:
            break
    metrics = score_transform(lane_points, pose_points, theta, bx, by)
    return theta, bx, by, metrics


def absolute_position(position, center):
    return (
        center[0] + float(position["x"]),
        center[1] + float(position["y"]),
    )


def boundary_points(boundary, point_by_id, center):
    ids = (
        boundary.get("point_id") or boundary.get("pointIds") or
        boundary.get("point_ids") or []
    )
    base_points = [point_by_id[point_id] for point_id in ids
                   if point_id in point_by_id]
    controls = (
        boundary.get("controlsPosition") or
        boundary.get("controls_position") or []
    )
    control_points = [
        absolute_position(control, center)
        for control in controls
        if isinstance(control, dict) and "x" in control and "y" in control
    ]
    if len(base_points) == 2 and len(control_points) in (1, 2):
        curve = [base_points[0]] + control_points + [base_points[1]]
        return [bezier(curve, index / 40.0) for index in range(41)]
    return base_points


def load_lane_centerlines(editor_map, target_center_y, target_center_x, samples):
    source_center = editor_map.get("basemapCenter")
    if not isinstance(source_center, dict):
        raise ValueError("editor_map.json has no basemapCenter")
    source_x = float(source_center["x"])
    target_x = source_x if target_center_x is None else float(target_center_x)
    center = (target_x, float(target_center_y))

    point_by_id = {}
    for point in editor_map.get("point", []):
        position = point.get("position")
        if not isinstance(position, dict):
            continue
        point_by_id[str(point["id"])] = absolute_position(position, center)

    boundary_by_id = {}
    for boundary in editor_map.get("boundary", []):
        points = boundary_points(boundary, point_by_id, center)
        if len(points) >= 2:
            boundary_by_id[str(boundary["id"])] = points

    lanes = []
    for lane in editor_map.get("lane", []):
        lane_id = str(lane.get("id", ""))
        left_id = str(lane.get("left_boundary_id") or lane.get("leftBoundaryId") or "")
        right_id = str(lane.get("right_boundary_id") or lane.get("rightBoundaryId") or "")
        if left_id not in boundary_by_id or right_id not in boundary_by_id:
            continue
        left = list(boundary_by_id[left_id])
        right = list(boundary_by_id[right_id])
        if lane.get("left_boundary_reverse") or lane.get("leftBoundaryReverse"):
            left.reverse()
        if lane.get("right_boundary_reverse") or lane.get("rightBoundaryReverse"):
            right.reverse()
        left_samples = resample_polyline(left, samples)
        right_samples = resample_polyline(right, samples)
        count = min(len(left_samples), len(right_samples))
        centerline = [
            (
                0.5 * (left_samples[index][0] + right_samples[index][0]),
                0.5 * (left_samples[index][1] + right_samples[index][1]),
            )
            for index in range(count)
        ]
        if len(centerline) >= 2 and polyline_length(centerline) > 1e-6:
            lanes.append({
                "lane_id": lane_id,
                "centerline": centerline,
                "length": polyline_length(centerline),
            })
    if not lanes:
        raise ValueError("editor_map.json has no usable lane centerlines")
    return center, lanes


def initial_guesses(lane_points, pose_points):
    lane_center = centroid(lane_points)
    pose_center = centroid(pose_points)
    guesses = [(0.0, 0.0, 0.0)]
    for angle_delta in (
            heading(pose_points) - heading(lane_points),
            heading(pose_points) - heading(lane_points) + math.pi,
            heading(pose_points) - heading(lane_points) - math.pi):
        cos_theta = math.cos(angle_delta)
        sin_theta = math.sin(angle_delta)
        bx = pose_center[0] - (
            cos_theta * lane_center[0] - sin_theta * lane_center[1])
        by = pose_center[1] - (
            sin_theta * lane_center[0] + cos_theta * lane_center[1])
        guesses.append((normalize_angle(angle_delta), bx, by))
    return guesses


def make_candidate(method, lane, direction, theta, bx, by, origin,
                   lane_points, pose_points):
    tx, ty = script_translation(theta, bx, by, origin)
    metrics = score_transform(lane_points, pose_points, theta, bx, by)
    return {
        "method": method,
        "lane_id": lane["lane_id"],
        "direction": direction,
        "lane_length": lane["length"],
        "rotate_deg": math.degrees(theta),
        "translate_x": tx,
        "translate_y": ty,
        "global_bx": bx,
        "global_by": by,
        "rms_error": metrics["rms"],
        "mean_error": metrics["mean"],
        "max_error": metrics["max"],
    }


def heading_centroid_candidate(lane, direction, lane_points, pose_points,
                               origin):
    theta = normalize_angle(heading(pose_points) - heading(lane_points))
    lane_center = centroid(lane_points)
    pose_center = centroid(pose_points)
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    bx = pose_center[0] - (
        cos_theta * lane_center[0] - sin_theta * lane_center[1])
    by = pose_center[1] - (
        sin_theta * lane_center[0] + cos_theta * lane_center[1])
    return make_candidate(
        "heading_centroid",
        lane,
        direction,
        theta,
        bx,
        by,
        origin,
        lane_points,
        pose_points,
    )


def script_translation(theta, bx, by, origin):
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    ox, oy = origin
    rotated_origin = (
        cos_theta * ox - sin_theta * oy,
        sin_theta * ox + cos_theta * oy,
    )
    return bx - ox + rotated_origin[0], by - oy + rotated_origin[1]


def candidate_is_preferred(candidate, max_abs_rotate_deg, max_abs_translate):
    return (
        abs(candidate["rotate_deg"]) <= max_abs_rotate_deg and
        abs(candidate["translate_x"]) <= max_abs_translate and
        abs(candidate["translate_y"]) <= max_abs_translate
    )


def estimate_alignment(lanes, pose_points, origin, lane_id, max_iterations,
                       max_abs_rotate_deg, max_abs_translate):
    candidates = []
    for lane in lanes:
        if lane_id and lane["lane_id"] != lane_id:
            continue
        variants = [
            ("forward", lane["centerline"]),
            ("reverse", list(reversed(lane["centerline"]))),
        ]
        for direction, lane_points in variants:
            candidates.append(heading_centroid_candidate(
                lane, direction, lane_points, pose_points, origin))
            best_for_variant = None
            for initial in initial_guesses(lane_points, pose_points):
                theta, bx, by, metrics = run_icp(
                    lane_points,
                    pose_points,
                    initial[0],
                    initial[1],
                    initial[2],
                    max_iterations,
                )
                tx, ty = script_translation(theta, bx, by, origin)
                candidate = {
                    "method": "icp",
                    "lane_id": lane["lane_id"],
                    "direction": direction,
                    "lane_length": lane["length"],
                    "rotate_deg": math.degrees(theta),
                    "translate_x": tx,
                    "translate_y": ty,
                    "global_bx": bx,
                    "global_by": by,
                    "rms_error": metrics["rms"],
                    "mean_error": metrics["mean"],
                    "max_error": metrics["max"],
                }
                if (best_for_variant is None or
                        candidate["rms_error"] < best_for_variant["rms_error"]):
                    best_for_variant = candidate
            if best_for_variant is not None:
                candidates.append(best_for_variant)
    if not candidates:
        raise ValueError("no lane candidates matched lane_id={!r}".format(lane_id))
    candidates.sort(key=lambda item: (
        0 if candidate_is_preferred(
            item, max_abs_rotate_deg, max_abs_translate) else 1,
        item["rms_error"],
        abs(item["rotate_deg"]),
        abs(item["translate_x"]) + abs(item["translate_y"]),
    ))
    return candidates


def format_command(install_script, released_map, best, workspace_root):
    return (
        "python3 {script} \\\n"
        "  --src {src} \\\n"
        "  --rotate-deg {rotate:.6f} \\\n"
        "  --translate-x {tx:.6f} \\\n"
        "  --translate-y {ty:.6f}"
    ).format(
        script=apollo_workspace_path(install_script, workspace_root),
        src=apollo_workspace_path(released_map, workspace_root),
        rotate=best["rotate_deg"],
        tx=best["translate_x"],
        ty=best["translate_y"],
    )


def main():
    args = parse_args()
    workspace_root = args.workspace_root
    map_dir = resolve_path(args.map_dir, workspace_root) if args.map_dir else None
    released_map = resolve_path(args.released_map, workspace_root)
    pose_file = resolve_path(
        args.pose_file, workspace_root) if args.pose_file else choose_pose_file(
            map_dir, None)
    source_offset_file = (
        resolve_path(args.source_offset_file, workspace_root)
        if args.source_offset_file
        else (map_dir / "gnss-map-offset.txt" if map_dir else None)
    )
    output = (
        resolve_path(args.output, workspace_root)
        if args.output
        else (map_dir / "hdmap_alignment.json" if map_dir else Path("hdmap_alignment.json"))
    )
    install_script = resolve_path(args.install_script, workspace_root)

    editor_map_path = released_map / "editor_map.json"
    if not editor_map_path.is_file():
        raise FileNotFoundError(str(editor_map_path))
    source_offset = read_offset(source_offset_file)
    pose_points = read_pose_points(
        pose_file,
        source_offset,
        args.pose_target_offset_x,
        args.pose_target_offset_y,
        args.min_pose_step,
    )
    editor_map = read_json(editor_map_path)
    origin, lanes = load_lane_centerlines(
        editor_map,
        args.target_basemap_center_y,
        args.target_basemap_center_x,
        args.samples_per_lane,
    )
    candidates = estimate_alignment(
        lanes,
        pose_points,
        origin,
        args.lane_id,
        args.max_iterations,
        args.max_abs_rotate_deg,
        args.max_abs_translate,
    )
    best = candidates[0]

    pose_span = distance(pose_points[0], pose_points[-1])
    pose_length = polyline_length(pose_points)
    turn_degrees = trajectory_turn_degrees(pose_points)
    warnings = []
    if pose_span < args.min_pose_span:
        warnings.append(
            "pose span {:.3f} m is short; alignment may be unreliable".format(
                pose_span))
    if turn_degrees < 10.0:
        warnings.append(
            "pose trajectory is nearly straight; longitudinal translation along "
            "the lane is weakly constrained. Prefer an L-shaped calibration "
            "trajectory or verify translate_x/y in Dreamview."
        )
    if best["rms_error"] > 0.5:
        warnings.append(
            "best RMS error {:.3f} m is high; check lane id or redraw the lane "
            "centerline closer to the driven path".format(best["rms_error"])
        )
    if not candidate_is_preferred(
            best, args.max_abs_rotate_deg, args.max_abs_translate):
        warnings.append(
            "best candidate is outside the preferred small correction range; "
            "verify lane id and pose trajectory before installing"
        )

    command = format_command(install_script, released_map, best, workspace_root)
    summary = {
        "workspace_root": str(workspace_root),
        "map_dir": str(map_dir) if map_dir else "",
        "released_map": str(released_map),
        "pose_file": str(pose_file),
        "source_offset_file": str(source_offset_file) if source_offset_file else "",
        "source_offset": list(source_offset) if source_offset else None,
        "pose_count": len(pose_points),
        "pose_start": list(pose_points[0]),
        "pose_end": list(pose_points[-1]),
        "pose_span_m": pose_span,
        "pose_length_m": pose_length,
        "pose_turn_degrees": turn_degrees,
        "rotation_origin": list(origin),
        "best": best,
        "candidates": candidates[:args.top_candidates],
        "warnings": warnings,
        "install_command": command,
    }

    if not args.no_write:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                          encoding="utf-8")

    print("RESULT: estimated HDMap alignment from pose trajectory.")
    print("pose_file={}".format(pose_file))
    print("released_map={}".format(released_map))
    print("pose_count={} span={:.3f}m length={:.3f}m turn={:.3f}deg".format(
        len(pose_points), pose_span, pose_length, turn_degrees))
    print("best_lane={} direction={} method={} rms={:.3f}m max={:.3f}m".format(
        best["lane_id"], best["direction"], best["method"],
        best["rms_error"], best["max_error"]))
    print("rotate_deg={:.6f}".format(best["rotate_deg"]))
    print("translate_x={:.6f}".format(best["translate_x"]))
    print("translate_y={:.6f}".format(best["translate_y"]))
    if warnings:
        print()
        print("warnings:")
        for warning in warnings:
            print("  - {}".format(warning))
    print()
    print("Install command:")
    print(command)
    if not args.no_write:
        print()
        print("summary={}".format(output))
    print()
    print("Top candidates:")
    for candidate in candidates[:args.top_candidates]:
        print(
            "  lane={lane_id} dir={direction} method={method} rms={rms_error:.3f} "
            "rot={rotate_deg:.3f} tx={translate_x:.3f} ty={translate_y:.3f}".format(
                **candidate
            )
        )


if __name__ == "__main__":
    main()
