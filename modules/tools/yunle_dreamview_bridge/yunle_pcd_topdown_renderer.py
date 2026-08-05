#!/usr/bin/env python3

"""Render a dependency-free top-down PNG from a Yunle indoor PCD map."""

import argparse
import binascii
import json
import math
import os
import struct
import zlib


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Render LIORF PCD points in an Apollo-aligned local view: "
            "vehicle forward is image up and vehicle left is image left."))
    parser.add_argument("pcd", help="input PCD file (ascii or binary)")
    parser.add_argument("--output", help="output PNG path")
    parser.add_argument("--metadata", help="output JSON metadata path")
    parser.add_argument("--pose-file",
                        help="optional LIORF 3D-Pose.txt trajectory")
    parser.add_argument("--offset-file",
                        help="optional LIORF gnss-map-offset.txt")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="map resolution in metres per pixel")
    parser.add_argument("--padding", type=float, default=0.5,
                        help="padding around selected points in metres")
    parser.add_argument("--z-min", type=float, default=-0.7,
                        help="minimum LIORF Z included in the projection")
    parser.add_argument("--z-max", type=float, default=1.5,
                        help="maximum LIORF Z included in the projection")
    parser.add_argument("--point-radius", type=int, default=1,
                        help="rendered point radius in pixels, in [0, 4]")
    return parser.parse_args()


def default_paths(pcd_path, output, metadata, pose_file, offset_file):
    directory = os.path.dirname(pcd_path)
    output = output or os.path.join(directory, "topdown_preview.png")
    metadata = metadata or os.path.splitext(output)[0] + ".json"
    if pose_file is None:
        candidate = os.path.join(directory, "3D-Pose.txt")
        pose_file = candidate if os.path.isfile(candidate) else None
    if offset_file is None:
        candidate = os.path.join(directory, "gnss-map-offset.txt")
        offset_file = candidate if os.path.isfile(candidate) else None
    return output, metadata, pose_file, offset_file


def parse_pcd_header(stream):
    header = {}
    while True:
        raw = stream.readline()
        if not raw:
            raise ValueError("PCD header ended before DATA")
        line = raw.decode("ascii", errors="strict").strip()
        if not line or line.startswith("#"):
            continue
        tokens = line.split()
        key = tokens[0].upper()
        header[key] = tokens[1:]
        if key == "DATA":
            break
    return header


def scalar_format(field_type, size):
    formats = {
        ("F", 4): "f",
        ("F", 8): "d",
        ("I", 1): "b",
        ("I", 2): "h",
        ("I", 4): "i",
        ("I", 8): "q",
        ("U", 1): "B",
        ("U", 2): "H",
        ("U", 4): "I",
        ("U", 8): "Q",
    }
    try:
        return formats[(field_type.upper(), size)]
    except KeyError:
        raise ValueError(
            "unsupported PCD field type/size: {}/{}".format(
                field_type, size))


def pcd_layout(header):
    fields = header.get("FIELDS", [])
    sizes = [int(value) for value in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(value) for value in header.get("COUNT", [])]
    if not counts:
        counts = [1] * len(fields)
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("inconsistent FIELDS/SIZE/TYPE/COUNT in PCD")
    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("PCD must contain x, y and z fields")

    offsets = {}
    formats = {}
    point_step = 0
    for name, size, field_type, count in zip(fields, sizes, types, counts):
        offsets[name] = point_step
        formats[name] = "<" + scalar_format(field_type, size)
        point_step += size * count
    return offsets, formats, point_step


def read_pcd_points(path):
    with open(path, "rb") as stream:
        header = parse_pcd_header(stream)
        offsets, formats, point_step = pcd_layout(header)
        data_type = header["DATA"][0].lower()
        expected_points = int(header.get("POINTS", ["0"])[0])
        points = []

        if data_type == "binary":
            payload = stream.read()
            available_points = len(payload) // point_step
            count = expected_points or available_points
            if available_points < count:
                raise ValueError(
                    "binary PCD payload is shorter than POINTS declares")
            for index in range(count):
                base = index * point_step
                x = struct.unpack_from(formats["x"], payload,
                                       base + offsets["x"])[0]
                y = struct.unpack_from(formats["y"], payload,
                                       base + offsets["y"])[0]
                z = struct.unpack_from(formats["z"], payload,
                                       base + offsets["z"])[0]
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    points.append((x, y, z))
        elif data_type == "ascii":
            fields = header["FIELDS"]
            x_index = fields.index("x")
            y_index = fields.index("y")
            z_index = fields.index("z")
            for raw in stream:
                values = raw.split()
                if len(values) < len(fields):
                    continue
                x = float(values[x_index])
                y = float(values[y_index])
                z = float(values[z_index])
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    points.append((x, y, z))
        elif data_type == "binary_compressed":
            raise ValueError("binary_compressed PCD is not supported")
        else:
            raise ValueError("unsupported PCD DATA type: {}".format(data_type))

    return points, data_type, expected_points


def read_offset(path):
    if path is None:
        return (0.0, 0.0, 0.0)
    with open(path, "r", encoding="utf-8") as stream:
        values = stream.readline().split()
    if len(values) < 3:
        raise ValueError("offset file must contain at least three values")
    return tuple(float(value) for value in values[:3])


def read_trajectory(path, offset):
    if path is None:
        return []
    trajectory = []
    with open(path, "r", encoding="utf-8") as stream:
        for line in stream:
            values = line.split()
            if len(values) < 3:
                continue
            liorf_x = float(values[0]) - offset[0]
            liorf_y = float(values[1]) - offset[1]
            # Measured JD03 relation: Apollo X forward = LIORF Y and
            # Apollo Y left = -LIORF X.
            trajectory.append((liorf_y, -liorf_x))
    return trajectory


def png_chunk(name, payload):
    return (struct.pack(">I", len(payload)) + name + payload +
            struct.pack(">I", binascii.crc32(name + payload) & 0xffffffff))


def write_rgb_png(path, width, height, pixels):
    rows = []
    stride = width * 3
    for row in range(height):
        start = row * stride
        rows.append(b"\x00" + bytes(pixels[start:start + stride]))
    signature = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    payload = b"".join(rows)
    with open(path, "wb") as stream:
        stream.write(signature)
        stream.write(png_chunk(b"IHDR", ihdr))
        stream.write(png_chunk(b"IDAT", zlib.compress(payload, 9)))
        stream.write(png_chunk(b"IEND", b""))


def set_pixel(pixels, width, height, column, row, color):
    if column < 0 or row < 0 or column >= width or row >= height:
        return
    index = (row * width + column) * 3
    pixels[index:index + 3] = bytes(color)


def draw_disc(pixels, width, height, column, row, radius, color):
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                set_pixel(pixels, width, height, column + dx, row + dy,
                          color)


def draw_line(pixels, width, height, start, end, color, radius=1):
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        draw_disc(pixels, width, height, x0, y0, radius, color)
        if x0 == x1 and y0 == y1:
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x0 += sx
        if doubled <= dx:
            error += dx
            y0 += sy


def main():
    args = parse_args()
    if not math.isfinite(args.resolution) or not 0.005 <= args.resolution <= 1.0:
        raise ValueError("resolution must be finite and in [0.005, 1.0]")
    if not math.isfinite(args.padding) or not 0.0 <= args.padding <= 20.0:
        raise ValueError("padding must be finite and in [0, 20]")
    if not (math.isfinite(args.z_min) and math.isfinite(args.z_max) and
            args.z_min < args.z_max):
        raise ValueError("z-min and z-max must be finite and z-min < z-max")
    if not 0 <= args.point_radius <= 4:
        raise ValueError("point-radius must be in [0, 4]")

    pcd_path = os.path.realpath(args.pcd)
    if not os.path.isfile(pcd_path):
        raise ValueError("input PCD does not exist: {}".format(pcd_path))
    output, metadata_path, pose_file, offset_file = default_paths(
        pcd_path, args.output, args.metadata, args.pose_file,
        args.offset_file)
    output = os.path.realpath(output)
    metadata_path = os.path.realpath(metadata_path)
    for path in (output, metadata_path):
        if os.path.exists(path):
            raise ValueError("refusing to overwrite existing output: {}".format(
                path))
        parent = os.path.dirname(path)
        if not os.path.isdir(parent):
            raise ValueError("output directory does not exist: {}".format(
                parent))

    all_points, pcd_data_type, declared_points = read_pcd_points(pcd_path)
    selected = [point for point in all_points
                if args.z_min <= point[2] <= args.z_max]
    if not selected:
        raise ValueError("no finite PCD points remain after the Z filter")

    # Convert LIORF coordinates to the measured JD03 Apollo-local convention.
    projected = [(point[1], -point[0]) for point in selected]
    forwards = [point[0] for point in projected]
    lefts = [point[1] for point in projected]
    min_forward = min(forwards) - args.padding
    max_forward = max(forwards) + args.padding
    min_left = min(lefts) - args.padding
    max_left = max(lefts) + args.padding
    width = int(math.ceil((max_left - min_left) / args.resolution)) + 1
    height = int(math.ceil((max_forward - min_forward) /
                           args.resolution)) + 1
    if width <= 0 or height <= 0 or width * height > 100000000:
        raise ValueError(
            "requested raster is invalid or exceeds 100 million pixels")

    def to_pixel(forward, left):
        column = int(round((max_left - left) / args.resolution))
        row = int(round((max_forward - forward) / args.resolution))
        return column, row

    pixels = bytearray(width * height * 3)
    occupancy = bytearray(width * height)
    for forward, left in projected:
        column, row = to_pixel(forward, left)
        if 0 <= column < width and 0 <= row < height:
            index = row * width + column
            occupancy[index] = min(255, occupancy[index] + 1)

    for row in range(height):
        for column in range(width):
            count = occupancy[row * width + column]
            if not count:
                continue
            brightness = min(255, int(120 + 45 * math.log(count + 1, 2)))
            draw_disc(pixels, width, height, column, row,
                      args.point_radius,
                      (brightness, brightness, brightness))

    offset = read_offset(offset_file)
    trajectory = read_trajectory(pose_file, offset)
    trajectory_pixels = [to_pixel(*point) for point in trajectory]
    for start, end in zip(trajectory_pixels, trajectory_pixels[1:]):
        draw_line(pixels, width, height, start, end, (255, 64, 64), 1)
    if trajectory_pixels:
        draw_disc(pixels, width, height, trajectory_pixels[0][0],
                  trajectory_pixels[0][1], 4, (32, 255, 64))
        draw_disc(pixels, width, height, trajectory_pixels[-1][0],
                  trajectory_pixels[-1][1], 4, (64, 128, 255))

    write_rgb_png(output, width, height, pixels)
    metadata = {
        "source_pcd": pcd_path,
        "pcd_data_type": pcd_data_type,
        "declared_points": declared_points,
        "finite_points": len(all_points),
        "selected_points": len(selected),
        "z_filter_m": {"min": args.z_min, "max": args.z_max},
        "resolution_m_per_pixel": args.resolution,
        "padding_m": args.padding,
        "image": {"width_px": width, "height_px": height},
        "apollo_local_bounds_m": {
            "forward_x": {"min": min_forward, "max": max_forward},
            "left_y": {"min": min_left, "max": max_left},
        },
        "coordinate_conversion": {
            "apollo_local_x_forward": "liorf_y",
            "apollo_local_y_left": "-liorf_x",
            "image_up": "+apollo_local_x_forward",
            "image_left": "+apollo_local_y_left",
        },
        "pixel_conversion": {
            "column": "(max_left_y - left_y) / resolution",
            "row": "(max_forward_x - forward_x) / resolution",
        },
        "trajectory": {
            "pose_file": pose_file,
            "offset_file": offset_file,
            "offset": list(offset),
            "pose_count": len(trajectory),
            "color": "red; start green; end blue",
        },
    }
    with open(metadata_path, "w", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)
        stream.write("\n")

    print("source_points={} selected_points={}".format(
        len(all_points), len(selected)))
    print("image={}x{} resolution={:.3f}m/px".format(
        width, height, args.resolution))
    print("apollo_local_bounds: forward_x=[{:.3f},{:.3f}]m "
          "left_y=[{:.3f},{:.3f}]m".format(
              min_forward, max_forward, min_left, max_left))
    print("trajectory_poses={}".format(len(trajectory)))
    print("png={}".format(output))
    print("metadata={}".format(metadata_path))
    print("RESULT: top-down test preview generated.")


if __name__ == "__main__":
    main()
