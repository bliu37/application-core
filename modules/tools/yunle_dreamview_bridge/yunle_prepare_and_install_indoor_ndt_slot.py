#!/usr/bin/env python3

"""Prepare and install a Yunle indoor point-cloud/NDT map into the fixed slot."""

import argparse
import datetime
import shutil
import subprocess
from pathlib import Path


DEFAULT_SOURCE_MAP_DIR = "data/map_work/yunle_indoor/jd03_indoor_new"
DEFAULT_TARGET_MAP_DIR = "data/map_work/yunle_indoor/jd03_indoor_20260803_03"
DEFAULT_NDT_PREPARE_SCRIPT = (
    "modules/tools/yunle_dreamview_bridge/yunle_indoor_ndt_map_prepare.py"
)
DEFAULT_TOPDOWN_SCRIPT = (
    "modules/tools/yunle_dreamview_bridge/yunle_pcd_topdown_renderer.py"
)
DEFAULT_NDT_MAP_CREATOR = "/opt/apollo/neo/bin/ndt_map_creator"

REQUIRED_SOURCE_FILES = (
    "global.pcd",
    "3D-Pose.txt",
    "6D-Pose.txt",
    "gnss-map-offset.txt",
    "slam_pose_result.bin",
)

OPTIONAL_SOURCE_ENTRIES = (
    "globalPointCloud.txt",
    "globalds.pcd",
    "surfPointCloud.txt",
    "surfPointDsCloud.txt",
    "record",
    "topdown_preview.png",
    "topdown_preview.json",
    "image_creator_conf.pb.txt",
)

NDT_REQUIRED_FILES = (
    "config.xml",
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
            "Generate topdown preview, prepare ndt_input_float_safe, run "
            "ndt_map_creator, then backup and replace the fixed indoor "
            "localization map slot. The fixed slot is used by current Yunle "
            "indoor localization configs/components."
        )
    )
    parser.add_argument(
        "--src",
        default=DEFAULT_SOURCE_MAP_DIR,
        help=(
            "new saved map directory containing global.pcd and pose files. "
            "Accepts host, /apollo_workspace, or workspace-relative paths."
        ),
    )
    parser.add_argument(
        "--dst",
        default=DEFAULT_TARGET_MAP_DIR,
        help="fixed runtime source map directory to replace.",
    )
    parser.add_argument(
        "--backup-dir",
        default="",
        help="backup directory. Defaults to <dst>.backup_YYYYmmdd_HHMMSS.",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="replace the fixed slot without keeping the old directory.",
    )
    parser.add_argument(
        "--skip-preview",
        action="store_true",
        help="do not generate topdown_preview.png.",
    )
    parser.add_argument(
        "--skip-ndt-prepare",
        action="store_true",
        help="do not run yunle_indoor_ndt_map_prepare.py.",
    )
    parser.add_argument(
        "--skip-ndt-create",
        action="store_true",
        help="do not run ndt_map_creator; require existing local_map_float_safe.",
    )
    parser.add_argument(
        "--skip-install",
        action="store_true",
        help="prepare source map only; do not replace the fixed slot.",
    )
    parser.add_argument(
        "--preview-script",
        default=DEFAULT_TOPDOWN_SCRIPT,
        help="topdown preview script path.",
    )
    parser.add_argument(
        "--ndt-prepare-script",
        default=DEFAULT_NDT_PREPARE_SCRIPT,
        help="NDT input preparation script path.",
    )
    parser.add_argument(
        "--ndt-map-creator",
        default=DEFAULT_NDT_MAP_CREATOR,
        help="ndt_map_creator binary path.",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.25,
        help="NDT XY resolution.",
    )
    parser.add_argument(
        "--resolution-z",
        type=float,
        default=0.25,
        help="NDT Z resolution.",
    )
    parser.add_argument(
        "--zone-id",
        type=int,
        default=50,
        help="UTM zone id passed to ndt_map_creator.",
    )
    parser.add_argument(
        "--pool-size",
        type=int,
        default=20,
        help="thread pool size passed to ndt_map_creator.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print planned actions without writing files.",
    )
    parser.set_defaults(workspace_root=workspace_root)
    return parser.parse_args()


def validate_source(src):
    if not src.is_dir():
        raise FileNotFoundError("source map directory not found: {}".format(src))
    missing = [name for name in REQUIRED_SOURCE_FILES if not (src / name).is_file()]
    if missing:
        raise FileNotFoundError(
            "source map is missing required files: {}".format(", ".join(missing))
        )


def validate_ndt_map(src):
    ndt_map = src / "ndt_map" / "local_map_float_safe"
    missing = [name for name in NDT_REQUIRED_FILES if not (ndt_map / name).is_file()]
    if missing:
        raise FileNotFoundError(
            "NDT map is missing required files under {}: {}".format(
                ndt_map, ", ".join(missing)
            )
        )
    return ndt_map


def preview_exists(src):
    output = src / "topdown_preview.png"
    metadata = src / "topdown_preview.json"
    if output.is_file() and metadata.is_file():
        return True
    if not output.exists() and not metadata.exists():
        return False
    raise RuntimeError(
        "topdown preview output is partially present; use --skip-preview or "
        "move the old preview files first: {}, {}".format(output, metadata)
    )


def ndt_map_exists(src):
    try:
        validate_ndt_map(src)
    except FileNotFoundError:
        return False
    return True


def run_command(command):
    print("run: {}".format(" ".join(str(part) for part in command)))
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
    if result.stdout.strip():
        print(result.stdout.strip())
    return result.stdout


def prepare_preview(args, src, workspace_root):
    script = resolve_path(args.preview_script, workspace_root)
    pcd = src / "global.pcd"
    output = src / "topdown_preview.png"
    metadata = src / "topdown_preview.json"
    command = [
        "python3",
        str(script),
        apollo_workspace_path(pcd, workspace_root),
        "--output",
        apollo_workspace_path(output, workspace_root),
        "--metadata",
        apollo_workspace_path(metadata, workspace_root),
    ]
    run_command(command)


def prepare_ndt_input(args, src, workspace_root):
    script = resolve_path(args.ndt_prepare_script, workspace_root)
    command = [
        "python3",
        str(script),
        "--map-dir",
        apollo_workspace_path(src, workspace_root),
    ]
    run_command(command)


def create_ndt_map(args, src, workspace_root):
    creator = resolve_binary(args.ndt_map_creator, workspace_root)
    pcd_dir = src / "ndt_input_float_safe" / "pcds"
    pose_file = src / "ndt_input_float_safe" / "poses.txt"
    map_folder = src / "ndt_map" / "local_map_float_safe"
    command = [
        creator,
        "--pcd_folders",
        apollo_workspace_path(pcd_dir, workspace_root),
        "--pose_files",
        apollo_workspace_path(pose_file, workspace_root),
        "--map_folder",
        apollo_workspace_path(map_folder, workspace_root),
        "--resolution_type",
        "single",
        "--resolution",
        "{:.6g}".format(args.resolution),
        "--resolution_z",
        "{:.6g}".format(args.resolution_z),
        "--zone_id",
        str(args.zone_id),
        "--set_road_cells",
        "false",
        "--pool_size",
        str(args.pool_size),
    ]
    run_command(command)


def copy_tree(src, dst):
    shutil.copytree(str(src), str(dst), symlinks=True)


def fix_installed_ndt_pcd_symlink(map_dir):
    pcd_link = map_dir / "ndt_input_float_safe" / "pcds" / "0.pcd"
    if not pcd_link.parent.is_dir():
        return
    if pcd_link.exists() or pcd_link.is_symlink():
        pcd_link.unlink()
    pcd_link.symlink_to(Path("..") / ".." / "global.pcd")


def install_fixed_slot(src, dst, backup_dir, keep_backup):
    if not dst.exists():
        raise FileNotFoundError("target fixed slot does not exist: {}".format(dst))
    if backup_dir.exists():
        raise FileExistsError("backup directory already exists: {}".format(backup_dir))

    temp_dir = dst.with_name(dst.name + ".installing_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    if temp_dir.exists():
        raise FileExistsError("temporary install directory already exists: {}".format(temp_dir))

    print("copy new map: {} -> {}".format(src, temp_dir))
    copy_tree(src, temp_dir)
    fix_installed_ndt_pcd_symlink(temp_dir)

    print("backup old fixed slot: {} -> {}".format(dst, backup_dir))
    dst.rename(backup_dir)
    print("activate new fixed slot: {} -> {}".format(temp_dir, dst))
    temp_dir.rename(dst)

    if not keep_backup:
        shutil.rmtree(str(backup_dir))
        backup_dir = None
    return backup_dir


def print_plan(args, src, dst, backup_dir, workspace_root):
    print("RESULT: dry run completed; no files were changed.")
    print("workspace_root={}".format(workspace_root))
    print("source={}".format(src))
    print("target_fixed_slot={}".format(dst))
    if not args.no_backup and not args.skip_install:
        print("backup={}".format(backup_dir))
    print("preview={}".format(src / "topdown_preview.png"))
    print("ndt_input={}".format(src / "ndt_input_float_safe"))
    print("ndt_map={}".format(src / "ndt_map" / "local_map_float_safe"))


def main():
    args = parse_args()
    workspace_root = args.workspace_root
    src = resolve_path(args.src, workspace_root)
    dst = resolve_path(args.dst, workspace_root)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = (
        resolve_path(args.backup_dir, workspace_root)
        if args.backup_dir
        else dst.with_name(dst.name + ".backup_" + timestamp)
    )

    validate_source(src)

    if args.dry_run:
        print_plan(args, src, dst, backup_dir, workspace_root)
        return

    if not args.skip_preview:
        if preview_exists(src):
            print("skip existing preview: {}".format(src / "topdown_preview.png"))
        else:
            prepare_preview(args, src, workspace_root)

    if not args.skip_ndt_prepare:
        prepare_ndt_input(args, src, workspace_root)

    if not args.skip_ndt_create:
        if ndt_map_exists(src):
            print("skip existing NDT map: {}".format(
                src / "ndt_map" / "local_map_float_safe"))
        else:
            create_ndt_map(args, src, workspace_root)

    ndt_map = validate_ndt_map(src)

    active_backup = None
    if not args.skip_install:
        active_backup = install_fixed_slot(
            src, dst, backup_dir, keep_backup=not args.no_backup
        )

    print()
    if args.skip_install:
        print("RESULT: source map is prepared for the fixed indoor localization slot.")
    else:
        print("RESULT: prepared and installed fixed indoor localization map slot.")
    print("workspace_root={}".format(workspace_root))
    print("source={}".format(src))
    print("ndt_map={}".format(ndt_map))
    print("target_fixed_slot={}".format(dst))
    if active_backup is not None:
        print("backup={}".format(active_backup))
    print("runtime_global_pcd=/apollo_workspace/data/map_work/yunle_indoor/jd03_indoor_20260803_03/global.pcd")
    print("runtime_ndt_map=/apollo_workspace/data/map_work/yunle_indoor/jd03_indoor_20260803_03/ndt_map/local_map_float_safe")
    print("Restart indoor localization before using the updated map.")


if __name__ == "__main__":
    main()
