#!/usr/bin/env python3

"""Install a map_creator released HDMap into the Yunle profile map slot."""

import argparse
import datetime
import shutil
from pathlib import Path


DEFAULT_SOURCE = (
    "modules/map_creator/map_editor/data/released_map/"
    "yunle_indoor_new_planning"
)
DEFAULT_TARGET = (
    "profiles/yunle/modules/map/data/yunle_indoor_map03_planning"
)

REQUIRED_MAP_FILES = (
    "base_map.bin",
    "base_map.txt",
    "sim_map.bin",
    "sim_map.txt",
    "routing_map.bin",
    "routing_map.txt",
)

OPTIONAL_MAP_FILES = (
    "editor_map.json",
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


def parse_args():
    workspace_root = find_workspace_root()
    parser = argparse.ArgumentParser(
        description=(
            "Backup the current Yunle profile HDMap and replace its map files "
            "with a released map_creator output. The runtime data/map_data "
            "symlink entry is left untouched."
        )
    )
    parser.add_argument(
        "--src",
        default=DEFAULT_SOURCE,
        help=(
            "released HDMap directory. Accepts a host path, a container "
            "/apollo_workspace path, or a path relative to the workspace."
        ),
    )
    parser.add_argument(
        "--dst",
        default=DEFAULT_TARGET,
        help=(
            "target profile map directory to replace. Defaults to the real "
            "source directory behind yunle_indoor_map03_planning."
        ),
    )
    parser.add_argument(
        "--backup-dir",
        default="",
        help=(
            "backup directory. Defaults to <dst>.backup_YYYYmmdd_HHMMSS."
        ),
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="replace files without creating a backup first.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print what would be done without copying files.",
    )
    parser.set_defaults(workspace_root=workspace_root)
    return parser.parse_args()


def validate_source(src):
    if not src.is_dir():
        raise FileNotFoundError("source directory not found: {}".format(src))
    missing = [name for name in REQUIRED_MAP_FILES if not (src / name).is_file()]
    if missing:
        raise FileNotFoundError(
            "source map is missing required files: {}".format(
                ", ".join(missing)
            )
        )


def validate_target(dst):
    if not dst.is_dir():
        raise FileNotFoundError("target directory not found: {}".format(dst))


def make_backup(dst, backup_dir, dry_run):
    if backup_dir.exists():
        raise FileExistsError("backup directory already exists: {}".format(backup_dir))
    print("backup: {} -> {}".format(dst, backup_dir))
    if not dry_run:
        shutil.copytree(str(dst), str(backup_dir), symlinks=True)


def copy_map_files(src, dst, dry_run):
    copied = []
    for name in REQUIRED_MAP_FILES + OPTIONAL_MAP_FILES:
        source_file = src / name
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

    validate_source(src)
    validate_target(dst)

    backup_dir = None
    if not args.no_backup:
        if args.backup_dir:
            backup_dir = resolve_path(args.backup_dir, workspace_root)
        else:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_dir = dst.with_name(dst.name + ".backup_" + timestamp)
        make_backup(dst, backup_dir, args.dry_run)

    copied = copy_map_files(src, dst, args.dry_run)

    print()
    if args.dry_run:
        print("RESULT: dry run completed; no files were changed.")
    else:
        print("RESULT: installed released HDMap into Yunle profile map slot.")
    print("workspace_root={}".format(workspace_root))
    print("source={}".format(src))
    print("target={}".format(dst))
    if backup_dir is not None:
        print("backup={}".format(backup_dir))
    print("copied_files={}".format(", ".join(copied)))
    print("runtime_map_dir=/apollo/modules/map/data/yunle_indoor_map03_planning")
    print("restart Dreamview/Routing/Planning before using the updated map.")


if __name__ == "__main__":
    main()
