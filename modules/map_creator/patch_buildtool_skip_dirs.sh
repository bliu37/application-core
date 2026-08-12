#!/usr/bin/env bash
set -euo pipefail

# Patch Apollo buildtool's workspace scanner so it skips large frontend/runtime
# directories while searching for cyberfile.xml.
#
# Run this inside the Apollo Docker container.

DEFAULT_SKIP_DIRS="${APOLLO_BUILDTOOL_DEFAULT_SKIP_DIRS:-node_modules,bower_components,map_editor}"
BUILD_TOOL_FILE="${APOLLO_BUILDTOOL_ACTION_INIT:-}"
BACKUP_SUFFIX="${APOLLO_BUILDTOOL_BACKUP_SUFFIX:-bak_skip_dirs}"

if [[ -z "${BUILD_TOOL_FILE}" ]]; then
  BUILD_TOOL_FILE="$(
    python3 - <<'PY'
import os
import sys

roots = [
    "/opt/apollo/neo/packages",
    "/usr/local/lib/python3.10/dist-packages",
    "/usr/lib/python3/dist-packages",
]

for root in roots:
    if not os.path.exists(root):
        continue
    for dirpath, _, filenames in os.walk(root):
        if "__init__.py" not in filenames:
            continue
        path = os.path.join(dirpath, "__init__.py")
        try:
            content = open(path, encoding="utf-8", errors="ignore").read()
        except OSError:
            continue
        if "def _search_cyberfile" in content:
            print(path)
            sys.exit(0)

sys.exit("failed to locate buildtool _search_cyberfile __init__.py")
PY
  )"
fi

if [[ ! -f "${BUILD_TOOL_FILE}" ]]; then
  echo "buildtool file not found: ${BUILD_TOOL_FILE}" >&2
  exit 1
fi

cp -n "${BUILD_TOOL_FILE}" "${BUILD_TOOL_FILE}.${BACKUP_SUFFIX}"

python3 - "${BUILD_TOOL_FILE}" "${DEFAULT_SKIP_DIRS}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
default_skip_dirs = sys.argv[2]
content = path.read_text()

if "APOLLO_BUILDTOOL_SKIP_DIRS" in content:
    print(f"buildtool skip-dir patch already present: {path}")
    sys.exit(0)

old = '''        for f in files:
            if f.startswith("."):
                continue
'''

new = f'''        default_skip_dirs = set(filter(None, os.environ.get(
            "APOLLO_BUILDTOOL_DEFAULT_SKIP_DIRS",
            "{default_skip_dirs}").split(",")))
        extra_skip_dirs = set(filter(None, os.environ.get(
            "APOLLO_BUILDTOOL_SKIP_DIRS", "").split(",")))
        skip_dirs = default_skip_dirs | extra_skip_dirs
        for f in files:
            if f.startswith(".") or f in skip_dirs:
                continue
'''

if old not in content:
    raise SystemExit(
        "target snippet not found; inspect buildtool version before patching"
    )

path.write_text(content.replace(old, new))
print(f"buildtool skip-dir patch applied: {path}")
PY

echo "backup: ${BUILD_TOOL_FILE}.${BACKUP_SUFFIX}"
echo "default skip dirs: ${DEFAULT_SKIP_DIRS}"
echo "extra skip dirs can be provided at build time with APOLLO_BUILDTOOL_SKIP_DIRS"

