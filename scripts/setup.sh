#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"     # ws/src/nav_stack
WS_ROOT="$(cd -- "${REPO_ROOT}/../.." && pwd)"     # ws

echo "==> Repo root:      $REPO_ROOT"
echo "==> Workspace root: $WS_ROOT"

if [[ ! -d "$WS_ROOT/src" ]]; then
  echo "ERROR: Expected workspace src/ at: $WS_ROOT/src"
  exit 1
fi

echo "==> Install Python tooling deps (editable)"
python3 -m pip install -U pip
python3 -m pip install -e "$REPO_ROOT[tooling]"

echo "==> Install ROS deps via rosdep (workspace src/)"
rosdep install --from-paths "$REPO_ROOT" --ignore-src -r -y

echo "==> Setup complete."
