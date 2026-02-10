#!/usr/bin/env bash
set -euo pipefail

export ROS_VERSION=2
export ROS_DISTRO="${ROS_DISTRO:=humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup not found at $ROS_SETUP"
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd -- "${REPO_ROOT}/../.." && pwd)"

echo "==> Repo root:      $REPO_ROOT"
echo "==> Workspace root: $WS_ROOT"

cd "$WS_ROOT"

if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
  set -u
fi

MODE="repo"
if [[ "${1:-}" == "--workspace" ]]; then
  MODE="workspace"
  shift
fi

if [[ "$MODE" == "workspace" ]]; then
  echo "==> colcon build (workspace)"
  colcon build --symlink-install --event-handlers console_direct+ "$@"
else
  echo "==> Discovering packages under repo root"
  mapfile -t PKGS < <(colcon list --base-paths "$REPO_ROOT" --names-only)

  if [[ "${#PKGS[@]}" -eq 0 ]]; then
    echo "ERROR: No colcon packages found under $REPO_ROOT"
    exit 1
  fi

  echo "==> colcon build (repo packages: ${#PKGS[@]})"
  colcon build --symlink-install --packages-select "${PKGS[@]}" --event-handlers console_direct+ "$@"
fi

echo "==> Build complete."
echo "==> Source this in your current shell:"
echo "    source \"$WS_ROOT/install/setup.bash\""
