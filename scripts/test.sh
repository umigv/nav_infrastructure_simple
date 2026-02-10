#!/usr/bin/env bash
set -euo pipefail

: "${ROS_DISTRO:=humble}"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "ERROR: ROS setup not found at /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 1
fi

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
else
  echo "ERROR: $WS_ROOT/install/setup.bash not found. Run scripts/build.sh first."
  exit 1
fi

echo "==> Discovering packages under repo root"
mapfile -t PKGS < <(colcon list --base-paths "$REPO_ROOT" --names-only)

if [[ "${#PKGS[@]}" -eq 0 ]]; then
  echo "ERROR: No colcon packages found under $REPO_ROOT"
  exit 1
fi

echo "==> colcon test (${#PKGS[@]} packages from repo)"
colcon test --packages-select "${PKGS[@]}" --event-handlers console_direct+

echo "==> colcon test-result (verbose)"
colcon test-result --verbose
