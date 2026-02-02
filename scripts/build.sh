#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"     # ws/src/nav_infrastructure_simple
WS_ROOT="$(cd -- "${REPO_ROOT}/../.." && pwd)"     # ws

echo "==> Repo root:      $REPO_ROOT"
echo "==> Workspace root: $WS_ROOT"

cd "$WS_ROOT"

# Source install if present (helps overlays / finding packages), but don't die on nounset issues.
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
