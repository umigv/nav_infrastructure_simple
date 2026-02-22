#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "$ROOT"

command -v ruff >/dev/null || { echo "ruff not found. Run scripts/setup.sh"; exit 1; }
command -v mypy >/dev/null || { echo "mypy not found. Run scripts/setup.sh"; exit 1; }

ONLY_PKGS=()
IGNORE_PKGS=()

usage() {
  cat <<EOF
Usage: $(basename "$0") [--only pkg1 pkg2 ...] [--ignore pkg1 pkg2 ...]

Options:
  --only     Lint only the specified packages
  --ignore   Lint all discovered packages except the specified ones

Examples:
  $0
  $0 --only nav_utils occupancy_grid_transform
  $0 --ignore perception cv_stack
EOF
  exit 1
}

# ---- parse args ----
while [[ $# -gt 0 ]]; do
  case "$1" in
    --only)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        ONLY_PKGS+=("$1")
        shift
      done
      ;;
    --ignore)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        IGNORE_PKGS+=("$1")
        shift
      done
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Unknown argument: $1"
      usage
      ;;
  esac
done

if [[ "${#ONLY_PKGS[@]}" -gt 0 && "${#IGNORE_PKGS[@]}" -gt 0 ]]; then
  echo "ERROR: --only and --ignore are mutually exclusive"
  exit 1
fi

echo "==> Discovering ROS packages"
mapfile -t SUBMODULE_DIRS < <(git submodule foreach --quiet 'echo $displaypath' 2>/dev/null || true)

mapfile -t ALL_PKG_DIRS < <(
  find . -mindepth 2 -maxdepth 2 -type f -name package.xml -print \
    | sed 's|^\./||' \
    | xargs -n1 dirname \
    | sort -u \
    | grep -vxF "$(printf '%s\n' "${SUBMODULE_DIRS[@]}")" || true
)

if [[ "${#ALL_PKG_DIRS[@]}" -eq 0 ]]; then
  echo "ERROR: No <pkg>/package.xml found under project root"
  exit 1
fi

PKG_DIRS=("${ALL_PKG_DIRS[@]}")

# ---- apply filters ----
if [[ "${#ONLY_PKGS[@]}" -gt 0 ]]; then
  PKG_DIRS=()
  for pkg in "${ONLY_PKGS[@]}"; do
    if [[ -d "$pkg" && -f "$pkg/package.xml" ]]; then
      PKG_DIRS+=("$pkg")
    else
      echo "ERROR: '$pkg' is not a valid ROS package directory"
      exit 1
    fi
  done
elif [[ "${#IGNORE_PKGS[@]}" -gt 0 ]]; then
  FILTERED=()
  for dir in "${ALL_PKG_DIRS[@]}"; do
    skip=false
    for ignore in "${IGNORE_PKGS[@]}"; do
      if [[ "$dir" == "$ignore" ]]; then
        skip=true
        break
      fi
    done
    $skip || FILTERED+=("$dir")
  done
  PKG_DIRS=("${FILTERED[@]}")
fi

if [[ "${#PKG_DIRS[@]}" -eq 0 ]]; then
  echo "ERROR: No packages left to lint after filtering"
  exit 1
fi

echo "==> Ruff format (check)"
ruff format --check "${PKG_DIRS[@]}"

echo "==> Ruff lint"
ruff check "${PKG_DIRS[@]}"

MYPYPATH="$(printf "%s:" "${ALL_PKG_DIRS[@]}")"
export MYPYPATH="${MYPYPATH%:}"

echo "==> mypy (${#PKG_DIRS[@]} packages)"
mypy "${PKG_DIRS[@]}"

echo "==> All checks passed!"
