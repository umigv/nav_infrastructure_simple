#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "$ROOT"

command -v ruff >/dev/null || { echo "ruff not found. Run scripts/setup.sh"; exit 1; }
command -v mypy >/dev/null || { echo "mypy not found. Run scripts/setup.sh"; exit 1; }

echo "==> Ruff format (check)"
ruff format --check .

echo "==> Ruff lint"
ruff check .

echo "==> Discovering ROS packages"
mapfile -t PKG_DIRS < <(
  find . -mindepth 2 -maxdepth 2 -type f -name package.xml -print \
    | sed 's|^\./||' \
    | xargs -n1 dirname \
    | sort -u
)

if [[ "${#PKG_DIRS[@]}" -eq 0 ]]; then
  echo "ERROR: No <pkg>/package.xml found under project root"
  exit 1
fi

MYPYPATH="$(printf "%s:" "${PKG_DIRS[@]}")"
export MYPYPATH="${MYPYPATH%:}"

echo "==> mypy (${#PKG_DIRS[@]} packages)"
mypy "${PKG_DIRS[@]}"

echo "==> All checks passed!"
