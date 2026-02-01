#!/usr/bin/env bash
set -euo pipefail

(
  SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
  cd "$ROOT"

  PKGS="${PKGS:-}"

  BUILD_ARGS=(--symlink-install)
  TEST_ARGS=()
  if [[ -n "$PKGS" ]]; then
    BUILD_ARGS+=(--packages-select $PKGS)
    TEST_ARGS+=(--packages-select $PKGS)
  fi

  echo "==> colcon build ${BUILD_ARGS[*]}"
  colcon build "${BUILD_ARGS[@]}"

  # shellcheck disable=SC1091
  source install/setup.bash

  echo "==> colcon test ${TEST_ARGS[*]}"
  colcon test "${TEST_ARGS[@]}"

  echo "==> colcon test-result --verbose"
  colcon test-result --verbose
)
