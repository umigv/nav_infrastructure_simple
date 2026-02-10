#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "$ROOT"

echo "==> Ruff lint (fix: imports + safe fixes)"
FIX_SUMMARY="$(ruff check --fix --exit-zero . 2>/dev/null | grep -E 'fixed' || true)"

if [[ -n "$FIX_SUMMARY" ]]; then
  echo "$FIX_SUMMARY"
else
  echo "No lint fixes applied"
fi

echo "==> Ruff format"
ruff format .
