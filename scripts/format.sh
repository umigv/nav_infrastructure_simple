#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "$ROOT"

echo "==> Ruff organize imports and remove unused"
ruff check --select I,F401 --fix .

echo "==> Ruff format"
ruff format .
