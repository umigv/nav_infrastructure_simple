#!/usr/bin/env bash
set -euo pipefail

BOLD='\033[1m'
CYAN='\033[0;36m'
YELLOW='\033[0;33m'
RESET='\033[0m'

log()  { echo -e "\n${BOLD}${CYAN}==> $*${RESET}"; }
note() { echo -e "    ${YELLOW}NOTE: $*${RESET}"; }
err()  { echo -e "\n${BOLD}ERROR: $*${RESET}" >&2; }

export ROS_VERSION=2
export ROS_DISTRO="${ROS_DISTRO:=humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  err "ROS setup not found at $ROS_SETUP"
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd -- "${REPO_ROOT}/../.." && pwd)"

log "Repo root:      $REPO_ROOT"
log "Workspace root: $WS_ROOT"

if [[ ! -d "$WS_ROOT/src" ]]; then
  err "Expected workspace src/ at: $WS_ROOT/src"
  exit 1
fi

log "Initializing git submodules"
git -C "$REPO_ROOT" submodule update --init --recursive

log "Installing Python tooling deps"
python3 -m pip install -U pip
python3 -m pip install -e "$REPO_ROOT[tooling]"

log "Installing ROS deps via rosdep"
rosdep install --from-paths "$REPO_ROOT" --ignore-src -r -y

log "Adding $USER to dialout group (USB/serial device access)"
sudo usermod -aG dialout "$USER"

log "Setup complete"
note "Restart your computer for group change to take effect"
