#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <package_name>"
  exit 1
fi

PKG_NAME="$1"

if [[ ! "$PKG_NAME" =~ ^[a-z][a-z0-9_]*$ ]]; then
  echo "ERROR: Invalid package name '$PKG_NAME'"
  echo "Package names must start with a lowercase letter and contain only lowercase letters, numbers, and underscores."
  exit 1
fi

echo "==> Creating ROS 2 Python package: $PKG_NAME"

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

ros2 pkg create \
  --build-type ament_python \
  --license Apache-2.0 \
  "$PKG_NAME"

echo "==> Package '$PKG_NAME' created successfully"
