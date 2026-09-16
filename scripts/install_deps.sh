#!/usr/bin/env bash
# Native Ubuntu 22.04 with Humble already installed. No changes unless --apply.
set -eo pipefail
packages=(
  build-essential cmake git pkg-config zsh iproute2 iputils-ping unzip zip
  python3-dev python3-colcon-common-extensions python3-rosdep python3-yaml
  python3-numpy python3-matplotlib libeigen3-dev libpcl-dev libboost-all-dev libapr1-dev
  ros-humble-ament-cmake-auto ros-humble-rosidl-default-generators
  ros-humble-rclcpp ros-humble-rclcpp-components ros-humble-rcl-interfaces
  ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-std-msgs
  ros-humble-std-srvs ros-humble-visualization-msgs ros-humble-tf2 ros-humble-tf2-ros
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-rmw-fastrtps-cpp
  ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2)
if [[ $# -gt 1 ]]; then echo "Only one option is accepted." >&2; exit 2; fi
case "${1:---print}" in
  --print)
    printf 'sudo apt-get update\nsudo apt-get install -y --no-install-recommends'
    printf ' %q' "${packages[@]}"
    printf '\n# Optional GUI: sudo apt-get install ros-humble-rviz2\n'
    exit 0 ;;
  --apply) ;;
  *) echo "Usage: bash scripts/install_deps.sh [--print|--apply]" >&2; exit 2 ;;
esac
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 || ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Requires Ubuntu 22.04 with ROS Humble already installed; see doc/JETSON_DEPLOY.md, or use docker/." >&2
  exit 2
fi
case "$(uname -m)" in aarch64|arm64|x86_64) ;; *) echo "Only ARM64/x86-64 SDKs are bundled." >&2; exit 2 ;; esac
privilege=()
if [[ "$(id -u)" != 0 ]]; then privilege=(sudo); fi
"${privilege[@]}" apt-get update
exec "${privilege[@]}" apt-get install -y --no-install-recommends "${packages[@]}"
