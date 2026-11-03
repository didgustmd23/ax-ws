#!/usr/bin/env bash
# 최초 준비용: ROS Jazzy는 미리 설치되어 있어야 하며 apt/rosdep 설치에는 sudo와 네트워크가 필요하다.
# Ubuntu 24.04 with ROS 2 Jazzy installed; run from any working directory.
set -eo pipefail
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "Install ROS 2 Jazzy on Ubuntu 24.04 first; see docs/guides/RUN_ON_ANOTHER_PC.md" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
cd "$(dirname "${BASH_SOURCE[0]}")/.."
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep python3-numpy python3-scipy
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --packages-up-to ax_description ax_interfaces ax_control
source install/setup.bash
python3 tools/check_portability.py
printf '\nReady: source install/setup.bash && ros2 launch ax_control simulation.launch.py\n'
