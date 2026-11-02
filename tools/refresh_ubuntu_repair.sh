#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
colcon build --packages-select ax_description --event-handlers console_direct- console_start_end+
source install/setup.bash
python3 tools/export_ax1_urdf.py
