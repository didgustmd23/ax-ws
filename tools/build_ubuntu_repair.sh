#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
python3 tools/generate_physical_properties.py --check
python3 tools/generate_asset_v2.py --check
colcon build --packages-select epick_description ax_description ax_interfaces robotiq_description \
  --parallel-workers 3 --event-handlers console_direct- console_start_end+ \
  --cmake-args -DCMAKE_INSTALL_MESSAGE=LAZY
source install/setup.bash
python3 tools/export_ax1_urdf.py
python3 -m unittest discover -s tools/tests -v 2>&1 | tee log/repair/unit_tests.log
