#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
source install/setup.bash
python3 tools/update_a1_docs.py
python3 tools/audit_robot_geometry.py
python3 tools/verify_drive_direction.py --arm-and-lidar
python3 tools/collect_drive_repair_evidence.py
