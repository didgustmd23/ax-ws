#!/usr/bin/env bash
# 실제 형상·주행·팔·LiDAR 회귀를 실행하고 결과를 수집한다. README와 일정은 자동으로 덮어쓰지 않는다.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
source install/setup.bash
python3 tools/audit_robot_geometry.py
python3 tools/verify_drive_direction.py --arm-and-lidar
python3 tools/collect_drive_repair_evidence.py
