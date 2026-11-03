#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
source install/setup.bash
exec python3 tools/verify_drive_direction.py "$@"
