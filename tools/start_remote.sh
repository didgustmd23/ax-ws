#!/usr/bin/env bash
# 이미 빌드된 workspace에서 HTTP 리모컨 서버만 실행한다. Gazebo를 자동 생성하지 않는다.
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$repo_dir/install/setup.bash" ]]; then
  echo "Build this workspace first: bash tools/setup_jazzy.sh" >&2
  exit 1
fi
source "$repo_dir/install/setup.bash"
exec python3 "$repo_dir/tools/remote_control.py" "$@"
