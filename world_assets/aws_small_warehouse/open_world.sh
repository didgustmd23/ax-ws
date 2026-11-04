#!/usr/bin/env bash
# 이 파일 위치를 기준으로 모델 경로를 잡는다. 기존 AX 월드·로봇 실행에는 영향을 주지 않는다.
set -eo pipefail
world_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$world_dir/upstream:$world_dir/upstream/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec gz sim -r "$world_dir/aws_warehouse_harmonic.world" "$@"
