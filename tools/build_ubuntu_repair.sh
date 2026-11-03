#!/usr/bin/env bash
# 생성 파일 일치 검사 후 4개 패키지 빌드·호환 URDF 내보내기·단위 검사를 수행한다. 실패는 pipefail로 전달한다.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
# 새 작업공간에서도 검사 로그를 저장할 하위 폴더를 먼저 준비한다.
mkdir -p log/repair
python3 tools/generate_physical_properties.py --check
python3 tools/generate_asset_v2.py --check
colcon build --packages-select epick_description ax_description ax_interfaces robotiq_description \
  --parallel-workers 3 --event-handlers console_direct- console_start_end+ \
  --cmake-args -DCMAKE_INSTALL_MESSAGE=LAZY
source install/setup.bash
python3 tools/export_ax1_urdf.py
python3 -m unittest discover -s tools/tests -v 2>&1 | tee log/repair/unit_tests.log
