#!/usr/bin/env bash
# C1 전용 CPU 환경. ROS용 시스템 Python 패키지는 변경하지 않는다.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if ! python3 -c 'import ensurepip' >/dev/null 2>&1; then
  sudo apt-get install -y python3-venv
fi
python3 -m venv --system-site-packages .venv-vision
.venv-vision/bin/python -m pip install 'torch==2.5.1' 'torchvision==0.20.1' --index-url https://download.pytorch.org/whl/cpu
.venv-vision/bin/python -m pip install 'numpy==1.26.4' 'opencv-python==4.11.0.86' 'ultralytics==8.4.202'
printf '\nReady: source /opt/ros/jazzy/setup.bash && source .venv-vision/bin/activate\n'
