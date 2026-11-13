"""Check only Kim's new W3 packages with the repository's ROS style tools."""
# [검사 범위] 김성민님 W3 패키지의 형식·문서 주석 규칙 검사입니다. 로봇 동작을 검사하지 않습니다.
from pathlib import Path

from ament_flake8.main import main_with_errors
from ament_pep257.main import main

ROOT = Path(__file__).resolve().parents[2]
PACKAGES = [str(ROOT / 'src' / package) for package in ('ax_corridor_manager', 'ax_grasp_planner')]


def test_w3_flake8():
    result, errors = main_with_errors(argv=PACKAGES)
    assert result == 0, '\n'.join(errors)


def test_w3_pep257():
    assert main(argv=PACKAGES) == 0
