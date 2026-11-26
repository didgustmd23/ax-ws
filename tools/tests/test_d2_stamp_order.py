"""D2 출력 stamp 단조 판정(d2_stamp_order) 순수 단위 시험(ROS 없음).

실행: python3 -m pytest -q -p no:cacheprovider tools/tests/test_d2_stamp_order.py
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from d2_stamp_order import check_stamp_order, clock_regression_s  # noqa: E402

S = 10**9
BASE = 1795698685 * S
STEADY = [0.0, 0.0, 0.0, 0.0]


# 초 단위 상대 stamp → ns 절대 stamp
def ns(*seconds):
    return [BASE + int(round(s * S)) for s in seconds]


# 역행 없음 + 단조 증가 → 통과(설명 없음)
def test_no_regression_increasing_ok():
    assert check_stamp_order(ns(0.0, 1.0, 2.0, 3.0), STEADY) == (True, '')


# 역행 없음 + 감소 → 실패
def test_no_regression_decrease_fails():
    ok, detail = check_stamp_order(ns(0.0, 1.0, 0.85, 1.85), STEADY)
    assert not ok and '역행 없음' in detail


# 역행 1.2 s + 감소 0.15 s → 통과(허용 내용을 설명에 남김)
def test_regression_allows_small_decrease():
    ok, detail = check_stamp_order(ns(0.0, 1.0, 0.85, 1.85), [0.0, 0.0, -1.2, -1.2, -1.2])
    assert ok and '역행 감지' in detail and '0.150 s 허용' in detail


# 역행 0.1 s + 감소 0.8 s → 실패(감소가 역행량 + 여유보다 큼)
def test_decrease_larger_than_regression_fails():
    ok, detail = check_stamp_order(ns(0.0, 1.0, 0.2, 1.2), [0.0, -0.1, -0.1])
    assert not ok and '넘음' in detail


# 감소 2회 → 실패(역행이 있어도)
def test_two_decreases_fail():
    ok, detail = check_stamp_order(ns(0.0, 1.0, 0.9, 1.9, 1.8), [0.0, -1.2, -1.2])
    assert not ok and '2회' in detail


# 중복 stamp → 실패(역행 여부와 무관)
@pytest.mark.parametrize('offsets', [STEADY, [0.0, -1.5]])
def test_duplicate_fails(offsets):
    ok, detail = check_stamp_order(ns(0.0, 1.0, 1.0), offsets)
    assert not ok and '중복' in detail


# 임계 경계: 역행 0.05 s 이하는 역행 없음으로 본다
def test_threshold_boundary():
    assert not check_stamp_order(ns(0.0, 1.0, 0.98), [0.0, -0.05])[0]
    assert check_stamp_order(ns(0.0, 1.0, 0.98), [0.0, -0.06])[0]


# 감소 폭 경계: 역행량 + 0.05 s까지 허용
def test_decrease_margin_boundary():
    assert check_stamp_order(ns(0.0, 1.0, 0.01), [0.0, -0.95])[0]
    assert not check_stamp_order(ns(0.0, 1.0, 0.01), [0.0, -0.93])[0]


# 역행량: 앞의 최댓값에서 뒤 값까지 가장 큰 감소(정방향 이동은 역행 아님)
def test_clock_regression_amount():
    assert clock_regression_s([]) == 0.0
    assert clock_regression_s([0.0, 0.5, 1.0]) == 0.0
    assert clock_regression_s([0.0, 0.3, -0.9, -0.8]) == pytest.approx(1.2)
