"""A3 후퇴 근거의 자세·연속 정지·누락·만료를 검증한다."""

from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.arm_retraction import ArmRetractionObserver  # noqa: E402


def sample(t, position=0.4, velocity=0.0):
    # 실제 JointState와 같은 필드를 쓰되 순수 판정 시험에는 ROS 실행이 필요 없다.
    ns = round(t * 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9)),
        name=['arm_joint'], position=[position], velocity=[velocity])


def observer():
    return ArmRetractionObserver(['arm_joint'], [0.4], approved=True)


def confirm(checker):
    for t in (1.0, 1.2, 1.4, 1.6):
        result = checker.update(sample(t), t, t)
    return result


def test_no_approved_pose_never_confirms():
    assert confirm(ArmRetractionObserver()).reason == 'POSE_NOT_APPROVED'


def test_requires_actual_pose_and_continuous_stop():
    checker = observer()
    assert checker.update(sample(1), 1, 1).state == 'UNKNOWN'
    for t in (1.2, 1.4, 1.6):
        result = checker.update(sample(t), t, t)
    assert result.state == 'RETRACTED'
    assert checker.snapshot(1.7, 1.7).state == 'RETRACTED'


@pytest.mark.parametrize('position,velocity,reason', [
    (0.7, 0.0, 'POSE_MISMATCH'), (0.4, 0.02, 'ARM_MOVING')])
def test_position_or_movement_blocks_completion(position, velocity, reason):
    checker = observer()
    confirm(checker)
    assert checker.update(sample(1.8, position, velocity), 1.8, 1.8).reason == reason


@pytest.mark.parametrize('fault', ['missing', 'duplicate', 'nan', 'no_velocity'])
def test_invalid_joint_fields_clear_prior_success(fault):
    checker = observer()
    confirm(checker)
    message = sample(1.8)
    if fault == 'missing':
        message.name = ['other_joint']
    elif fault == 'duplicate':
        message.name *= 2
    elif fault == 'nan':
        message.velocity = [float('nan')]
    else:
        message.velocity = []
    assert checker.update(message, 1.8, 1.8).state == 'UNKNOWN'


@pytest.mark.parametrize('stamp,now', [(1.6, 1.8), (1.5, 1.8), (2.0, 1.8), (1.8, 2.5)])
def test_replayed_future_or_old_stamp_clears_success(stamp, now):
    checker = observer()
    confirm(checker)
    assert checker.update(sample(stamp), now, now).state == 'UNKNOWN'


def test_gap_restarts_stop_window():
    checker = observer()
    checker.update(sample(1), 1, 1)
    assert checker.update(sample(1.6), 1.6, 1.6).state == 'UNKNOWN'


@pytest.mark.parametrize('ros,steady', [(2, 2), (1.7, 2), (1, 1.7), (float('nan'), 1.7)])
def test_stale_paused_or_reversed_clock_invalidates(ros, steady):
    checker = observer()
    confirm(checker)
    assert checker.snapshot(ros, steady).state == 'UNKNOWN'


def test_approved_empty_pose_rejected():
    with pytest.raises(ValueError):
        ArmRetractionObserver(approved=True)
