"""미지원 작업이 실제 실행으로 들어가지 않고 모의 결과가 실측으로 둔갑하지 않는지 확인한다."""

from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

from ax_interfaces.action import ExecuteLocalTask
from ax_interfaces.msg import LoadState
import pytest
from rclpy.action import GoalResponse

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.mock_executor import MockExecutor, SUPPORTED_MOCK_TASK_KINDS  # noqa: E402


@pytest.mark.parametrize('kind', [
    ExecuteLocalTask.Goal.PICK_AND_STOW, ExecuteLocalTask.Goal.UNLOAD_AND_PLACE,
    ExecuteLocalTask.Goal.RETRACT_ARM, ExecuteLocalTask.Goal.LOCK_HANDOFF,
    ExecuteLocalTask.Goal.UNLOCK_HANDOFF])
def test_unsupported_goal_has_no_execution_side_effects(kind):
    # Goal 거절 전에 원장 조회·실행 자리 선점·하위 명령을 시작하지 않아야 한다.
    server = Mock()
    goal = SimpleNamespace(task_kind=kind, execution=SimpleNamespace(execution_id='scope-test'))
    assert MockExecutor.accept_goal(server, goal) == GoalResponse.REJECT
    server._has_result.assert_not_called()
    server.state.claim.assert_not_called()


def test_only_mock_confirm_load_is_supported():
    assert SUPPORTED_MOCK_TASK_KINDS == frozenset((ExecuteLocalTask.Goal.CONFIRM_LOAD,))


def test_mock_result_cannot_supply_real_observation_provenance():
    # Action 결과와 Topic 메시지는 다르다. 결과에 없는 source를 소비자가 추정하면 안 된다.
    result = MockExecutor.make_result('STOWED')
    assert result['load_state'] == LoadState.STOWED
    assert 'source' not in result
    assert '실제 관측 아님' in result['status']['detail']
    observation = LoadState()
    assert observation.source == LoadState.SOURCE_UNKNOWN
    assert not observation.meta.valid
