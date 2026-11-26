"""원본 C4 요청 생성 → 원본 A2 명령 확인 → 내 A3 원장까지 직접 연결합니다. 물리 실행은 없습니다."""
from concurrent.futures import Future
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
for package in ('ax_c4', 'ax_task_executor'):
    sys.path.insert(0, str(ROOT / 'src' / package))
sys.path.insert(0, str(ROOT / 'tools/tests'))
from ax_c4.manipulate_object_server import ManipulateObjectServer
from ax_interfaces.srv import ToolCommand
from ax_task_executor.lease_check import LeaseChecker, LeaseLedger
from ax_task_executor.runtime_authority import RuntimeAuthority
from test_a3_lease_check import load_suction_tool_node


def completed(value):
    """ROS 전송 대신 완료된 Future만 제공하며 검사 함수 자체는 대체하지 않습니다."""
    future = Future()
    future.set_result(value)
    return future


class C4ToolHandoffTests(unittest.TestCase):
    """배정·Goal·lease 준비는 시험용이며 실제 D4 응답·발급기를 연결한 결과는 아닙니다."""

    def setUp(self):
        """격리 원장과 제어 가능한 내 권한 상태를 준비합니다."""
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.now = 100.0
        self.authority = RuntimeAuthority('robot_1', clock=lambda: self.now)
        self.addCleanup(self.authority.close)
        self.ledger = LeaseLedger(Path(folder.name) / 'lease.db', 'robot_1')
        self.addCleanup(self.ledger.close)
        self.checker = LeaseChecker('robot_1', 'operational', '', 1.5, self.ledger,
                                    authority=self.authority)
        self.goal = SimpleNamespace(execution_id='c4-pick', goal_id='a' * 32, object_id='box')
        self.execution = SimpleNamespace(cancel_requested=False, revoke_reason='',
                                         lease=SimpleNamespace(lease_generation=7))
        self.handle = SimpleNamespace(is_cancel_requested=False)
        self.tool_module = load_suction_tool_node()
        self.requests = []

        def check_request(request):
            self.requests.append(request)
            return completed(SimpleNamespace(**self.checker.check(request)))

        self.a2 = SimpleNamespace(lease_client=SimpleNamespace(
            service_is_ready=lambda: True, call_async=check_request))

        def tool_request(request):
            allowed, reason = self.tool_module.SuctionToolNode.check_lease(
                self.a2, request, '1' * 64, self.tool_module.boottime_seconds())
            return completed(SimpleNamespace(accepted=allowed, reason_code=reason))

        self.c4 = SimpleNamespace(
            robot_id='robot_1', mock_execution=False,
            # C4 내부 준비는 시험 대역이다. 그래도 실제 A2→A3 권한 검사가 거절하면 성공이 되지 않는다.
            _ensure_runtime=lambda *args: SimpleNamespace(ok=True),
            tool_client=SimpleNamespace(wait_for_service=Mock(return_value=True), call_async=tool_request),
            _wait_future=lambda future, *args: future.result())

    def ready(self):
        """확인된 배정·현재 부모 Goal·미만료 lease를 시험 상태에 등록합니다."""
        self.authority.begin_assignment_check('check', 'assignment', 1)
        self.assertTrue(self.authority.confirm_assignment(SimpleNamespace(
            request_id='check', a_session_id=self.authority.session_id, d_session_id='test-d4',
            assignment_generation=1, sequence=1, valid=True, validity_s=2.0)))
        self.authority.activate_goal(SimpleNamespace(
            execution_id=self.goal.execution_id, assignment_id='assignment', assignment_generation=1),
            self.goal.goal_id)
        self.authority.set_lease(self.goal.execution_id, self.goal.goal_id, 7, 101.5)

    def call(self):
        """원본 C4가 만든 ToolCommand를 원본 A2의 검사 경로로 전달합니다."""
        return ManipulateObjectServer._call_tool(
            self.c4, self.goal, ToolCommand.Request.ATTACH, self.handle, self.execution)

    def test_original_c4_and_a2_preserve_parent_identity_to_a3(self):
        """부모 Goal·실행 ID·세대가 바뀌지 않고 내 수락 원장에 한 건 기록됩니다."""
        self.ready()
        self.assertEqual(self.call(), (True, ''))
        request = self.requests[-1]
        self.assertEqual((request.execution_id, request.goal_id, request.lease_generation),
                         ('c4-pick', 'a' * 32, 7))
        self.assertEqual(self.ledger.accept_count(), 1)

    def test_c4_request_without_confirmed_assignment_is_denied(self):
        """C4가 요청을 만들어도 내 배정 확인이 없으면 물리 허가가 되지 않습니다."""
        self.assertEqual(self.call(), (False, 'ASSIGNMENT_STALE'))
        self.assertEqual(self.ledger.accept_count(), 0)

    def test_revoked_lease_is_denied_through_original_a2(self):
        """철회가 상대 호출 경로를 거쳐도 내 A3에서 거절됩니다."""
        self.ready()
        self.authority.revoke(self.goal.execution_id, self.goal.goal_id, 7)
        self.assertEqual(self.call(), (False, 'LEASE_REVOKED'))

    def test_wrong_parent_goal_is_denied(self):
        """다른 조작 Goal의 요청이 내 현재 작업 권한을 빌려 쓰지 못합니다."""
        self.ready()
        self.goal.goal_id = 'b' * 32
        self.assertEqual(self.call(), (False, 'GOAL_MISMATCH'))

    def test_expired_lease_is_denied(self):
        """응답 형식이 맞아도 실행 기한이 끝나면 허용하지 않습니다."""
        self.ready()
        self.now = 101.5
        self.assertEqual(self.call(), (False, 'LEASE_EXPIRED'))
