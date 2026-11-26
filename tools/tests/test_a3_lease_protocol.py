"""A3 lease 토픽의 영속 세대·멱등·갱신·철회와 기존 명령 확인의 결합을 검사합니다."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sqlite3
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_task_executor'))
sys.path.insert(0, str(ROOT / 'tools/tests'))
from ax_interfaces.msg import BudgetGrant, ExecutionLeaseRequest
from ax_task_executor.lease_check import LeaseChecker, LeaseLedger
from ax_task_executor.lease_protocol import LeaseProtocol
from ax_task_executor.runtime_authority import RuntimeAuthority
from test_a3_lease_check import tool_request


class LeaseProtocolTests(unittest.TestCase):
    """배정·Goal·예산은 시험에서 명시하며 물리 명령·외부 토픽은 사용하지 않습니다."""

    def setUp(self):
        """임시 실제 SQLite 원장과 제어 가능한 BOOTTIME을 준비합니다."""
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.now = 100.0
        self.authority = RuntimeAuthority('robot_1', clock=lambda: self.now)
        self.addCleanup(self.authority.close)
        self.ledger = LeaseLedger(Path(self.folder.name) / 'lease.db', 'robot_1')
        self.addCleanup(self.ledger.close)
        self.protocol = LeaseProtocol(self.authority, self.ledger)
        self.execution = SimpleNamespace(execution_id='exec-1', assignment_id='asg-1', assignment_generation=1)
        self.budget = BudgetGrant(budget_grant_id='budget-1', kind='GRASP_ATTEMPT',
                                  granted_attempts=2, allowed_execution_ids=['exec-1'], time_budget_s=10.0)

    def ready(self, authority=None, protocol=None):
        """상위 확인을 받은 현재 Goal에만 예산을 등록합니다."""
        authority, protocol = authority or self.authority, protocol or self.protocol
        authority.begin_assignment_check('check', 'asg-1', 1)
        self.assertTrue(authority.confirm_assignment(SimpleNamespace(
            request_id='check', a_session_id=authority.session_id, d_session_id='test-d4',
            assignment_generation=1, sequence=1, valid=True, validity_s=2.0)))
        authority.activate_goal(self.execution, 'a' * 32)
        protocol.register_budgets('exec-1', 'a' * 32, [self.budget])

    def request(self, request_id='req-1', kind=1, generation=0):
        """C4의 실제 공통 메시지 형식으로 요청을 구성합니다."""
        return ExecutionLeaseRequest(request_id=request_id, kind=kind, execution_id='exec-1',
                                     goal_id='a' * 32, c_session_id='c4-session',
                                     budget_grant_id='budget-1', lease_generation=generation)

    def command(self):
        """실제 A2 형태의 명령 확인을 같은 RuntimeAuthority·원장에 연결합니다."""
        request = tool_request()
        request.execution_id, request.goal_id = 'exec-1', 'a' * 32
        request.lease_generation = self.authority.context.lease_generation
        checker = LeaseChecker('robot_1', 'operational', '', 1.5, self.ledger, authority=self.authority)
        return checker.check(request)

    def test_first_grant_and_command_service_share_authority(self):
        """토픽 grant의 세대가 그대로 명령 확인의 수락 권한이 됩니다."""
        self.ready()
        grant, events = self.protocol.handle(self.request())
        self.assertTrue(grant['granted'])
        self.assertEqual(grant['lease_generation'], 1)
        self.assertEqual(grant['lease_s'], 1.5)
        self.assertFalse(events)
        self.assertTrue(self.command()['allowed'])
        self.assertEqual(self.ledger.accept_count(), 1)

    def test_denial_is_cached_before_later_assignment(self):
        """권한 준비 뒤에도 같은 거절 요청을 허용으로 바꾸지 않습니다."""
        request = self.request()
        denied, _ = self.protocol.handle(request)
        self.ready()
        self.assertEqual(self.protocol.handle(request)[0], denied)
        self.assertFalse(denied['granted'])
        self.assertTrue(self.protocol.handle(self.request('new'))[0]['granted'])

    def test_duplicate_does_not_extend_lease_and_mismatch_precedes_revocation(self):
        """같은 ID의 재수신은 기한·세대를 늘리지 않고 다른 내용이면 먼저 중복 오류입니다."""
        self.ready()
        request = self.request()
        original, _ = self.protocol.handle(request)
        deadline = self.authority.context.lease_deadline
        self.now += 0.2
        self.assertEqual(self.protocol.handle(request)[0], original)
        self.assertEqual(self.authority.context.lease_deadline, deadline)
        self.protocol.revoke()
        request.c_session_id = 'changed'
        self.assertEqual(self.protocol.handle(request)[0]['reason_code'], 'DUPLICATE_REQUEST')

    def test_renew_keeps_generation_and_expired_renew_is_rejected(self):
        """유효 세대만 갱신하며 만료된 세대를 RENEW로 되살리지 않습니다."""
        self.ready()
        self.protocol.handle(self.request())
        self.now += 0.2
        renewed, _ = self.protocol.handle(self.request('renew', 2, 1))
        self.assertTrue(renewed['granted'])
        self.assertEqual(renewed['lease_generation'], 1)
        self.assertAlmostEqual(self.authority.context.lease_deadline, 101.7)
        self.now = 101.7
        self.assertEqual(self.protocol.handle(self.request('late', 2, 1))[0]['reason_code'], 'LEASE_EXPIRED')

    def test_clean_resync_revokes_previous_generation_before_new_grant(self):
        """명령 수락 기록이 없을 때만 새 세대를 주며 이전 세대 철회 정보를 남깁니다."""
        self.ready()
        self.protocol.handle(self.request())
        self.now = 101.5
        grant, events = self.protocol.handle(self.request('resync'))
        self.assertTrue(grant['granted'])
        self.assertEqual(grant['lease_generation'], 2)
        self.assertEqual(grant['lease_s'], 0.5)
        self.assertFalse(events)
        row = self.ledger.db.execute('SELECT generation FROM lease_revocations').fetchone()
        self.assertEqual(row[0], 1)
        stale, _ = self.protocol.handle(self.request('old-renew', 2, 1))
        self.assertEqual(stale['reason_code'], 'LEASE_REVOKED')

    def test_resync_after_command_accept_is_uncertain(self):
        """토픽 원장만 보지 않고 A2/A4 명령 수락 기록까지 확인해 재실행을 막습니다."""
        self.ready()
        self.protocol.handle(self.request())
        self.assertTrue(self.command()['allowed'])
        grant, _ = self.protocol.handle(self.request('resync'))
        self.assertFalse(grant['granted'])
        self.assertEqual(grant['reason_code'], 'LEASE_RESYNC_UNCERTAIN')
        self.assertFalse(self.command()['allowed'])

    def test_revoke_denies_renew_and_command(self):
        """명시적 철회는 명령 확인부터 차단하고 부모 Goal cancel 대상도 보존합니다."""
        self.ready()
        self.protocol.handle(self.request())
        events = self.protocol.revoke('SAFETY_STOP')
        self.assertTrue(events[0]['cancel'])
        self.assertEqual(events[0]['goal_id'], 'a' * 32)
        self.assertEqual(self.protocol.handle(self.request('renew', 2, 1))[0]['reason_code'], 'LEASE_REVOKED')
        self.assertFalse(self.command()['allowed'])
        self.assertFalse(self.protocol.revoke())

    def test_wrong_goal_budget_and_unspecified_kind_are_denied(self):
        """다른 Goal·미위임 예산·미지정 종류는 허가되지 않습니다."""
        self.ready()
        for key, value, reason in (('goal_id', 'b' * 32, 'GOAL_MISMATCH'),
                                   ('budget_grant_id', 'other', 'BUDGET_EXHAUSTED'),
                                   ('kind', 0, 'AUTHORITY_INVALID')):
            request = self.request(key)
            setattr(request, key, value)
            self.assertEqual(self.protocol.handle(request)[0]['reason_code'], reason)

    def test_restart_replays_response_without_restoring_runtime_permission(self):
        """저장된 grant는 재전송해도 새 프로세스의 권한을 복원하지 않습니다."""
        self.ready()
        original, _ = self.protocol.handle(self.request())
        authority = RuntimeAuthority('robot_1', clock=lambda: self.now)
        self.addCleanup(authority.close)
        protocol = LeaseProtocol(authority, self.ledger)
        self.assertEqual(protocol.handle(self.request())[0], original)
        self.assertIsNone(authority.context)
        self.ready(authority, protocol)
        next_grant, _ = protocol.handle(self.request('new-after-restart'))
        self.assertEqual(next_grant['lease_generation'], 2)

    def test_resync_limit_and_budget_reregistration(self):
        """Goal당 재동기화 2회와 예산 재등록 금지를 지킵니다."""
        self.ready()
        with self.assertRaises(ValueError):
            self.protocol.register_budgets('exec-1', 'a' * 32, [self.budget])
        for index in range(3):
            self.assertTrue(self.protocol.handle(self.request(f'first-{index}'))[0]['granted'])
        self.assertFalse(self.protocol.handle(self.request('fourth'))[0]['granted'])

    def test_commit_failure_never_installs_runtime_authority(self):
        """저장 실패는 grant나 메모리 권한으로 바뀌지 않습니다."""
        self.ready()
        original = self.ledger.db

        class BrokenCommit:
            def execute(self, sql, *args):
                if sql == 'COMMIT':
                    raise sqlite3.OperationalError('injected storage failure')
                return original.execute(sql, *args)

        with patch.object(self.ledger, 'db', BrokenCommit()):
            grant, _ = self.protocol.handle(self.request())
        self.assertEqual(grant['reason_code'], 'A3_RECORD_FAILED')
        self.assertEqual(self.authority.context.lease_generation, 0)

    def test_concurrent_duplicate_requests_issue_one_generation(self):
        """동시 재전송에도 세대와 응답 원장 행은 하나입니다."""
        self.ready()
        with ThreadPoolExecutor(max_workers=4) as pool:
            replies = list(pool.map(lambda _: self.protocol.handle(self.request())[0], range(8)))
        self.assertTrue(all(reply == replies[0] for reply in replies))
        self.assertEqual(self.authority.context.lease_generation, 1)

    def test_latched_failure_keeps_readable_cached_response_immutable(self):
        """새 발급 장애가 생겨도 읽을 수 있는 기존 request_id 응답은 유지합니다."""
        self.ready()
        original, _ = self.protocol.handle(self.request())
        self.protocol.failed = True
        self.assertEqual(self.protocol.handle(self.request())[0], original)
        self.assertEqual(self.protocol.handle(self.request('new'))[0]['reason_code'], 'A3_RECORD_FAILED')

    def test_revoke_storage_failure_still_emits_cancel_event_once(self):
        """저장소가 고장 나도 메모리 권한 차단과 실제 철회 통지를 생략하지 않습니다."""
        self.ready()
        self.protocol.handle(self.request())
        with patch.object(self.ledger, 'db') as broken:
            broken.execute.side_effect = sqlite3.OperationalError('storage fault')
            events = self.protocol.revoke('SAFETY_STOP')
            self.assertTrue(events[0]['cancel'])
            self.assertFalse(self.protocol.revoke('SAFETY_STOP'))
        self.assertTrue(self.protocol.failed)
        self.assertFalse(self.command()['allowed'])
