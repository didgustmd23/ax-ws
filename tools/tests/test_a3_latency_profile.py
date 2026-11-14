"""진단 계측이 A3 응답·멱등성·내구성·동시 요청 식별을 바꾸지 않는지 확인합니다."""
from pathlib import Path
import sys
import tempfile
import threading
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/tests'))
sys.path.insert(0, str(ROOT / 'src/ax_task_executor'))
from a3_latency_profile import LeaseProfiler
from ax_task_executor.lease_check import LeaseChecker, LeaseLedger
from test_a3_lease_check import tool_request


class LatencyProfileTests(unittest.TestCase):
    """ROS 노드 없이 실제 SQLite 원장을 사용해 시험 계측기의 투명성을 검사합니다."""

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.ledger = LeaseLedger(Path(self.folder.name) / 'lease.db', 'robot_1')
        self.addCleanup(self.ledger.close)
        self.checker = LeaseChecker('robot_1', 'dev_allow', 'LEASE_EXPIRED', 2.0, self.ledger)

    def test_profile_preserves_response_idempotency_and_full_sync(self):
        """계측 전후 같은 응답·수락 기록 하나·FULL 동기화를 유지합니다."""
        request = tool_request()
        original = self.checker.check(request)
        profiler = LeaseProfiler(self.checker)
        self.assertEqual(self.checker.check(request), original)
        self.assertEqual(self.ledger.accept_count(), 1)
        self.assertEqual(self.ledger.db.execute('PRAGMA synchronous').fetchone()[0], 2)
        report = profiler.report([(0.5, 0, 0, 0, request.request_id)])
        self.assertEqual(report['requests'][0]['request_id'], request.request_id)
        self.assertIn('ledger_lock_wait_s', report['stages'])
        self.assertIn('commit_s', report['stages'])

    def test_concurrent_requests_have_separate_phase_records(self):
        """서로 다른 스레드의 잠금·SQL 시간을 같은 요청으로 섞지 않습니다."""
        profiler = LeaseProfiler(self.checker)
        requests = [tool_request() for _ in range(8)]
        responses = []
        workers = [threading.Thread(target=lambda request=request: responses.append(
            self.checker.check(request))) for request in requests]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join(5)
            self.assertFalse(worker.is_alive())
        self.assertEqual(len(responses), 8)
        self.assertTrue(all(response['allowed'] for response in responses))
        self.assertEqual(self.ledger.accept_count(), 8)
        report = profiler.report([])
        self.assertEqual({row['request_id'] for row in report['requests']},
                         {request.request_id for request in requests})
        self.assertTrue(all(row['commit_s'] >= 0 for row in report['requests']))


if __name__ == '__main__':
    unittest.main()
