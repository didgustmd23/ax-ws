"""D3 grants, physical occupancy, restart, FIFO waiting and durable replay."""
# [검사 범위] D3 허가·점유 원장을 확인합니다. 모의 관측으로 단일 허가·완전 이탈·재시작을 검사합니다.
from pathlib import Path
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_corridor_manager'))
from ax_corridor_manager.manager import (  # noqa: E402
    APPROACHING, BLOCKED_UNKNOWN, CorridorManager, FREE, INSIDE, OCCUPIED, OUTSIDE, RESERVED, UNKNOWN)


class CorridorTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / 'ledger.sqlite3'
        self.now = 100.0
        self.manager = self.open_manager()
        self.addCleanup(lambda: self.manager.close())

    def open_manager(self, enabled=True):
        return CorridorManager(self.path, ['corridor'], ['robot_1', 'robot_2'],
                               enabled=enabled, clock=lambda: self.now)

    def ready(self, first=OUTSIDE, second=OUTSIDE):
        for robot, state in (('robot_1', first), ('robot_2', second)):
            self.manager.observe(robot, {'corridor': (state, state != UNKNOWN)})
            self.manager.stopped(robot, True)
        self.manager.assign(['robot_1', 'robot_2'])

    def state(self):
        return self.manager.snapshot()[1][0]

    def test_no_observations_or_approval_cannot_issue_grant(self):
        self.assertEqual(self.state()['state'], BLOCKED_UNKNOWN)
        self.assertFalse(self.manager.request('corridor', 'robot_1', 'none')['granted'])
        self.manager.enabled = False
        self.ready()
        self.assertEqual(self.manager.request('corridor', 'robot_1', 'unapproved')['reason_code'],
                         'REGIONS_NOT_APPROVED')

    def test_normal_entry_and_release_require_new_full_exit(self):
        # [핵심] 허가 전 관측이나 중심점 이탈만으로 해제하면 안 됩니다. 허가 이후의 완전 이탈 확인이 필요합니다.
        self.ready()
        self.assertEqual(self.state()['state'], FREE)
        grant = self.manager.request('corridor', 'robot_1', 'entry')
        self.assertTrue(grant['granted'])
        self.assertEqual(self.state()['state'], RESERVED)
        self.assertFalse(self.manager.release('corridor', 'robot_1', grant['grant_id'], 'too-early')['accepted'])
        self.ready(first=INSIDE)
        self.assertEqual(self.state()['state'], OCCUPIED)
        self.assertFalse(self.manager.release('corridor', 'robot_1', grant['grant_id'], 'inside')['accepted'])
        self.ready()
        # OUTSIDE alone does not revoke the owner's grant before the release handshake.
        self.assertEqual(self.state()['active_grant_id'] if 'active_grant_id' in self.state()
                         else self.state()['grant_id'], grant['grant_id'])
        released = self.manager.release('corridor', 'robot_1', grant['grant_id'], 'exit')
        self.assertTrue(released['accepted'])
        self.assertEqual(self.state()['state'], FREE)
        self.assertGreater(released['corridor_revision'], grant['corridor_revision'])

    def test_two_concurrent_requests_never_produce_two_active_grants(self):
        # [핵심] 두 로봇이 동시에 요청해도 같은 통로의 활성 허가는 하나만 발급돼야 합니다.
        self.ready()
        barrier = threading.Barrier(3)
        results = []

        def request(robot):
            barrier.wait()
            results.append(self.manager.request('corridor', robot, 'request-' + robot))

        threads = [threading.Thread(target=request, args=(robot,)) for robot in ('robot_1', 'robot_2')]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(3)
        self.assertEqual(len(results), 2)
        self.assertEqual(sum(result['granted'] for result in results), 1)

    def test_cached_requests_same_response_and_changed_payload_rejected(self):
        self.ready()
        first = self.manager.request('corridor', 'robot_1', 'same')
        self.assertEqual(self.manager.request('corridor', 'robot_1', 'same'), first)
        changed = self.manager.request('corridor', 'robot_2', 'same')
        self.assertFalse(changed['granted'])
        self.assertEqual(changed['reason_code'], 'DUPLICATE_REQUEST')

    def test_silence_unknown_and_timeout_preserve_owner_and_grant(self):
        self.ready()
        grant = self.manager.request('corridor', 'robot_1', 'entry')
        self.ready(first=INSIDE)
        self.now += 10
        row = self.state()
        self.assertEqual(row['state'], BLOCKED_UNKNOWN)
        self.assertEqual(row['owner'], 'robot_1')
        self.assertEqual(row['grant_id'], grant['grant_id'])
        self.assertFalse(self.manager.release('corridor', 'robot_1', grant['grant_id'], 'stale')['accepted'])

    def test_restart_preserves_owner_blocks_until_fresh_exit_and_replays(self):
        # [핵심] 재시작해도 기존 owner/grant를 보존하고 불명 상태로 시작한 뒤 새 이탈 관측을 기다립니다.
        self.ready()
        grant = self.manager.request('corridor', 'robot_1', 'entry')
        self.ready(first=INSIDE)
        revision = self.state()['revision']
        self.manager.close()
        self.manager = self.open_manager()
        self.assertEqual(self.state()['state'], BLOCKED_UNKNOWN)
        self.assertEqual(self.state()['grant_id'], grant['grant_id'])
        self.assertGreater(self.state()['revision'], revision)
        self.assertEqual(self.manager.request('corridor', 'robot_1', 'entry'), grant)
        self.ready()
        self.assertTrue(self.manager.release('corridor', 'robot_1', grant['grant_id'], 'exit')['accepted'])

    def test_waiting_robot_gets_next_turn(self):
        self.ready()
        grant = self.manager.request('corridor', 'robot_1', 'first')
        self.assertFalse(self.manager.request('corridor', 'robot_2', 'wait')['granted'])
        self.ready()
        self.assertTrue(self.manager.release('corridor', 'robot_1', grant['grant_id'], 'exit')['accepted'])
        self.assertEqual(self.manager.request('corridor', 'robot_1', 'again')['reason_code'], 'WAITING_TURN')
        self.assertTrue(self.manager.request('corridor', 'robot_2', 'next')['granted'])

    def test_actual_stop_and_assignment_are_required(self):
        self.ready()
        self.manager.stopped('robot_1', False)
        self.assertEqual(self.manager.request('corridor', 'robot_1', 'moving')['reason_code'], 'STOP_NOT_CONFIRMED')
        self.manager.stopped('robot_1', True)
        self.manager.assign([])
        self.assertEqual(self.manager.request('corridor', 'robot_1', 'unassigned')['reason_code'], 'ASSIGNMENT_STALE')

    def test_other_robot_inside_or_unknown_blocks_release_and_entry(self):
        self.ready(second=INSIDE)
        self.assertFalse(self.manager.request('corridor', 'robot_1', 'blocked')['granted'])
        self.ready()
        grant = self.manager.request('corridor', 'robot_1', 'entry')
        self.ready(second=UNKNOWN)
        self.assertFalse(self.manager.release('corridor', 'robot_1', grant['grant_id'], 'blocked-exit')['accepted'])

    def test_approaching_owner_keeps_grant_and_unrelated_release_rejected(self):
        self.ready(first=APPROACHING)
        grant = self.manager.request('corridor', 'robot_1', 'entry')
        self.assertTrue(grant['granted'])
        self.now += 0.1
        self.ready(first=APPROACHING)
        self.assertEqual(self.state()['grant_id'], grant['grant_id'])
        self.assertEqual(self.manager.release('corridor', 'robot_2', grant['grant_id'], 'wrong')['reason_code'],
                         'GRANT_MISMATCH')

    def test_old_release_replay_does_not_clear_a_new_owners_grant(self):
        # [핵심] 옛 해제 요청의 재전송이 새로 허가받은 로봇의 권한을 지우면 안 됩니다.
        self.ready()
        first = self.manager.request('corridor', 'robot_1', 'first')
        self.ready()
        released = self.manager.release('corridor', 'robot_1', first['grant_id'], 'release')
        self.assertTrue(released['accepted'])
        second = self.manager.request('corridor', 'robot_2', 'second')
        self.assertTrue(second['granted'])
        self.assertEqual(self.manager.release('corridor', 'robot_1', first['grant_id'], 'release'), released)
        self.assertEqual(self.state()['grant_id'], second['grant_id'])

    def test_missing_observation_never_proves_exit(self):
        self.ready()
        grant = self.manager.request('corridor', 'robot_1', 'first')
        self.manager.observe('robot_1', {})
        self.assertEqual(self.state()['state'], BLOCKED_UNKNOWN)
        self.assertFalse(self.manager.release('corridor', 'robot_1', grant['grant_id'], 'exit')['accepted'])

    def test_waiter_timeout_does_not_revoke_active_grant(self):
        # [핵심] 대기표 만료와 실제 통로 점유는 다릅니다. 시간이 지났다고 활성 허가를 회수하면 안 됩니다.
        self.ready()
        grant = self.manager.request('corridor', 'robot_1', 'first')
        self.manager.request('corridor', 'robot_2', 'wait')
        self.now += 6
        self.ready(first=APPROACHING)
        self.assertEqual(self.state()['grant_id'], grant['grant_id'])
        self.ready()
        self.assertTrue(self.manager.release('corridor', 'robot_1', grant['grant_id'], 'exit')['accepted'])
        self.assertTrue(self.manager.request('corridor', 'robot_1', 'again')['granted'])


if __name__ == '__main__':
    unittest.main()
