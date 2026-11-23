"""D1 crash/replay/cancellation boundaries without granting any actuation permission."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[2] / 'src/ax_navigation/scripts/navigation_execution.py'
spec = importlib.util.spec_from_file_location('navigation_execution', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class NavigationExecutionTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'navigation.sqlite'
        self.ledger = module.ExecutionLedger(self.path)
        self.manager = module.NavigationExecution(self.ledger)
        self.request = dict(execution_id='approved-child-id', robot='robot_1',
            assignment_id='assignment-a', assignment_generation=1, x=1., y=2., yaw=0.)

    def tearDown(self):
        self.ledger.close()
        self.directory.cleanup()

    def test_persist_before_send_and_no_replay_after_restart(self):
        sent = []
        def send(row):
            other = module.ExecutionLedger(self.path)
            self.assertEqual(other.get(row['execution_id'])['state'], 'SENDING')
            other.close()
            sent.append(row['goal_uuid'])
        self.manager.submit(send, **self.request)
        self.ledger.close()
        self.ledger = module.ExecutionLedger(self.path)
        self.manager = module.NavigationExecution(self.ledger)
        row = self.manager.submit(send, **self.request)
        self.assertEqual(sent, [row['goal_uuid']])

    def test_arguments_and_assignment_cannot_change(self):
        self.manager.submit(lambda row: None, **self.request)
        for change in (dict(x=3.), dict(assignment_generation=2), dict(robot='robot_2')):
            with self.assertRaises(ValueError):
                self.manager.submit(lambda row: self.fail('resent'), **(self.request | change))

    def test_unknown_send_keeps_robot_reserved(self):
        def fail(row):
            raise RuntimeError('response lost')
        with self.assertRaises(RuntimeError):
            self.manager.submit(fail, **self.request)
        self.assertEqual(self.ledger.get(self.request['execution_id'])['state'], 'UNKNOWN')
        with self.assertRaises(ValueError):
            self.manager.submit(lambda row: None, **(self.request | dict(execution_id='another')))

    def start(self):
        self.manager.submit(lambda row: None, **self.request)
        self.manager.accepted(self.request['execution_id'], True)
        return self.request['execution_id']

    def test_cancel_ack_waits_for_actual_result(self):
        key = self.start()
        self.manager.cancel(key, lambda row: None)
        self.manager.cancel_response(key, True)
        self.assertEqual(self.ledger.get(key)['state'], 'CANCEL_REQUESTED')
        self.manager.result(key, 5)
        self.assertEqual(self.ledger.get(key)['state'], 'CANCELED')

    def test_arrival_is_not_stopped(self):
        key = self.start()
        self.manager.result(key, 4)
        self.assertEqual(self.ledger.get(key)['state'], 'ARRIVED')
        with self.assertRaises(ValueError):
            self.manager.result(key, 6)

    def test_rejected_cancel_and_aborted_goal(self):
        key = self.start()
        self.manager.cancel(key, lambda row: None)
        self.manager.cancel_response(key, False)
        self.assertEqual(self.ledger.get(key)['state'], 'ACTIVE')
        self.manager.result(key, 6)
        self.assertEqual(self.ledger.get(key)['state'], 'ABORTED')

    def test_invalid_goal_does_not_reserve(self):
        with self.assertRaises(ValueError):
            self.manager.submit(lambda row: self.fail('sent'), **(self.request | dict(x=float('nan'))))
        self.assertIsNone(self.ledger.get(self.request['execution_id']))

    def test_prepared_crash_is_not_resent(self):
        self.ledger.reserve(**self.request)
        self.manager.submit(lambda row: self.fail('resent'), **self.request)


if __name__ == '__main__':
    unittest.main()
