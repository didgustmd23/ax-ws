"""ROS-independent D1 expiry, pending-acceptance and persistent hold boundaries."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPTS = Path(__file__).resolve().parents[2] / 'src/ax_navigation/scripts'
def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

watchdog = load('navigation_watchdog')
execution = load('navigation_execution')


class WatchdogTest(unittest.TestCase):
    def ready(self):
        guard = watchdog.InputWatchdog()
        for source in guard.LIMITS:
            guard.observe(source, 10., 100.)
        return guard

    def test_missing_and_healthy(self):
        self.assertEqual(watchdog.InputWatchdog().reason(10, 100), 'LOCALIZATION_MISSING')
        self.assertEqual(self.ready().reason(10, 100), '')

    def test_each_input_can_expire(self):
        for source in watchdog.InputWatchdog.LIMITS:
            guard = self.ready()
            guard.observe(source, 8., 100.)
            self.assertEqual(guard.reason(10, 100), source.upper() + '_STALE')

    def test_receive_timeout_even_when_sim_clock_paused(self):
        self.assertEqual(self.ready().reason(10, 101.1), 'LOCALIZATION_STALE')

    def test_replayed_old_stamp_does_not_refresh(self):
        guard = self.ready()
        guard.observe('front', 9., 100.)
        self.assertEqual(guard.reason(10, 100), 'FRONT_STALE')

    def test_invalid_and_future_input(self):
        guard = self.ready()
        guard.observe('localization', 10, 100, False)
        self.assertEqual(guard.reason(10, 100), 'LOCALIZATION_INVALID')
        for bad in (11., float('nan'), 0.):
            guard = self.ready()
            guard.observe('rear', bad, 100)
            self.assertIn('TIME_INVALID', guard.reason(10, 100))

    def test_clock_rewind_invalidates_all_inputs(self):
        guard = self.ready()
        guard.reason(10, 100)
        self.assertEqual(guard.reason(9, 100), 'ROS_TIME_REWIND')
        self.assertEqual(guard.reason(9, 100), 'LOCALIZATION_MISSING')

    def test_persistent_first_cause_blocks_new_goals_after_cancel(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'ledger.sqlite'
            ledger = execution.ExecutionLedger(path)
            manager = execution.NavigationExecution(ledger)
            request = dict(execution_id='e1', robot='robot_1', assignment_id='a1',
                           assignment_generation=1, x=1., y=0., yaw=0.)
            manager.submit(lambda row: None, **request)
            manager.accepted('e1', True)
            ledger.hold('robot_1', 'e1', 'FRONT_STALE')
            ledger.hold('robot_1', 'e1', 'LOCALIZATION_INVALID')
            manager.cancel('e1', lambda row: None)
            manager.result('e1', 5)
            ledger.close()
            ledger = execution.ExecutionLedger(path)
            try:
                self.assertEqual(ledger.held('robot_1')['reason'], 'FRONT_STALE')
                with self.assertRaises(ValueError):
                    ledger.reserve(**(request | dict(execution_id='e2')))
            finally:
                ledger.close()


if __name__ == '__main__':
    unittest.main()
