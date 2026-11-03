import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('desktop_session', Path(__file__).resolve().parents[1] / 'desktop_session.py')
desktop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(desktop)


class SessionOwnership(unittest.TestCase):
    def test_start_reuses_gazebo_without_starting_remote(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'src/ax_description').mkdir(parents=True)
            (root / 'install').mkdir()
            (root / 'install/setup.bash').touch()
            found = {'launch': [{'pid': 123, 'env': {}}], 'remote': [], 'keyboard': [], 'gazebo': []}
            args = SimpleNamespace(workspace=directory, action='start', tool='robotiq', port=8765)
            with patch.object(desktop, 'discover', return_value=found), \
                    patch.object(desktop, 'spawn') as spawn, patch.object(desktop, 'remote_state') as remote:
                result = desktop.run(args)
            spawn.assert_not_called()
            remote.assert_not_called()
            self.assertEqual(result['launch_pid'], 123)
            self.assertTrue(Path(result['terminal_rcfile']).is_file())
            self.assertNotIn('remote_pid', result)

    def test_pid_start_time_must_match(self):
        item = desktop.process(os.getpid())
        self.assertTrue(desktop.alive(item))
        self.assertFalse(desktop.alive(dict(item, start='wrong-start-time')))

    def test_stop_does_not_kill_unrelated_process(self):
        own = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
        other = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
        try:
            desktop.stop_processes([desktop.process(own.pid)])
            own.wait(timeout=3)
            self.assertIsNone(other.poll())
        finally:
            for child in (own, other):
                if child.poll() is None:
                    child.terminate()
                child.wait(timeout=3)
