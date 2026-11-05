# PID 시작 시각/소유권과 실행 범위를 검사한다. 다른 프로세스를 종료하지 않는지가 중요하다.
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
    def test_world_selection_restarts_same_world_with_wrong_tool(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'src/ax_description').mkdir(parents=True)
            world = root / 'install/ax_description/share/ax_description/worlds/ax_test.world'
            world.parent.mkdir(parents=True)
            world.touch()
            (root / 'install/setup.bash').touch()
            found = {'launch': [{'pid': 123, 'env': {},
                                'args': 'ros2 launch ax_control simulation.launch.py tool:=robotiq world:=ax_test.world'}],
                     'gazebo': [{'pid': 124}], 'remote': [], 'keyboard': [], 'terminal': []}
            args = SimpleNamespace(workspace=directory, action='start', tool='epick',
                                   world='ax_test.world', switch_world=True, port=8765)
            # 재시작 대상 선택을 검사하며 실제 프로세스는 종료하지 않는다.
            with patch.object(desktop, 'discover', return_value=found), \
                    patch.object(desktop, 'stop_processes', side_effect=RuntimeError('restart selected')) as stop:
                with self.assertRaisesRegex(RuntimeError, 'restart selected'):
                    desktop.run(args)
            stop.assert_called_once_with(found['launch'] + found['gazebo'])

    def test_remote_does_not_reuse_launch_after_gazebo_died(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'src/ax_description').mkdir(parents=True)
            (root / 'install').mkdir()
            (root / 'install/setup.bash').touch()
            (root / 'tools').mkdir()
            (root / 'tools/start_remote.sh').touch()
            old = {'launch': [{'pid': 123, 'env': {}}], 'remote': [{'pid': 125}],
                   'keyboard': [], 'gazebo': [], 'terminal': []}
            cleaned = dict(old, launch=[])
            args = SimpleNamespace(workspace=directory, action='remote', tool='epick', port=8765)
            with patch.object(desktop, 'discover', side_effect=[old, cleaned]), \
                    patch.object(desktop, 'stop_processes') as stop, \
                    patch.object(desktop, 'spawn') as spawn, \
                    patch.object(desktop, 'alive', return_value=True), \
                    patch.object(desktop, 'remote_state', return_value={'robot': 'robot_1'}):
                result = desktop.run(args)
            stop.assert_called_once_with(old['launch'])
            spawn.assert_not_called()
            self.assertIsNone(result['launch_pid'])

    def test_start_reuses_gazebo_without_starting_remote(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'src/ax_description').mkdir(parents=True)
            (root / 'install').mkdir()
            (root / 'install/setup.bash').touch()
            found = {'launch': [{'pid': 123, 'env': {}}], 'remote': [], 'keyboard': [], 'gazebo': [{'pid': 124}]}
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
