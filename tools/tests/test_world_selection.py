"""실행 월드 선택(기준 질감 월드 기본 + 단색 보존 월드)을 도구마다 확인한다. GUI·Gazebo를 띄우지 않는다.

기준은 src/ax_description/config/worlds.yaml 하나다. robot_test.launch.py·desktop_session·verify_unified_runtime·
B2 mapping_drive·check_map_accuracy·run_mapping_session이 이 목록을 따르는지 본다.
"""
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / 'src/ax_description/config/worlds.yaml'
SCRIPTS = ROOT / 'src/ax_localization/scripts'
REFERENCE = 'planned/ax_warehouse.world'
PLAIN = 'planned/ax_warehouse_plain.world'
ROUTE = ROOT / 'src/ax_localization/config/routes/ax_warehouse_center.yaml'


# 파일 경로의 파이썬 모듈을 읽는다
def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


class WorldSelection(unittest.TestCase):

    # 목록: 기본은 기준 질감 월드, 두 월드 모두 있고 같은 구조·같은 출발 위치, 월드 파일이 있다
    def test_catalog(self):
        catalog = yaml.safe_load(CATALOG.read_text(encoding='utf-8'))
        worlds = catalog['worlds']

        self.assertEqual(catalog['default'], REFERENCE)
        self.assertEqual(set(worlds), {REFERENCE, PLAIN})
        self.assertEqual(worlds[PLAIN]['structure'], worlds[REFERENCE]['structure'])
        self.assertEqual(worlds[PLAIN]['spawn_positions'], worlds[REFERENCE]['spawn_positions'])

        for name in worlds:
            self.assertTrue((ROOT / 'src/ax_description/worlds' / name).is_file(), name)

    # robot_test.launch.py는 목록에서 선택지·기본값·출발 위치를 읽는다(중복 정의 없음)
    def test_launch_reads_catalog(self):
        launch = load(ROOT / 'src/ax_description/launch/robot_test.launch.py', 'world_selection_launch')
        catalog = yaml.safe_load(CATALOG.read_text(encoding='utf-8'))
        argument = next(a for a in launch.generate_launch_description().entities
                        if getattr(a, 'name', None) == 'world')

        self.assertEqual(argument.default_value[0].text, REFERENCE)
        self.assertEqual(list(argument.choices), list(catalog['worlds']))
        self.assertEqual(launch.WORLD_SPAWN_POSITIONS[PLAIN], [(-10.0, -2.4), (-6.8, 0.0), (-17.0, -2.4), (-13.0, -2.4)])
        self.assertEqual(launch.WORLD_SPAWN_POSITIONS[PLAIN], launch.WORLD_SPAWN_POSITIONS[REFERENCE])

    # desktop_session: 두 월드 선택, 기본 기준 월드, 잘못된 월드 거절
    def test_desktop_session_arguments(self):
        result = subprocess.run([sys.executable, str(ROOT / 'tools/desktop_session.py'), '--help'],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(REFERENCE, result.stdout)
        self.assertIn(PLAIN, result.stdout)

        bad = subprocess.run([sys.executable, str(ROOT / 'tools/desktop_session.py'), 'status',
                              '--world', 'planned/other.world'], capture_output=True, text=True, timeout=30)
        self.assertEqual(bad.returncode, 2)
        self.assertIn('invalid choice', bad.stderr)

        sys.path.insert(0, str(ROOT / 'tools'))
        import world_catalog
        self.assertEqual(world_catalog.default(), REFERENCE)
        self.assertEqual(world_catalog.choices(), [REFERENCE, PLAIN])

    # verify_unified_runtime: 두 월드 선택지, 잘못된 월드는 ROS 연결 전 거절
    def test_verify_unified_runtime_arguments(self):
        script = str(ROOT / 'tools/verify_unified_runtime.py')
        help_text = subprocess.run([sys.executable, script, '--help'], capture_output=True, text=True, timeout=60)

        self.assertEqual(help_text.returncode, 0, help_text.stderr)
        self.assertIn(PLAIN, help_text.stdout)

        bad = subprocess.run([sys.executable, script, '--world', 'planned/other.world'],
                             capture_output=True, text=True, timeout=60)
        self.assertEqual(bad.returncode, 2)
        self.assertIn('invalid choice', bad.stderr)

    # B2 경로 대조: 기준 월드용 경로를 단색 월드에 쓸 수 있고, 다른 구조 월드는 거절
    def test_mapping_drive_route_world(self):
        base = [sys.executable, str(SCRIPTS / 'mapping_drive.py'), '--check-only', '--route', str(ROUTE),
                '--gz-world', 'ax_test', '--robot-id', 'robot_2']

        for world, code in ((REFERENCE, 0), (PLAIN, 0), ('planned/other.world', 2)):
            with self.subTest(world=world):
                result = subprocess.run(base + ['--world', world], capture_output=True, text=True, timeout=60)
                self.assertEqual(result.returncode, code, result.stdout + result.stderr)

        tools = load(SCRIPTS / 'map_tools.py', 'world_selection_map_tools')
        self.assertTrue(tools.same_world_structure(REFERENCE, PLAIN))
        self.assertFalse(tools.same_world_structure(REFERENCE, 'planned/other.world'))
        self.assertTrue(tools.same_world_structure('x.world', 'x.world'))

    # run_mapping_session.sh: 단색 월드는 월드 검사를 통과하고(뒤 검사에서 멈춤), 목록 밖 월드는 거절
    def test_run_mapping_session_world(self):
        session = ROOT / 'src/ax_localization/scripts/run_mapping_session.sh'
        plain = subprocess.run(['bash', str(session), '--world', PLAIN, '--route', str(ROUTE), '--robot-id', 'robot_1'],
                               capture_output=True, text=True, timeout=120, stdin=subprocess.DEVNULL)
        bad = subprocess.run(['bash', str(session), '--world', 'planned/other.world', '--route', str(ROUTE)],
                             capture_output=True, text=True, timeout=120, stdin=subprocess.DEVNULL)

        # 단색 월드: 월드는 받아들이고 로봇 불일치(경로는 robot_2)로 시뮬레이션 전에 멈춘다.
        self.assertEqual(plain.returncode, 2)
        self.assertNotIn('지원하지 않는 --world', plain.stdout + plain.stderr)
        self.assertIn('robot_id', plain.stdout + plain.stderr)
        self.assertEqual(bad.returncode, 2)
        self.assertIn('지원하지 않는 --world', bad.stdout + bad.stderr)
        self.assertIn(PLAIN, bad.stdout + bad.stderr)


if __name__ == '__main__':
    unittest.main()
