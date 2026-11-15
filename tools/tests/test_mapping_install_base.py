"""별도 install base 선택을 가짜 설치본으로 검사한다. ROS 노드와 Gazebo는 실행하지 않는다."""

import hashlib
import json
import os
import shlex
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'src/ax_localization/scripts/run_mapping_session.sh'
WORLD = 'planned/ax_warehouse.world'


@unittest.skipUnless(Path('/opt/ros/jazzy/setup.bash').is_file(), 'ROS Jazzy setup 필요')
class MappingInstallBase(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='ax_mapping_install_')
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.workspace = self.folder / 'workspace with spaces'
        self.scripts = self.workspace / 'src/ax_localization/scripts'
        self.scripts.mkdir(parents=True)
        self.script = self.scripts / SCRIPT.name
        shutil.copyfile(SCRIPT, self.script)
        catalog = self.workspace / 'src/ax_description/config/worlds.yaml'
        catalog.parent.mkdir(parents=True)
        catalog.write_text(f'worlds:\n  {WORLD}: {{}}\n')
        self.route = self.folder / 'route.yaml'
        self.route.write_text('test route\n')
        # route 사전검사에서 종료해 실제 세션/프로세스가 생성되지 않게 한다.
        # AX_INSTALL_TEST_CHECK_RC로 사전검사 통과를 흉내 내 그 뒤 단계까지 시험할 수 있다(기본 2: 여기서 종료).
        (self.scripts / 'mapping_drive.py').write_text(
            'import os,sys\n'
            'print("선택된 설치본: " + os.environ["AX_INSTALL_TEST_SELECTED"])\n'
            'sys.exit(int(os.environ.get("AX_INSTALL_TEST_CHECK_RC", "2")))\n')
        self.modules = self.folder / 'modules'
        package = self.modules / 'ament_index_python'
        package.mkdir(parents=True)
        (package / '__init__.py').write_text('')
        (package / 'packages.py').write_text(
            'import os\n'
            'def get_package_share_directory(name, print_warning=True):\n'
            '    return os.environ.get("AX_INSTALL_TEST_SHARE_" + name, os.environ["AX_INSTALL_TEST_SHARE"])\n'
            'def get_package_prefix(name):\n'
            '    return os.environ["AX_INSTALL_TEST_PREFIX"]\n')
        self.install(self.workspace / 'install', 'default')

    # 가짜 설치본을 만든다. extra는 setup.bash 끝에 붙일 줄이다.
    def install(self, base, label, merged=False, extra=''):
        base.mkdir(parents=True)
        share = base / ('share/ax_description' if merged else 'ax_description/share/ax_description')
        world = share / 'worlds' / WORLD
        world.parent.mkdir(parents=True)
        world.write_text(label)
        (base / 'setup.bash').write_text(
            f'export AX_INSTALL_TEST_SELECTED={shlex.quote(label)}\n'
            f'export AX_INSTALL_TEST_SHARE={shlex.quote(str(share))}\n'
            f'export AX_INSTALL_TEST_PREFIX={shlex.quote(str(base))}\n'
            f'export PYTHONPATH={shlex.quote(str(self.modules))}:"${{PYTHONPATH:-}}"\n' + extra)
        return world

    def run_script(self, extra=(), cwd=None):
        result = subprocess.run(
            ['bash', str(self.script), '--world', WORLD, '--route', str(self.route), *extra],
            cwd=cwd or self.folder, env=dict(os.environ), capture_output=True, text=True, timeout=30)
        self.assertFalse((self.workspace / 'log').exists(), '사전 검사에서 세션 생성 금지')
        return result, result.stdout + result.stderr

    def test_default_install_unchanged(self):
        result, output = self.run_script()
        self.assertEqual(result.returncode, 2, output)
        self.assertIn('선택된 설치본: default', output)

    def test_external_isolated_and_merged_with_spaces(self):
        for merged in (False, True):
            with self.subTest(merged=merged):
                base = self.folder / f'external install {merged}'
                self.install(base, f'external-{merged}', merged)
                result, output = self.run_script(['--install-base', str(base)])
                self.assertEqual(result.returncode, 2, output)
                self.assertIn(f'선택된 설치본: external-{merged}', output)

    def test_relative_install_base_uses_invocation_directory(self):
        base = self.folder / 'external install'
        self.install(base, 'relative')
        caller = self.folder / 'caller'
        caller.mkdir()
        result, output = self.run_script(['--install-base', '../external install'], cwd=caller)
        self.assertEqual(result.returncode, 2, output)
        self.assertIn('선택된 설치본: relative', output)

    def test_missing_install_base_value(self):
        for args in (['--install-base'], ['--install-base', '--gui']):
            with self.subTest(args=args):
                result, output = self.run_script(args)
                self.assertEqual(result.returncode, 2, output)
                self.assertIn('--install-base에 설치 폴더가 필요', output)

    def test_missing_setup_fails_without_default_fallback(self):
        base = self.folder / 'missing setup'
        base.mkdir()
        result, output = self.run_script(['--install-base', str(base)])
        self.assertEqual(result.returncode, 2, output)
        self.assertIn('설치 환경 설정 파일이 없음', output)
        self.assertNotIn('선택된 설치본: default', output)

    def test_failed_setup_stops_before_route_check(self):
        base = self.folder / 'failed setup'
        base.mkdir()
        (base / 'setup.bash').write_text('return 1\n')
        result, output = self.run_script(['--install-base', str(base)])
        self.assertEqual(result.returncode, 2, output)
        self.assertIn('설치 환경 설정 실패', output)
        self.assertNotIn('선택된 설치본:', output)

    def test_missing_external_world_does_not_use_default_world(self):
        base = self.folder / 'without world'
        world = self.install(base, 'missing-world')
        world.unlink()
        result, output = self.run_script(['--install-base', str(base)])
        self.assertEqual(result.returncode, 2, output)
        self.assertIn('설치된 월드 파일이 없음', output)
        self.assertNotIn('선택된 설치본:', output)

    def test_real_ament_index_selects_external_package(self):
        # 실제 ament 인덱스에서 별도 prefix가 선택되는지도 확인한다(노드 실행 없음).
        for merged in (False, True):
            with self.subTest(merged=merged):
                base = self.folder / f'real index {merged}'
                world = self.install(base, 'unused', merged)
                world.unlink()
                prefix = base if merged else base / 'ax_description'
                marker = prefix / 'share/ament_index/resource_index/packages/ax_description'
                marker.parent.mkdir(parents=True)
                marker.touch()
                (base / 'setup.bash').write_text(
                    f'export AMENT_PREFIX_PATH={shlex.quote(str(prefix))}:"${{AMENT_PREFIX_PATH:-}}"\n')
                result, output = self.run_script(['--install-base', str(base)])
                self.assertEqual(result.returncode, 2, output)
                self.assertIn(f'설치된 월드 파일이 없음(ax_description 빌드 필요): {world}', output)

    # 앞서 source된 다른 설치본의 패키지로 넘어가지 않는지 확인한다
    def test_package_from_previously_sourced_install_is_rejected(self):
        # 호출 셸에 다른 설치본이 source돼 있고 선택한 설치본에 ax_description이 없으면 그쪽으로 넘어가지 않는다.
        other = self.folder / 'other install' / 'ax_description'
        marker = other / 'share/ament_index/resource_index/packages/ax_description'
        marker.parent.mkdir(parents=True)
        marker.touch()
        (other / 'share/ax_description/worlds' / WORLD).parent.mkdir(parents=True)
        (other / 'share/ax_description/worlds' / WORLD).write_text('other')
        base = self.folder / 'empty install'
        base.mkdir()
        (base / 'setup.bash').write_text('true\n')
        env = dict(os.environ, AMENT_PREFIX_PATH=str(other))
        result = subprocess.run(
            ['bash', str(self.script), '--world', WORLD, '--route', str(self.route), '--install-base', str(base)],
            cwd=self.folder, env=env, capture_output=True, text=True, timeout=30)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 2, output)
        self.assertIn('선택한 설치 폴더 밖의 ax_description', output)
        self.assertFalse((self.workspace / 'log').exists())

    # launch가 쓸 패키지가 선택한 설치본 밖이면 세션 전에 거절하는지 확인한다
    def test_session_package_outside_install_is_rejected(self):
        # 월드는 선택한 설치본에 있어도 launch가 쓸 다른 패키지가 밖에서 오면 세션을 만들지 않는다.
        outside = self.folder / 'outside' / 'ax_control'
        outside.mkdir(parents=True)
        base = self.folder / 'partial install'
        self.install(base, 'partial', extra=f'export AX_INSTALL_TEST_SHARE_ax_control={shlex.quote(str(outside))}\n')
        env = dict(os.environ, AX_INSTALL_TEST_CHECK_RC='0')
        result = subprocess.run(
            ['bash', str(self.script), '--world', WORLD, '--route', str(self.route), '--install-base', str(base)],
            cwd=self.folder, env=env, capture_output=True, text=True, timeout=30)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 2, output)
        self.assertIn('선택한 설치 폴더 밖의 ax_control', output)
        self.assertFalse((self.workspace / 'log').exists())

    # session.json에 외부 설치 위치·실제 월드·설정·병합 노드 출처가 남는지 확인한다
    def test_session_records_external_install_and_world(self):
        # 사전검사 뒤 세션 기록까지 진행하고, 첫 ROS 명령(가짜 ros2)에서 세션 셸을 끝낸다. 실제 노드·Gazebo 없음.
        session_procs = SCRIPT.parent / 'session_procs.sh'
        shutil.copyfile(session_procs, self.scripts / session_procs.name)
        fake_bin = self.folder / 'fake bin'
        fake_bin.mkdir()
        ros2_log = self.folder / 'ros2_calls.txt'
        # 세션 셸에 TERM을 한 번만 보낸다(정리 중 두 번째 신호로 정리가 끊기지 않게 mkdir로 1회 보장).
        once = self.folder / 'term_sent'
        (fake_bin / 'ros2').write_text(
            '#!/bin/bash\n'
            f'echo "$*" >> {shlex.quote(str(ros2_log))}\n'
            f'mkdir {shlex.quote(str(once))} 2>/dev/null && kill -TERM "${{AX_SESSION_OWNER##*-}}"\n'
            'exit 1\n')
        (fake_bin / 'ros2').chmod(0o755)
        base = self.folder / 'external install'
        world = self.install(base, 'external-session', merged=True,
                             extra=f'export PATH={shlex.quote(str(fake_bin))}:"$PATH"\n')
        share = world.parents[len(Path(WORLD).parts)]
        configs = share / 'config'
        configs.mkdir()

        for name in ('slam_mapping.yaml', 'ax2_controllers.yaml', 'common_controllers.yaml',
                     'sensor_mounts.yaml', 'a1_validation.yaml'):
            (configs / name).write_text(f'{name}: external\n')

        merge = base / 'lib/ax_localization/scan_merge.py'
        merge.parent.mkdir(parents=True)
        merge.write_text('# external scan_merge\n')
        env = dict(os.environ, AX_INSTALL_TEST_CHECK_RC='0')
        result = subprocess.run(
            ['bash', str(self.script), '--world', WORLD, '--route', str(self.route),
             '--install-base', 'external install', '--domain', '99'],
            cwd=self.folder, env=env, capture_output=True, text=True, timeout=60)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 130, output)
        self.assertIn('launch ax_control simulation.launch.py', ros2_log.read_text())

        sessions = list((self.workspace / 'log/b2_mapping').glob('*/session'))
        self.assertEqual(len(sessions), 1, output)
        session = sessions[0]
        record = json.loads((session / 'session.json').read_text(encoding='utf-8'))
        self.assertEqual(record['install_base'], str(base))
        self.assertEqual(record['world_file'], str(world))
        self.assertEqual(record['world_sha256'], hashlib.sha256(world.read_bytes()).hexdigest())
        self.assertEqual((session / 'world.world').read_text(), 'external-session')
        self.assertEqual(record['scan_merge']['file'], str(merge))

        for entry in record['config_files']['files'].values():
            self.assertTrue(entry['source'].startswith(str(share) + '/'), entry)

        self.assertTrue(json.loads((session / 'cleanup.json').read_text())['clean'])


if __name__ == '__main__':
    unittest.main()
