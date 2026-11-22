"""내 외부 설치 환경을 명시해 상대 매핑 사전검사를 실행합니다. Gazebo를 시작하지 않습니다."""
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]
PREFIX = os.environ.get('AX_KIM_INSTALL_BASE', '')


@unittest.skipUnless(PREFIX, 'AX_KIM_INSTALL_BASE로 검증할 설치 경로를 명시하세요.')
class KimMappingExternalInstallTests(unittest.TestCase):
    """원본 스크립트·기준을 바꾸지 않고 내가 선택한 설치본으로 검사합니다."""

    def run_precheck(self, world, prefix=PREFIX):
        """robot_2 경로와 robot_1을 의도적으로 불일치시켜 시뮬레이션 전에 종료합니다."""
        return subprocess.run(
            ['bash', str(ROOT / 'src/ax_localization/scripts/run_mapping_session.sh'),
             '--install-base', str(prefix), '--world', world,
             '--route', str(ROOT / 'src/ax_localization/config/routes/ax_warehouse_center.yaml'),
             '--robot-id', 'robot_1'], cwd=ROOT, stdin=subprocess.DEVNULL,
            capture_output=True, text=True, timeout=120)

    def test_selected_install_reaches_robot_id_check(self):
        """명시한 설치본의 월드를 읽고 원래 의도한 ID 불일치 검사에 도달합니다."""
        result = self.run_precheck('planned/ax_warehouse_plain.world')
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 2, output)
        self.assertIn('robot_id', output)
        self.assertIn('경로 사전검사 실패', output)
        self.assertNotIn('선택한 설치 폴더 밖', output)

    def test_unknown_world_is_still_rejected(self):
        """설치 경로를 맞춰도 지원하지 않는 월드를 허용하지 않습니다."""
        result = self.run_precheck('planned/other.world')
        self.assertEqual(result.returncode, 2)
        self.assertIn('지원하지 않는 --world', result.stdout + result.stderr)

    def test_missing_install_does_not_fall_back_to_other_workspace(self):
        """없는 설치본을 지정했을 때 기존 셸 설치본으로 몰래 우회하지 않습니다."""
        result = self.run_precheck('planned/ax_warehouse_plain.world',
                                   Path(PREFIX) / 'not-an-install-directory')
        self.assertEqual(result.returncode, 2)
        self.assertIn('설치 환경 설정 파일이 없음', result.stdout + result.stderr)
