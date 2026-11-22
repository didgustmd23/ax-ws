"""내 실행 도구가 단계 실패를 숨기거나 기존 근거를 덮지 않는지 검사합니다."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'kim_review_runner_under_test', Path(__file__).with_name('run_kim_team_review.py'))
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


class KimReviewRunnerTests(unittest.TestCase):
    """명령 실행만 대체해 실패 전달·근거 보존을 확인합니다. 실제 ROS 검사는 아닙니다."""

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        root = Path(self.folder.name)
        self.prefix, self.c2 = root / 'main install', root / 'c2 install'
        for path in (self.prefix, self.c2):
            path.mkdir()
            (path / 'setup.bash').write_text('# fixture', encoding='utf-8')
        gate = self.prefix / 'ax_safety/lib/ax_safety/a4_safety_gate'
        gate.parent.mkdir(parents=True)
        gate.write_text('fixture, never executed', encoding='utf-8')
        self.output = root / 'results'
        self.arguments = [str(runner.__file__), '--install-prefix', str(self.prefix),
                          '--c2-prefix', str(self.c2), '--output', str(self.output)]

    def test_failed_c2_stage_cannot_be_reported_as_overall_success(self):
        """앞뒤 단계가 통과해도 C2 실패를 보존하고 자동 재시도하지 않습니다."""
        outcomes = [SimpleNamespace(returncode=code) for code in (0, 1, 0)]
        with patch.object(sys, 'argv', self.arguments), \
                patch.object(runner.subprocess, 'run', side_effect=outcomes) as execute:
            self.assertEqual(runner.main(), 1)
        summary = json.loads((self.output / 'summary.json').read_text(encoding='utf-8'))
        self.assertFalse(summary['all_stages_passed'])
        self.assertEqual([row['returncode'] for row in summary['stages']], [0, 1, 0])
        self.assertEqual(execute.call_count, 3)

    def test_existing_evidence_is_preserved_without_running_any_stage(self):
        """진행 중이거나 이전 실행의 로그가 있으면 시작 전에 거절합니다."""
        self.output.mkdir()
        evidence = self.output / 'core.log'
        evidence.write_text('original failure evidence', encoding='utf-8')
        with patch.object(sys, 'argv', self.arguments), \
                patch.object(runner.subprocess, 'run') as execute:
            with self.assertRaises(SystemExit) as result:
                runner.main()
        self.assertEqual(result.exception.code, 2)
        execute.assert_not_called()
        self.assertEqual(evidence.read_text(encoding='utf-8'), 'original failure evidence')
