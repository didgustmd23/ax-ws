import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('evidence_files', Path(__file__).resolve().parents[1] / 'evidence_files.py')
evidence = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evidence)


class SharedEvidence(unittest.TestCase):
    def test_identical_logs_share_a_file_but_changed_results_are_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / 'docs/evidence/a1'
            second = root / 'docs/evidence/drive_repair'
            first.mkdir(parents=True)
            second.mkdir(parents=True)
            a = evidence.write_test_log(root, first, 'one\r\nOK\r\n')
            b = evidence.write_test_log(root, second, 'one\nOK\n')
            self.assertEqual((first / a).resolve(), (second / b).resolve())
            changed = evidence.write_test_log(root, first, 'two\nOK\n')
            self.assertNotEqual(a, changed)
            self.assertEqual((first / a).read_text(), 'one\nOK\n')
            self.assertEqual(len(list((root / 'docs/evidence/shared').glob('*.txt'))), 2)

    def test_run_date_is_iso_kst_date(self):
        # 검증 자료 날짜는 고정값이 아니라 실행 시점의 KST 날짜(YYYY-MM-DD)다.
        self.assertRegex(evidence.kst_today(), r'^\d{4}-\d{2}-\d{2}$')
