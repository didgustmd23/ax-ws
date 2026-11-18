"""원본 A4 정지 시험의 경로만 외부 설치본에 연결합니다. 상대 파일·판정은 수정하지 않습니다."""
import argparse
import importlib.util
from pathlib import Path
import tempfile
import unittest


def main():
    """별도 임시 install 링크로 원본 시험을 실행하고 실패를 그대로 반환합니다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-prefix', type=Path, required=True)
    args = parser.parse_args()
    prefix = args.install_prefix.resolve()
    if not (prefix / 'ax_safety/lib/ax_safety/a4_safety_gate').is_file():
        parser.error('install-prefix에 현재 a4_safety_gate 실행 파일이 필요합니다.')
    source = Path(__file__).with_name('test_a4_stop_confirmation_ros.py')
    spec = importlib.util.spec_from_file_location('external_a4_stop_review', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory(prefix='ax_external_a4_review_') as folder:
        root = Path(folder)
        (root / 'install').symlink_to(prefix, target_is_directory=True)
        # 원본은 ROOT/install에서 바이너리를 찾는다. 입력·기준값·assert·domain 예약은 원본 그대로다.
        module.ROOT = root
        print(f'원본 시험: {source}\n외부 설치본: {prefix}', flush=True)
        result = unittest.TextTestRunner(verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromModule(module))
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
