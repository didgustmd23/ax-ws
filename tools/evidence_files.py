"""Store identical unit-test logs once without overwriting older evidence."""
from datetime import datetime, timedelta, timezone
import hashlib
import os
from pathlib import Path
import subprocess
import sys


# 검증 자료에 기록할 실행 날짜(KST, YYYY-MM-DD)를 돌려준다.
def kst_today():
    return datetime.now(timezone(timedelta(hours=9))).date().isoformat()


# 수집 시점의 코드로 tools/tests 단위 테스트를 실행하고 (통과 여부, 출력)을 돌려준다.
def run_unit_tests(workspace):
    result = subprocess.run(
        [sys.executable, '-m', 'unittest', 'discover', '-s', 'tools/tests', '-v'],
        cwd=workspace, capture_output=True, text=True)

    return result.returncode == 0, result.stdout + result.stderr


# 같은 단위 테스트 로그는 한 파일로 공유하고, 다른 결과는 덮어쓰지 않고 새 파일로 남긴다.
def write_test_log(workspace, destination, text):
    data = text.replace('\r\n', '\n').encode('utf-8')
    shared = Path(workspace) / 'docs/evidence/shared'
    shared.mkdir(parents=True, exist_ok=True)
    path = shared / ('unit_tests-' + hashlib.sha256(data).hexdigest()[:16] + '.txt')
    if path.exists() and path.read_bytes() != data:
        raise ValueError('Evidence hash collision')
    path.write_bytes(data)
    return Path(os.path.relpath(path, destination)).as_posix()
