# 내용이 같은 검사 로그를 공용 증거 파일 하나로 저장한다. 내용이 바뀌면 새 파일을 만들어 과거 결과를 보존한다.
"""Store identical unit-test logs once without overwriting older evidence."""
import hashlib
import os
from pathlib import Path


# 줄바꿈을 LF로 정규화하고 내용 해시로 공용 파일을 선택한다. 보고서 위치 기준 상대 링크를 반환한다.
def write_test_log(workspace, destination, text):
    data = text.replace('\r\n', '\n').encode('utf-8')
    shared = Path(workspace) / 'docs/evidence/shared'
    shared.mkdir(parents=True, exist_ok=True)
    path = shared / ('unit_tests-' + hashlib.sha256(data).hexdigest()[:16] + '.txt')
    if path.exists() and path.read_bytes() != data:
        raise ValueError('Evidence hash collision')
    path.write_bytes(data)
    return Path(os.path.relpath(path, destination)).as_posix()
