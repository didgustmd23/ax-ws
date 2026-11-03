"""Store identical unit-test logs once without overwriting older evidence."""
import hashlib
import os
from pathlib import Path


def write_test_log(workspace, destination, text):
    data = text.replace('\r\n', '\n').encode('utf-8')
    shared = Path(workspace) / 'docs/evidence/shared'
    shared.mkdir(parents=True, exist_ok=True)
    path = shared / ('unit_tests-' + hashlib.sha256(data).hexdigest()[:16] + '.txt')
    if path.exists() and path.read_bytes() != data:
        raise ValueError('Evidence hash collision')
    path.write_bytes(data)
    return Path(os.path.relpath(path, destination)).as_posix()
