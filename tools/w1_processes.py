"""Find only processes tagged with the unique AX W1 Gazebo partition."""
import os
from pathlib import Path
import signal
import time


def tagged_processes(partition=None):
    result = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit() or int(path.name) == os.getpid():
            continue
        try:
            # Do not print or persist process environments.
            entries = path.joinpath('environ').read_bytes().split(b'\0')
            tag = next((entry.split(b'=', 1)[1].decode() for entry in entries
                        if entry.startswith(b'GZ_PARTITION=')), '')
            if (partition is None and tag.startswith('ax-w1-')) or tag == partition:
                stat = path.joinpath('stat').read_text().rsplit(')', 1)[1].split()
                if stat[0] != 'Z':
                    result[int(path.name)] = stat[19]  # Linux starttime, guards PID reuse.
        except (OSError, UnicodeError, IndexError):
            continue
    return result


def stop_tagged(partition=None):
    signalled = set()
    for sig, delay in ((signal.SIGTERM, 3), (signal.SIGKILL, 1)):
        owned = tagged_processes(partition)
        if not owned:
            break
        for pid, starttime in owned.items():
            try:
                stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
                if stat[19] == starttime:
                    os.kill(pid, sig)
                    signalled.add(pid)
            except (OSError, IndexError):
                pass
        until = time.monotonic() + delay
        while time.monotonic() < until and tagged_processes(partition):
            time.sleep(.1)
    return {'signalled_count': len(signalled), 'remaining_count': len(tagged_processes(partition))}


if __name__ == '__main__':
    # 인자 없이 실행하면 verify_* 도구가 만든 'ax-w1-' 접두사 partition의 프로세스만 정리한다.
    print(stop_tagged())
