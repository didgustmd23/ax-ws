#!/usr/bin/env python3
"""회귀 시험용 정확한 프로세스 도우미. 명령줄로 프로세스를 찾지 않는다.

사용법: test_proc.py stamp PID            -> 커널 시작 시각 출력(없으면 종료 코드 1)
        test_proc.py alive PID START      -> PID가 살아 있고 그 시작 시각일 때만 종료 코드 0
        test_proc.py kill PID START SIG   -> 시작 시각이 여전히 같을 때만 pidfd로 신호
좀비는 이미 종료됐으므로 살아 있지 않은 것으로 센다."""
import os, signal, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from own_procs import start_time


# /proc의 한 글자 프로세스 상태를 돌려준다. PID가 없으면 None.
def state(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return None
    return stat[stat.rindex(')') + 2]


# 하위 명령(stamp·alive·kill)을 실행한다.
cmd, pid = sys.argv[1], int(sys.argv[2])
if cmd == 'stamp':
    s = start_time(pid)
    print(s) if s is not None else sys.exit(1)
elif cmd == 'alive':
    sys.exit(0 if start_time(pid) == int(sys.argv[3]) and state(pid) not in (None, 'Z') else 1)
elif cmd == 'kill':
    try:
        fd = os.pidfd_open(pid)
    except OSError:
        sys.exit(0)
    try:
        if start_time(pid) == int(sys.argv[3]):
            signal.pidfd_send_signal(fd, getattr(signal, sys.argv[4]))
    except ProcessLookupError:
        pass
    finally:
        os.close(fd)
