#!/usr/bin/env python3
"""Exact-process helper for the regression test: never searches by command line.

usage: test_proc.py stamp PID            -> prints the kernel start time (exit 1 if gone)
       test_proc.py alive PID START      -> exit 0 only if PID is alive AND has that start time
       test_proc.py kill PID START SIG   -> pidfd signal, only if the start time still matches
A zombie counts as not alive (it has already terminated)."""
import os, signal, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from own_procs import start_time


def state(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return None
    return stat[stat.rindex(')') + 2]


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
