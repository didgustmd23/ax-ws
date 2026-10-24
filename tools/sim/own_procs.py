#!/usr/bin/env python3
"""Owned-process bookkeeping for isolated verification scripts (used by owned_procs.sh).

Ownership (any of):
  1. a root started by own_start and registered directly ('register'), so a root that cleared its
     environment (env -i) is still owned. bash may reap an exited child at once, so the root's PID
     could in principle be recycled before registration. A PID is therefore accepted only if it is
     alive, its parent is the registering shell and it is not this helper: between starting the root
     and registering it the shell forks nothing but this helper, so a match is the root we started.
     A root that has already exited is not registered (its descendants are still found by 2/3);
  2. its environment carries AX_OWN_TOKEN=<token> (inherited by every descendant even after it changes
     session/process group or is re-parented to init);
  3. it is a live descendant (by parent PID) of an owned process. Records are (pid, start_time) pairs, start_time being
field 22 of /proc/<pid>/stat, so a recycled PID never matches an old record. Signals go through
pidfd: the pidfd is opened first, the start time is re-checked, then pidfd_send_signal() is used,
which removes the check-then-kill race. Pre-existing (pid, start_time) pairs are never signalled.

usage: own_procs.py init STATE_DIR
       own_procs.py register STATE_DIR PID PARENT_PID  (exit 2: already gone, 1: refused)
       own_procs.py snapshot STATE_DIR TOKEN
       own_procs.py signal STATE_DIR SIGNAME      (prints how many processes were signalled)
       own_procs.py alive STATE_DIR               (prints live owned 'pid start cmd')
"""
import os
import signal
import sys
from pathlib import Path


def start_time(pid):
    """Kernel start time (clock ticks since boot) or None when the process is gone."""
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return None
    return int(stat[stat.rindex(')') + 2:].split()[19])   # field 22 overall


def parent(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return None
    return int(stat[stat.rindex(')') + 2:].split()[1])


def has_token(pid, token):
    try:
        return f'AX_OWN_TOKEN={token}'.encode() in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
    except OSError:                     # other users' processes are unreadable -> never owned
        return False


def pids():
    return [int(p) for p in os.listdir('/proc') if p.isdigit()]


def load(path):
    if not path.exists():
        return set()
    return {tuple(map(int, line.split())) for line in path.read_text().split('\n') if line.strip()}


def save(path, pairs):
    path.write_text(''.join(f'{p} {s}\n' for p, s in sorted(pairs)))


def main():
    cmd, state = sys.argv[1], Path(sys.argv[2])
    pre_f, own_f = state / 'pre.txt', state / 'owned.txt'
    if cmd == 'init':
        state.mkdir(parents=True, exist_ok=True)
        save(pre_f, {(p, s) for p in pids() if (s := start_time(p)) is not None})
        save(own_f, set())
        return
    pre, owned = load(pre_f), load(own_f)
    if cmd == 'register':
        pid, expected_parent = int(sys.argv[3]), int(sys.argv[4])
        s = start_time(pid)
        if s is None:                       # root already exited (and was reaped): nothing to own
            sys.exit(2)
        # PID-reuse guard: only a live direct child of the registering shell, never this helper,
        # never a pre-existing (pid, start_time) pair.
        if parent(pid) != expected_parent or pid == os.getpid() or (pid, s) in pre:
            print(f'own_procs: refused to register pid {pid}', file=sys.stderr)
            sys.exit(1)
        save(own_f, (owned | {(pid, s)}) - pre)
    elif cmd == 'snapshot':
        token = sys.argv[3]
        live = {}
        for p in pids():
            s = start_time(p)
            if s is not None:
                live[p] = s
        owned |= {(p, s) for p, s in live.items() if has_token(p, token)}
        changed = True
        while changed:                  # descendants of owned processes that are still recorded live
            changed = False
            alive_owned = {p for p, s in owned if live.get(p) == s}
            for p, s in live.items():
                if (p, s) not in owned and parent(p) in alive_owned:
                    owned.add((p, s)); changed = True
        save(own_f, owned - pre)
    elif cmd == 'signal':
        sig = getattr(signal, sys.argv[3])
        sent = 0
        for p, s in sorted(owned - pre):
            try:
                fd = os.pidfd_open(p)
            except (ProcessLookupError, OSError):
                continue
            try:
                if start_time(p) == s:   # pidfd now pins this exact process
                    signal.pidfd_send_signal(fd, sig)
                    sent += 1
            except ProcessLookupError:
                pass
            finally:
                os.close(fd)
        print(sent)
    elif cmd == 'alive':
        for p, s in sorted(owned - pre):
            if start_time(p) == s:
                try:
                    c = Path(f'/proc/{p}/cmdline').read_bytes().replace(b'\0', b' ').decode()[:100]
                except OSError:
                    c = '?'
                print(p, s, c)


if __name__ == '__main__':
    main()
