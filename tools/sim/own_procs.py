#!/usr/bin/env python3
"""격리 검증 스크립트용 소유 프로세스 기록 도우미(owned_procs.sh가 사용한다).

소유 조건(다음 중 하나):
  1. own_start가 시작해 직접 등록('register')한 루트 프로세스. 그래서 환경을 비운(env -i) 루트도
     소유로 본다. bash는 종료한 자식을 곧바로 회수할 수 있으므로 원리상 등록 전에 루트의 PID가
     재사용될 수 있다. 따라서 PID는 살아 있고, 부모가 등록하는 셸이며, 이 도우미 자신이 아닐 때만
     받아들인다. 루트를 시작한 뒤 등록하기까지 셸은 이 도우미 말고는 fork하지 않으므로, 조건에
     맞으면 우리가 시작한 루트다. 이미 종료한 루트는 등록하지 않는다(자손은 2·3으로 계속 찾는다).
  2. 환경에 AX_OWN_TOKEN=<token>이 있다(세션·프로세스 그룹을 바꾸거나 init으로 부모가 바뀌어도
     모든 자손이 물려받는다).
  3. 소유 프로세스의 살아 있는 자손이다(부모 PID 기준).
기록은 (pid, start_time) 쌍이며 start_time은 /proc/<pid>/stat의 22번째 필드이므로, 재사용된
PID는 옛 기록과 맞지 않는다. 신호는 pidfd로 보낸다. 먼저 pidfd를 열고 시작 시각을 다시 확인한
뒤 pidfd_send_signal()을 쓰므로 확인 후 종료 사이의 경쟁이 없다. 시작 전부터 있던
(pid, start_time) 쌍에는 신호를 보내지 않는다.

usage: own_procs.py init STATE_DIR
       own_procs.py register STATE_DIR PID PARENT_PID  (종료 코드 2: 이미 종료, 1: 거부)
       own_procs.py snapshot STATE_DIR TOKEN
       own_procs.py signal STATE_DIR SIGNAME      (신호를 보낸 프로세스 수를 출력)
       own_procs.py alive STATE_DIR               (살아 있는 소유 프로세스 'pid start cmd'를 출력)
"""
import os
import signal
import sys
from pathlib import Path


# 커널 시작 시각(부팅 후 clock tick)을 돌려준다. 프로세스가 없으면 None.
def start_time(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return None
    return int(stat[stat.rindex(')') + 2:].split()[19])   # 전체 기준 22번째 필드


# 부모 PID를 돌려준다. 프로세스가 없으면 None.
def parent(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return None
    return int(stat[stat.rindex(')') + 2:].split()[1])


# 프로세스 환경에 AX_OWN_TOKEN=<token>이 있는지 확인한다.
def has_token(pid, token):
    try:
        return (f'AX_OWN_TOKEN={token}'.encode()
                in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0'))
    except OSError:                     # 다른 사용자의 프로세스는 읽을 수 없음 -> 소유 아님
        return False


# /proc에 있는 모든 PID 목록을 돌려준다.
def pids():
    return [int(p) for p in os.listdir('/proc') if p.isdigit()]


# 기록 파일에서 (pid, start_time) 쌍 집합을 읽는다. 파일이 없으면 빈 집합.
def load(path):
    if not path.exists():
        return set()
    return {tuple(map(int, line.split())) for line in path.read_text().split('\n') if line.strip()}


# (pid, start_time) 쌍 집합을 정렬해 기록 파일에 쓴다.
def save(path, pairs):
    path.write_text(''.join(f'{p} {s}\n' for p, s in sorted(pairs)))


# init·register·snapshot·signal·alive 하위 명령을 실행한다.
def main():
    cmd, state = sys.argv[1], Path(sys.argv[2])
    pre_f, own_f = state / 'pre.txt', state / 'owned.txt'

    # init: 시작 전부터 있던 프로세스를 기록하고 소유 기록을 비운다.
    if cmd == 'init':
        state.mkdir(parents=True, exist_ok=True)
        save(pre_f, {(p, s) for p in pids() if (s := start_time(p)) is not None})
        save(own_f, set())
        return

    pre, owned = load(pre_f), load(own_f)

    # register: own_start가 시작한 루트 프로세스를 직접 등록한다.
    if cmd == 'register':
        pid, expected_parent = int(sys.argv[3]), int(sys.argv[4])
        s = start_time(pid)
        if s is None:                       # 루트가 이미 종료(회수)됨: 소유할 것이 없음
            sys.exit(2)

        # PID 재사용 방지: 등록하는 셸의 살아 있는 직계 자식만 받는다. 이 도우미 자신과
        # 시작 전부터 있던 (pid, start_time) 쌍은 받지 않는다.
        if parent(pid) != expected_parent or pid == os.getpid() or (pid, s) in pre:
            print(f'own_procs: refused to register pid {pid}', file=sys.stderr)
            sys.exit(1)
        save(own_f, (owned | {(pid, s)}) - pre)

    # snapshot: 토큰을 가진 프로세스와 소유 프로세스의 자손을 소유 기록에 더한다.
    elif cmd == 'snapshot':
        token = sys.argv[3]
        live = {}
        for p in pids():
            s = start_time(p)
            if s is not None:
                live[p] = s

        owned |= {(p, s) for p, s in live.items() if has_token(p, token)}
        changed = True
        while changed:                  # 기록된 소유 프로세스 중 살아 있는 것의 자손
            changed = False
            alive_owned = {p for p, s in owned if live.get(p) == s}
            for p, s in live.items():
                if (p, s) not in owned and parent(p) in alive_owned:
                    owned.add((p, s))
                    changed = True
        save(own_f, owned - pre)

    # signal: 시작 시각이 같은 소유 프로세스에만 pidfd로 신호를 보내고 보낸 수를 출력한다.
    elif cmd == 'signal':
        sig = getattr(signal, sys.argv[3])
        sent = 0
        for p, s in sorted(owned - pre):
            try:
                fd = os.pidfd_open(p)
            except (ProcessLookupError, OSError):
                continue

            try:
                if start_time(p) == s:   # 이제 pidfd가 바로 이 프로세스를 고정함
                    signal.pidfd_send_signal(fd, sig)
                    sent += 1
            except ProcessLookupError:
                pass
            finally:
                os.close(fd)
        print(sent)

    # alive: 살아 있는 소유 프로세스를 'pid start cmd'로 출력한다.
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
