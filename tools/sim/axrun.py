"""AX 실행 도구의 공용 부분: 실행 폴더, 단계 결과표, ROS 환경, 프로세스.

단계 하나의 결과 상태:
  PASS     실행했고 통과
  FAIL     실행했고 실패
  BLOCKED  이번 실행의 선행 단계가 통과하지 않아 실행하지 않음
  NOT_RUN  실행할 수 없음(도구 없음, 시험 결과 0개 등)
  INFO     참고용이며 통과·실패 근거가 아님
실행 전체 종료 코드: 0 필수 단계 모두 PASS, 1 FAIL 있음, 2 FAIL은 없으나 필수 단계가
BLOCKED/NOT_RUN(미완료), 3 사용법 오류.
"""
import json
import os
import secrets
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]
ROS_SETUP = Path('/opt/ros/jazzy/setup.bash')
sys.path.insert(0, str(HERE))
import own_procs  # noqa: E402  (기존 도우미, 여기서는 읽기 전용으로 사용)

REQUIRED_STATES = ('PASS', 'FAIL', 'BLOCKED', 'NOT_RUN')


# 실행 폴더와 domain 할당 기록을 두는 폴더(AX_RUN_ROOT, 기본 /tmp/ax_runs)를 돌려준다.
def run_root():
    return Path(os.environ.get('AX_RUN_ROOT', '/tmp/ax_runs')).resolve()


# 시각과 난수로 실행 ID를 만들고 그 실행 폴더를 새로 만든다.
def new_run_dir(kind):
    run_id = time.strftime('%Y%m%d-%H%M%S') + '_' + secrets.token_hex(3)
    path = run_root() / f'{run_id}_{kind}'
    path.mkdir(parents=True)
    return run_id, path


# JSON을 임시 파일에 쓴 뒤 이름을 바꿔, 읽는 쪽이 반쯤 쓴 기록을 보지 않게 한다.
def write_json(path, data):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')
    os.replace(tmp, path)


# JSON 파일을 읽는다. 없거나 깨졌으면 None을 돌려준다.
def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


# 로그에서 warning을 언급한 줄 수를 센다. 짧은 요약에 경고가 묻히지 않게 보고한다.
def warning_count(log):
    try:
        return sum('warn' in line.lower()
                   for line in Path(log).read_text(errors='replace').splitlines())
    except OSError:
        return 0


# 단계마다 summary.tsv로 다시 쓰는 단계 결과표. 전체 로그는 실행 폴더에 따로 남는다.
class Summary:
    # 결과표 경로를 정하고 빈 표를 바로 쓴다.
    def __init__(self, run_dir, name='summary.tsv'):
        self.path = Path(run_dir) / name
        self.rows = []
        self.write()

    # 단계 결과 한 줄(상태·종료 코드·시간·경고 수·로그·메모)을 더하고 표를 쓴 뒤 화면에 출력한다.
    def add(self, step, status, rc='', sec='', log='', note=''):
        warn = warning_count(log) if log else ''
        self.rows.append(dict(step=step, status=status, exit=rc, sec=sec, warnings=warn,
                              log=str(log), note=note))
        self.write()
        print(f'{status:8} {step}' + (f'  ({note})' if note else ''), flush=True)
        return status

    # 지정한 단계의 상태를 돌려준다(없으면 None).
    def status(self, step):
        return next((r['status'] for r in self.rows if r['step'] == step), None)

    # 현재 결과표를 탭 구분 파일로 쓴다.
    def write(self):
        head = 'step\tstatus\texit\tsec\twarnings\tlog\tnote\n'
        body = ''.join('\t'.join(str(r[k]).replace('\t', ' ').replace('\n', ' ')
                                 for k in ('step', 'status', 'exit', 'sec', 'warnings', 'log',
                                           'note'))
                       + '\n' for r in self.rows)
        self.path.write_text(head + body)

    # 필수 상태로 실행 종료 코드를 정한다. FAIL이면 1, BLOCKED·NOT_RUN만 있거나 비었으면 2, 아니면 0.
    def exit_code(self):
        states = [r['status'] for r in self.rows if r['status'] in REQUIRED_STATES]
        if 'FAIL' in states:
            return 1
        if 'BLOCKED' in states or 'NOT_RUN' in states or not states:
            return 2
        return 0


# 명령을 실행해 stdout·stderr 전체를 로그 파일에 남기고 (종료 코드, 걸린 초)를 돌려준다.
# 시간 초과는 124, 실행 파일 없음은 127이다.
def run_step(cmd, log, env=None, cwd=WS, timeout=None):
    start = time.monotonic()
    with open(log, 'w') as out:
        out.write('$ ' + ' '.join(map(str, cmd)) + '\n')
        out.flush()
        try:
            rc = subprocess.run(list(map(str, cmd)), stdout=out, stderr=subprocess.STDOUT,
                                env=env, cwd=cwd, timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            out.write(f'\n[timeout after {timeout} s]\n')
            rc = 124
        except FileNotFoundError as error:
            out.write(f'\n[{error}]\n')
            rc = 127
    return rc, round(time.monotonic() - start, 1)


# 현재 환경에서 이 작업공간이나 이전 실행 폴더를 가리키는 경로 항목을 뺀다. 이전에 source한
# install/이 새 빌드나 검사에 섞이지 않게 한다.
def clean_env():
    env = {}
    stale = (str(WS), str(run_root()))
    for key, value in os.environ.items():
        if any(s in value for s in stale) and (os.pathsep in value or key.endswith('PATH')):
            kept = [e for e in value.split(os.pathsep)
                    if e and not any(e.startswith(s) for s in stale)]
            if kept:
                env[key] = os.pathsep.join(kept)
        else:
            env[key] = value
    return env


# ROS Jazzy와(지정하면) 이 작업공간 install 폴더를 source한 뒤의 환경을 만든다.
def ros_env(install_base=None, extra=None):
    if not ROS_SETUP.exists():
        raise RuntimeError(f'{ROS_SETUP} not found')
    script = f'source "{ROS_SETUP}"'
    if install_base is not None:
        setup = Path(install_base) / 'setup.bash'
        if not setup.exists():
            raise RuntimeError(f'{setup} not found')
        script += f' && source "{setup}"'

    base = clean_env()
    base.update(extra or {})
    out = subprocess.run(['bash', '-c', script + ' && env -0'], env=base, check=True,
                         capture_output=True).stdout
    env = dict(item.split('=', 1) for item in out.decode().split('\0') if '=' in item)
    env.update(extra or {})
    return env


# 프로세스 ----------------------------------------------------------------------------------

# /proc의 한 글자 프로세스 상태(R, S, D, Z 등)를 돌려준다. PID가 없으면 None.
def proc_state(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return None
    return stat[stat.rindex(')') + 2]


# 정확히 그 (PID, 시작 시각)인 프로세스가 살아 있고 종료 후 회수 대기(좀비)가 아닐 때만 참이다.
def is_running(pid, start):
    return own_procs.start_time(pid) == start and proc_state(pid) not in (None, 'Z')


# 현재 프로세스의 (PID, 시작 시각)을 돌려준다.
def self_identity():
    pid = os.getpid()
    return [pid, own_procs.start_time(pid)]


# 한 실행의 소유 프로세스를 읽기 전용으로 찾아 (실행 중, 좀비) 목록을 (PID, 시작 시각, 명령)으로
# 돌려준다. own_procs.py 기록과, 토큰을 주면 그 토큰을 가진 프로세스를 쓴다. own_procs.py의
# `alive`는 시작 시각만 보므로 좀비도 나열하는데, 여기서는 좀비를 따로 나누고 실행 중으로 세지 않는다.
def owned_processes(state_dir, token=None):
    state_dir = Path(state_dir)
    pre = own_procs.load(state_dir / 'pre.txt')
    pairs = own_procs.load(state_dir / 'owned.txt') - pre
    if token:
        for pid in own_procs.pids():
            start = own_procs.start_time(pid)
            if start is not None and (pid, start) not in pre and own_procs.has_token(pid, token):
                pairs.add((pid, start))

    running, zombies = [], []
    for pid, start in sorted(pairs):
        if own_procs.start_time(pid) != start:
            continue
        try:
            cmd = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()[:120]
        except OSError:
            cmd = '?'
        (zombies if proc_state(pid) == 'Z' else running).append((pid, start, cmd.strip()))
    return running, zombies


# 현재 HEAD·브랜치·변경 경로 수를 기록용으로 읽는다(실패하면 None).
def git_state():
    # git 명령을 실행해 성공하면 출력, 실패하면 None을 돌려준다.
    def git(*args):
        result = subprocess.run(['git', *args], cwd=WS, capture_output=True, text=True)
        return result.stdout.strip() if result.returncode == 0 else None

    status = git('status', '--porcelain=v1', '--untracked-files=all')
    return {'head': git('rev-parse', 'HEAD'), 'branch': git('branch', '--show-current'),
            'changed_paths': None if status is None else len(status.splitlines())}
