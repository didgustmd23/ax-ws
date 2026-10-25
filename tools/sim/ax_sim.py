#!/usr/bin/env python3
"""AX 시뮬레이션을 격리 headless Gazebo로 실행하고 소유 프로세스만 정리한다.

  ax_sim.py start  [--model ax1|ax2|both] [--domain N] [--install-base DIR]   실행 폴더를 출력
  ax_sim.py status RUN_DIR
  ax_sim.py probe  RUN_DIR [--timeout S]      실행 중인 시뮬레이션의 기본 통신 확인
  ax_sim.py stop   RUN_DIR                    이 실행의 프로세스만 정지, 반복해도 안전
  ax_sim.py smoke  [--model ...] [--timeout S]   start + probe + stop("기본 통신 smoke")

격리: 실행마다 별도 GZ_PARTITION과 ROS_DOMAIN_ID(기본 60~99, ROS 2가 Linux 기본 설정에 권장하는
0~101 범위 안)를 받아 $AX_RUN_ROOT/alloc에 기록한다. 중복은 같은 할당 폴더를 쓰는 실행끼리만
막는다. 이 도구 없이(또는 다른 AX_RUN_ROOT로) 시작한 프로그램과는 격리되지 않는다.

smoke는 메시지 수신(/clock, 로봇별 joint_states·odom)만 확인한다. 제어기 활성, 명령 추종, 정지는
검증하지 않는다.
"""
import argparse
import fcntl
import json
import subprocess
import sys
import time
import uuid
from pathlib import Path

import axrun
from axrun import HERE, Summary, write_json, read_json

DOMAINS = range(60, 100)
# simulation.launch.py의 namespace 규칙(로봇 반복문, `legacy = is_arm and selection == 'ax1'`):
# ax1 단독은 기존 namespace 없는 토픽, 그 밖에는 /ax/<robot_id>다. launch와 함께 갱신한다.
ROBOTS = {
    'ax1': [('robot_1', '')],
    'ax2': [('robot_2', '/ax/robot_2')],
    'both': [('robot_1', '/ax/robot_1'), ('robot_2', '/ax/robot_2')],
}
STOP_STEPS = (('SIGINT', 8.0), ('SIGTERM', 4.0), ('SIGKILL', 2.0))


# domain 할당 -------------------------------------------------------------------------------

# 할당 폴더 전체에 대한 배타 잠금. 할당·회수·해제는 모두 이 잠금 아래에서 한다.
class AllocLock:
    # 잠금 대상인 할당 폴더 경로를 정한다.
    def __init__(self):
        self.folder = axrun.run_root() / 'alloc'

    # 할당 폴더를 만들고 잠금 파일에 배타 잠금을 건다.
    def __enter__(self):
        self.folder.mkdir(parents=True, exist_ok=True)
        self.handle = open(self.folder / '.lock', 'w')
        fcntl.flock(self.handle, fcntl.LOCK_EX)
        return self.folder

    # 잠금을 풀고 잠금 파일을 닫는다.
    def __exit__(self, *exc):
        fcntl.flock(self.handle, fcntl.LOCK_UN)
        self.handle.close()


# domain 기록을 회수해도 되는 이유를 돌려주고, 아직 쓰일 수 있으면 None을 돌려준다.
# STARTING 기록은 시작 담당 프로세스(PID + 시작 시각)가 살아 있는 동안 회수하지 않는다. 소유
# 프로세스가 아직 기록되지 않았을 수 있기 때문이다. 그 밖에는 실행 중인 소유 프로세스가 없을
# 때만(좀비 제외) 회수할 수 있다.
def reclaim_reason(record):
    if not isinstance(record, dict):
        return None                                  # 읽을 수 없는 기록은 유지(보수적)
    if record.get('state') == 'STARTING':
        pid, start = record.get('starter') or (None, None)
        if pid and axrun.is_running(pid, start):
            return None

    run_dir = Path(record.get('run_dir', ''))
    token = (run_dir / 'token').read_text().strip() if (run_dir / 'token').exists() else None
    if (run_dir / 'own').is_dir():
        running, _ = axrun.owned_processes(run_dir / 'own', token)
        if running:
            return None

    return f"{record.get('state')} run {record.get('run_id')} has no running process"


# 잠금 아래에서 요청한 domain(없으면 60~99 중 빈 것)을 STARTING 상태로 기록하고 돌려준다.
# 쓸 수 있는 domain이 없으면 None이다.
def allocate(run_id, run_dir, requested=None):
    with AllocLock() as folder:
        for domain in ([requested] if requested is not None else DOMAINS):
            path = folder / f'domain_{domain}.json'
            if path.exists():
                reason = reclaim_reason(read_json(path))
                if reason is None:
                    continue
                with open(folder / 'reclaimed.log', 'a') as log:
                    log.write(f'{time.strftime("%F %T")} domain {domain} '
                              f'reclaimed for {run_id}: {reason}\n')

            write_json(path, {'domain': domain, 'run_id': run_id, 'run_dir': str(run_dir),
                              'state': 'STARTING', 'starter': axrun.self_identity(),
                              'since': time.strftime('%F %T')})
            return domain
    return None


# 이 실행의 domain 기록 상태를 바꾸거나 해제한다. 다른 실행이 회수한 기록은 건드리지 않는다.
def set_record(domain, run_id, state=None, release=False):
    with AllocLock() as folder:
        path = folder / f'domain_{domain}.json'
        record = read_json(path)
        if not record or record.get('run_id') != run_id:
            return                                    # 다른 실행이 회수함: 건드리지 않음
        if release:
            path.unlink()
        else:
            record['state'] = state
            write_json(path, record)


# 실행 수명 주기 ---------------------------------------------------------------------------

# 실행 폴더의 run.json을 읽는다. 없으면 오류로 종료한다.
def load_run(run_dir):
    run_dir = Path(run_dir).resolve()
    run = read_json(run_dir / 'run.json')
    if not run:
        sys.exit(f'{run_dir}: run.json not found (not an ax_sim run folder)')
    return run_dir, run


# domain·partition을 할당하고 소유 프로세스 도구로 headless 시뮬레이션을 시작한다. 실패해도
# run.json에 상태를 남기고 실행 폴더를 돌려준다.
def start(model, domain=None, install_base=None, run=None):
    install = Path(install_base or axrun.WS / 'install').resolve()
    run_id, run_dir = run or axrun.new_run_dir(f'sim-{model}')
    info = {'run_id': run_id, 'model': model, 'install_base': str(install), 'status': 'STARTING',
            'robots': ROBOTS[model], 'git': axrun.git_state(), 'started': time.strftime('%F %T')}
    write_json(run_dir / 'run.json', info)

    chosen = allocate(run_id, run_dir, domain)
    if chosen is None:
        info.update(status='START_FAILED', error=f'no free ROS_DOMAIN_ID ({domain or "60-99"})')
        write_json(run_dir / 'run.json', info)
        return run_dir

    info.update(domain=chosen, partition=f'ax_{run_id.replace("-", "_")}')
    write_json(run_dir / 'run.json', info)
    try:
        env = axrun.ros_env(install, {'ROS_DOMAIN_ID': str(chosen),
                                      'GZ_PARTITION': info['partition']})
    except (RuntimeError, subprocess.CalledProcessError) as error:
        info.update(status='START_FAILED', error=str(error))
        write_json(run_dir / 'run.json', info)
        set_record(chosen, run_id, release=True)
        return run_dir

    # 이 domain의 다른 참여자를 가능한 범위에서 찾는다(참고용).
    try:
        graph = subprocess.run([sys.executable, HERE / 'comm_probe.py', '--graph-only'], env=env,
                               capture_output=True, text=True, timeout=30)
        info['other_nodes_before_start'] = json.loads(graph.stdout.splitlines()[-1])['other_nodes']
    except (subprocess.TimeoutExpired, ValueError, IndexError, KeyError):
        info['other_nodes_before_start'] = 'graph probe failed'

    # 기존 소유 프로세스 도우미를 그대로 재사용한다. 실행 폴더에 상태를 초기화한 뒤, 이 상태와
    # 토큰으로 owned_procs.sh를 source한 셸에서 own_start로 launch를 시작한다.
    token = uuid.uuid4().hex
    (run_dir / 'token').write_text(token + '\n')
    subprocess.run([sys.executable, HERE / 'own_procs.py', 'init', run_dir / 'own'], check=True)
    launch = ['ros2', 'launch', 'ax_control', 'simulation.launch.py', f'robot_model:={model}',
              'gui:=false']
    shell = ('source "$1"; OWN_STATE="$2"; OWN_TOKEN="$3"; shift 3; own_start "$@"; '
             'echo "$OWN_LAST_PID"')
    out = subprocess.run(['bash', '-c', shell, '_', HERE / 'owned_procs.sh', run_dir / 'own', token,
                          run_dir / 'launch.log', *launch], env=env, capture_output=True, text=True)

    # launch PID를 기록하고 잠시 기다린 뒤 소유 프로세스를 갱신해 생존 여부로 상태를 정한다.
    pid = int(out.stdout.split()[-1]) if out.stdout.split() else None
    info['launch'] = [pid, axrun.own_procs.start_time(pid) if pid else None]
    time.sleep(3)
    subprocess.run([sys.executable, HERE / 'own_procs.py', 'snapshot', run_dir / 'own', token])
    if pid and axrun.is_running(*info['launch']):
        info['status'] = 'RUNNING'
        write_json(run_dir / 'run.json', info)
        set_record(chosen, run_id, 'RUNNING')
    else:
        info.update(status='START_FAILED', error='launch exited early; see launch.log')
        write_json(run_dir / 'run.json', info)
        stop(run_dir)
    return run_dir


# 이 실행의 소유 프로세스만 SIGINT → SIGTERM → SIGKILL 순으로 종료한다.
# 0 STOPPED, 1 STOP_FAILED, 2 소유 기록 없음을 돌려준다.
def stop(run_dir):
    run_dir, run = load_run(run_dir)
    state, token_file = run_dir / 'own', run_dir / 'token'
    if not state.is_dir() or not token_file.exists():
        print('no ownership record in this run folder; nothing was signalled')
        if 'domain' in run and not state.is_dir():
            set_record(run['domain'], run['run_id'], release=True)
        return 2

    token = token_file.read_text().strip()
    helper = [sys.executable, HERE / 'own_procs.py']
    previous = read_json(run_dir / 'stop.json')

    # 저장한 상태와 토큰을 되살려 쓰며 own_init은 다시 호출하지 않는다.
    result = {'signalled': {}, 'started': time.strftime('%F %T')}
    for sig, wait in STOP_STEPS:
        subprocess.run(helper + ['snapshot', state, token])
        running, _ = axrun.owned_processes(state, token)
        if not running:
            break
        out = subprocess.run(helper + ['signal', state, sig], capture_output=True, text=True).stdout
        result['signalled'][sig] = int(out.strip() or 0)
        end = time.monotonic() + wait
        while time.monotonic() < end and axrun.owned_processes(state, token)[0]:
            time.sleep(0.3)

    subprocess.run(helper + ['snapshot', state, token])
    running, zombies = axrun.owned_processes(state, token)
    # 좀비는 이미 종료되어 부모의 회수만 기다리는 상태이므로 정지 실패가 아니다.
    result.update(result='STOPPED' if not running else 'STOP_FAILED',
                  running=[list(p) for p in running], zombies=[list(p) for p in zombies],
                  finished=time.strftime('%F %T'))
    if previous and previous.get('result') == 'STOPPED' and not result['signalled']:
        result['repeat_of'] = previous.get('finished')

    write_json(run_dir / 'stop.json', result)
    run['status'] = 'STOPPED' if not running else 'STOP_FAILED'
    write_json(run_dir / 'run.json', run)
    if not running and 'domain' in run:
        set_record(run['domain'], run['run_id'], release=True)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if not running else 1


# 실행 상태와 남은 소유 프로세스·좀비 수, 마지막 종료 결과를 출력한다.
def status(run_dir):
    run_dir, run = load_run(run_dir)
    token = (run_dir / 'token').read_text().strip() if (run_dir / 'token').exists() else None
    running, zombies = (axrun.owned_processes(run_dir / 'own', token)
                        if (run_dir / 'own').is_dir() else ([], []))
    report = {k: run.get(k) for k in ('run_id', 'model', 'status', 'domain', 'partition', 'launch')}
    report.update(running=len(running), zombies=len(zombies),
                  processes=[f'{p} {c}' for p, _, c in running][:30],
                  stop=read_json(run_dir / 'stop.json'))
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


# 실행 중인 시뮬레이션에서 기본 통신 확인을 하고 결과 행을 summary에 더한 뒤 종료 코드를 돌려준다.
def probe(run_dir, timeout=120.0, summary=None):
    run_dir, run = load_run(run_dir)
    summary = summary or Summary(run_dir)
    if run.get('status') != 'RUNNING':
        summary.add('comm:probe', 'BLOCKED', note=f"simulation status {run.get('status')}")
        return 2

    env = axrun.ros_env(run['install_base'], {'ROS_DOMAIN_ID': str(run['domain']),
                                              'GZ_PARTITION': run['partition']})
    log = run_dir / f'probe_{time.strftime("%H%M%S")}.log'
    args = [a for robot, ns in run['robots'] for a in ('--robot', f'{robot}={ns}')]
    rc, sec = axrun.run_step([sys.executable, HERE / 'comm_probe.py', *args, '--timeout', timeout],
                             log, env=env, timeout=timeout + 60)

    lines = [line for line in Path(log).read_text().splitlines() if line.startswith('{')]
    data = json.loads(lines[-1]) if lines else None
    if not data:
        summary.add('comm:probe', 'FAIL', rc, sec, log, 'probe produced no result')
        return 1

    for item in data['checks']:
        summary.add(f"comm:{item['name']}", item['status'], rc, sec, log,
                    f"{item['topic']}: {item['detail']}")
    info = data['info']
    summary.add('comm:graph', 'INFO',
                note=f"topics {info['topics_seen']}, nodes {info['nodes_seen']} "
                '(reference only)')

    alive = run.get('launch') and axrun.is_running(*run['launch'])
    summary.add('comm:launch_alive', 'PASS' if alive else 'FAIL', note='launch process after probe')
    return rc if alive else 1


# 기본 통신 smoke: 시작 → 확인 → 정지. 제어기·추종·정지 검증이 아니다.
def smoke(model, timeout=120.0, domain=None, install_base=None, run=None, summary=None):
    run_dir = start(model, domain, install_base, run)
    summary = summary or Summary(run_dir)
    info = read_json(run_dir / 'run.json')
    if info.get('status') != 'RUNNING':
        summary.add('comm:start', 'FAIL', log=run_dir / 'launch.log', note=info.get('error', ''))
        for robot, _ in ROBOTS[model]:
            summary.add(f'comm:{robot}', 'BLOCKED', note='simulation did not start')
    else:
        summary.add('comm:start', 'PASS', log=run_dir / 'launch.log',
                    note=f"domain {info['domain']}, partition {info['partition']}, "
                    f"other nodes before start: {info.get('other_nodes_before_start')}")
        probe(run_dir, timeout, summary)

    if not (run_dir / 'own').is_dir():
        summary.add('comm:stop', 'INFO', note='no process was started')
        return run_dir, summary

    rc = stop(run_dir)
    stopped = read_json(run_dir / 'stop.json') or {}
    summary.add('comm:stop', 'PASS' if rc == 0 else 'FAIL', rc, log=run_dir / 'stop.json',
                note=f"{stopped.get('result')}, zombies {len(stopped.get('zombies', []))}")
    return run_dir, summary


# start·status·probe·stop·smoke 하위 명령을 해석해 실행하고 종료 코드를 돌려준다.
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    for name in ('start', 'smoke'):
        p = sub.add_parser(name)
        p.add_argument('--model', choices=sorted(ROBOTS), default='ax1')
        p.add_argument('--domain', type=int)
        p.add_argument('--install-base')
        if name == 'smoke':
            p.add_argument('--timeout', type=float, default=120.0)
    for name in ('status', 'stop', 'probe'):
        p = sub.add_parser(name)
        p.add_argument('run_dir')
        if name == 'probe':
            p.add_argument('--timeout', type=float, default=120.0)

    args = parser.parse_args()
    if args.cmd == 'start':
        run_dir = start(args.model, args.domain, args.install_base)
        status_ = read_json(run_dir / 'run.json')['status']
        print(run_dir)
        return 0 if status_ == 'RUNNING' else 1
    if args.cmd == 'smoke':
        run_dir, summary = smoke(args.model, args.timeout, args.domain, args.install_base)
        print(f'basic communication smoke (reception only): {summary.path}')
        return summary.exit_code()
    if args.cmd == 'probe':
        summary = Summary(args.run_dir, f'probe_summary_{time.strftime("%H%M%S")}.tsv')
        return probe(args.run_dir, args.timeout, summary)
    return {'status': status, 'stop': stop}[args.cmd](args.run_dir)


if __name__ == '__main__':
    sys.exit(main())
