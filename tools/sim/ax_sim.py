#!/usr/bin/env python3
"""Isolated headless Gazebo run of the AX simulation with owned-process cleanup.

  ax_sim.py start  [--model ax1|ax2|both] [--domain N] [--install-base DIR]   prints the run folder
  ax_sim.py status RUN_DIR
  ax_sim.py probe  RUN_DIR [--timeout S]      basic communication checks on a running simulation
  ax_sim.py stop   RUN_DIR                    stops only this run's processes; safe to repeat
  ax_sim.py smoke  [--model ...] [--timeout S]   start + probe + stop ("basic communication smoke")

Isolation: every run gets its own GZ_PARTITION and a ROS_DOMAIN_ID (default range 60-99, inside the
0-101 range ROS 2 recommends for Linux default settings) recorded under $AX_RUN_ROOT/alloc. Duplicates
are prevented only between runs that share that allocation folder; programs started without this
tool (or with another AX_RUN_ROOT) are not isolated from.

The smoke checks message reception only (/clock, per-robot joint_states and odom). It does not verify
controller activation, command tracking or stopping.
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
# Namespaces of simulation.launch.py (robot loop, `legacy = is_arm and selection == 'ax1'`):
# ax1 alone keeps the original un-namespaced topics, otherwise /ax/<robot_id>. Update with the launch.
ROBOTS = {
    'ax1': [('robot_1', '')],
    'ax2': [('robot_2', '/ax/robot_2')],
    'both': [('robot_1', '/ax/robot_1'), ('robot_2', '/ax/robot_2')],
}
STOP_STEPS = (('SIGINT', 8.0), ('SIGTERM', 4.0), ('SIGKILL', 2.0))


# Domain allocation --------------------------------------------------------------------------

class AllocLock:
    """Exclusive lock over the allocation folder; allocation, reclaim and release happen under it."""

    def __init__(self):
        self.folder = axrun.run_root() / 'alloc'

    def __enter__(self):
        self.folder.mkdir(parents=True, exist_ok=True)
        self.handle = open(self.folder / '.lock', 'w')
        fcntl.flock(self.handle, fcntl.LOCK_EX)
        return self.folder

    def __exit__(self, *exc):
        fcntl.flock(self.handle, fcntl.LOCK_UN)
        self.handle.close()


def reclaim_reason(record):
    """Why a domain record may be reclaimed, or None while it may still be in use.

    STARTING: never reclaimed while the starting process (pid + start time) still runs, because its
    owned processes may not be recorded yet. Otherwise a record is free only when its run has no
    running owned process (zombies do not count)."""
    if not isinstance(record, dict):
        return None                                  # unreadable record: keep it (conservative)
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


def allocate(run_id, run_dir, requested=None):
    with AllocLock() as folder:
        for domain in ([requested] if requested is not None else DOMAINS):
            path = folder / f'domain_{domain}.json'
            if path.exists():
                reason = reclaim_reason(read_json(path))
                if reason is None:
                    continue
                with open(folder / 'reclaimed.log', 'a') as log:
                    log.write(f'{time.strftime("%F %T")} domain {domain} reclaimed for {run_id}: {reason}\n')
            write_json(path, {'domain': domain, 'run_id': run_id, 'run_dir': str(run_dir),
                              'state': 'STARTING', 'starter': axrun.self_identity(),
                              'since': time.strftime('%F %T')})
            return domain
    return None


def set_record(domain, run_id, state=None, release=False):
    with AllocLock() as folder:
        path = folder / f'domain_{domain}.json'
        record = read_json(path)
        if not record or record.get('run_id') != run_id:
            return                                    # reclaimed by someone else: leave it alone
        if release:
            path.unlink()
        else:
            record['state'] = state
            write_json(path, record)


# Run lifecycle ------------------------------------------------------------------------------

def load_run(run_dir):
    run_dir = Path(run_dir).resolve()
    run = read_json(run_dir / 'run.json')
    if not run:
        sys.exit(f'{run_dir}: run.json not found (not an ax_sim run folder)')
    return run_dir, run


def start(model, domain=None, install_base=None, run=None):
    """Start the simulation; returns the run folder (also on failure, with run.json status)."""
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
        env = axrun.ros_env(install, {'ROS_DOMAIN_ID': str(chosen), 'GZ_PARTITION': info['partition']})
    except (RuntimeError, subprocess.CalledProcessError) as error:
        info.update(status='START_FAILED', error=str(error))
        write_json(run_dir / 'run.json', info)
        set_record(chosen, run_id, release=True)
        return run_dir

    # Best-effort look for other participants on this domain (reference only).
    try:
        graph = subprocess.run([sys.executable, HERE / 'comm_probe.py', '--graph-only'], env=env,
                               capture_output=True, text=True, timeout=30)
        info['other_nodes_before_start'] = json.loads(graph.stdout.splitlines()[-1])['other_nodes']
    except (subprocess.TimeoutExpired, ValueError, IndexError, KeyError):
        info['other_nodes_before_start'] = 'graph probe failed'

    # Reuse the existing owned-process helper unchanged: init its state in the run folder, then
    # own_start the launch from a shell that sources owned_procs.sh with this state and token.
    token = uuid.uuid4().hex
    (run_dir / 'token').write_text(token + '\n')
    subprocess.run([sys.executable, HERE / 'own_procs.py', 'init', run_dir / 'own'], check=True)
    launch = ['ros2', 'launch', 'ax_control', 'simulation.launch.py', f'robot_model:={model}', 'gui:=false']
    shell = ('source "$1"; OWN_STATE="$2"; OWN_TOKEN="$3"; shift 3; own_start "$@"; '
             'echo "$OWN_LAST_PID"')
    out = subprocess.run(['bash', '-c', shell, '_', HERE / 'owned_procs.sh', run_dir / 'own', token,
                          run_dir / 'launch.log', *launch], env=env, capture_output=True, text=True)
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


def stop(run_dir):
    """Stop this run's owned processes. Returns 0 STOPPED, 1 STOP_FAILED, 2 no ownership record."""
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
    # The saved state and token are restored; own_init is never called again.
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
    # A zombie has terminated and only waits to be reaped by its parent: not a stop failure.
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


def probe(run_dir, timeout=120.0, summary=None):
    """Basic communication checks; appends rows to `summary` and returns the probe exit code."""
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
    summary.add('comm:graph', 'INFO', note=f"topics {info['topics_seen']}, nodes {info['nodes_seen']} "
                '(reference only)')
    alive = run.get('launch') and axrun.is_running(*run['launch'])
    summary.add('comm:launch_alive', 'PASS' if alive else 'FAIL', note='launch process after probe')
    return rc if alive else 1


def smoke(model, timeout=120.0, domain=None, install_base=None, run=None, summary=None):
    """Basic communication smoke: start, probe, stop. Not a controller/tracking/stop verification."""
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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
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
