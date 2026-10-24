"""Shared pieces of the AX run tools: run folders, step summary, ROS environment, processes.

Result states of one step:
  PASS     ran and passed
  FAIL     ran and failed
  BLOCKED  not run because a prerequisite step of this run did not pass
  NOT_RUN  could not run (missing tool, zero test results, ...)
  INFO     reference only, never a pass/fail ground
Exit code of a whole run: 0 every required step PASS, 1 any FAIL, 2 no FAIL but a required step
BLOCKED/NOT_RUN (incomplete), 3 usage error.
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
import own_procs  # noqa: E402  (existing helper, used read-only here)

REQUIRED_STATES = ('PASS', 'FAIL', 'BLOCKED', 'NOT_RUN')


def run_root():
    """Folder holding run folders and the domain allocation records (AX_RUN_ROOT)."""
    return Path(os.environ.get('AX_RUN_ROOT', '/tmp/ax_runs')).resolve()


def new_run_dir(kind):
    run_id = time.strftime('%Y%m%d-%H%M%S') + '_' + secrets.token_hex(3)
    path = run_root() / f'{run_id}_{kind}'
    path.mkdir(parents=True)
    return run_id, path


def write_json(path, data):
    """Write JSON atomically (temp file + rename) so a reader never sees half a record."""
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')
    os.replace(tmp, path)


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def warning_count(log):
    """Lines that mention a warning; reported so warnings are not hidden by a short summary."""
    try:
        return sum('warn' in line.lower() for line in Path(log).read_text(errors='replace').splitlines())
    except OSError:
        return 0


class Summary:
    """Step table written to summary.tsv after every step (full logs stay in the run folder)."""

    def __init__(self, run_dir, name='summary.tsv'):
        self.path = Path(run_dir) / name
        self.rows = []
        self.write()

    def add(self, step, status, rc='', sec='', log='', note=''):
        warn = warning_count(log) if log else ''
        self.rows.append(dict(step=step, status=status, exit=rc, sec=sec, warnings=warn,
                              log=str(log), note=note))
        self.write()
        print(f'{status:8} {step}' + (f'  ({note})' if note else ''), flush=True)
        return status

    def status(self, step):
        return next((r['status'] for r in self.rows if r['step'] == step), None)

    def write(self):
        head = 'step\tstatus\texit\tsec\twarnings\tlog\tnote\n'
        body = ''.join('\t'.join(str(r[k]).replace('\t', ' ').replace('\n', ' ')
                                 for k in ('step', 'status', 'exit', 'sec', 'warnings', 'log', 'note'))
                       + '\n' for r in self.rows)
        self.path.write_text(head + body)

    def exit_code(self):
        states = [r['status'] for r in self.rows if r['status'] in REQUIRED_STATES]
        if 'FAIL' in states:
            return 1
        if 'BLOCKED' in states or 'NOT_RUN' in states or not states:
            return 2
        return 0


def run_step(cmd, log, env=None, cwd=WS, timeout=None):
    """Run a command with its whole stdout/stderr in `log`; returns (exit code, seconds)."""
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


def clean_env():
    """Current environment without entries pointing into this workspace or earlier run folders,
    so a previously sourced install/ cannot leak into a fresh build or check."""
    env = {}
    stale = (str(WS), str(run_root()))
    for key, value in os.environ.items():
        if any(s in value for s in stale) and (os.pathsep in value or key.endswith('PATH')):
            kept = [e for e in value.split(os.pathsep) if e and not any(e.startswith(s) for s in stale)]
            if kept:
                env[key] = os.pathsep.join(kept)
        else:
            env[key] = value
    return env


def ros_env(install_base=None, extra=None):
    """Environment after sourcing ROS Jazzy and, if given, this workspace's install folder."""
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


# Processes ----------------------------------------------------------------------------------

def proc_state(pid):
    """One-letter /proc state (R, S, D, Z, ...) or None when the PID does not exist."""
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return None
    return stat[stat.rindex(')') + 2]


def is_running(pid, start):
    """True only for that exact (pid, start time) and not a terminated, unreaped zombie."""
    return own_procs.start_time(pid) == start and proc_state(pid) not in (None, 'Z')


def self_identity():
    pid = os.getpid()
    return [pid, own_procs.start_time(pid)]


def owned_processes(state_dir, token=None):
    """Owned processes of a run, read-only: (running, zombies) lists of (pid, start, cmd).

    Uses the recorded (pid, start) pairs of own_procs.py and, if a token is given, processes that
    carry that token. own_procs.py `alive` checks only the start time, so it also lists zombies
    (terminated, not yet reaped); they are separated here and never count as running."""
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


def git_state():
    def git(*args):
        result = subprocess.run(['git', *args], cwd=WS, capture_output=True, text=True)
        return result.stdout.strip() if result.returncode == 0 else None
    status = git('status', '--porcelain=v1', '--untracked-files=all')
    return {'head': git('rev-parse', 'HEAD'), 'branch': git('branch', '--show-current'),
            'changed_paths': None if status is None else len(status.splitlines())}
