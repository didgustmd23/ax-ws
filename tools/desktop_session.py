#!/usr/bin/env python3
# Windows 바로가기에서 호출하는 Ubuntu 세션 관리자. start는 Gazebo만, remote는 리모컨만, stop은 이 작업공간만 정리한다.
"""Start/stop only this workspace's AX simulation, called by Windows shortcuts."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
import urllib.request


# /proc에서 PID·부모 PID·시작 시각·실행 인자·환경을 읽는다. 종료된 프로세스는 None으로 처리한다.
def process(pid):
    try:
        path = Path('/proc') / str(pid)
        stat = (path / 'stat').read_text().split(') ', 1)[1].split()
        if stat[0] == 'Z':
            return None
        env = dict(item.split(b'=', 1) for item in (path / 'environ').read_bytes().split(b'\0') if b'=' in item)
        return {'pid': int(pid), 'parent': int(stat[1]), 'start': stat[19],
                'args': (path / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace'),
                'env': {k.decode(): v.decode(errors='replace') for k, v in env.items()}}
    except (OSError, ValueError):
        return None


# AMENT 설치 경로가 이 workspace인 AX 프로세스만 찾는다. 다른 작업공간의 세션은 제외한다.
def discover(workspace):
    result = {'launch': [], 'remote': [], 'keyboard': [], 'gazebo': [], 'terminal': []}
    prefix = str(workspace / 'install')
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        item = process(entry.name)
        if not item:
            continue
        args, env = item['args'], item['env']
        same = any(path == prefix or path.startswith(prefix + '/')
                   for path in env.get('AMENT_PREFIX_PATH', '').split(':'))
        if not same:
            continue
        if 'ros2 launch ax_control simulation.launch.py' in args:
            result['launch'].append(item)
        elif 'python3' in args and 'remote_control.py' in args:
            result['remote'].append(item)
        elif 'python3' in args and 'keyboard_teleop.py' in args:
            result['keyboard'].append(item)
        elif 'bash --rcfile ' in args and str(workspace / 'log/desktop_session/terminal.bash') in args:
            result['terminal'].append(item)
        elif args.startswith('gz sim ') or ('gz sim ' in args and ('ruby' in args or '/bin/sh' in args)):
            result['gazebo'].append(item)
    return result


# 종료 전에 자식 프로세스 목록을 확보해 부모가 먼저 끝나도 잔류 자식을 추적한다.
def descendants(roots):
    entries = [process(p.name) for p in Path('/proc').iterdir() if p.name.isdigit()]
    entries = [p for p in entries if p]
    owned = {p['pid']: p for p in roots}
    while True:
        extra = [p for p in entries if p['parent'] in owned and p['pid'] not in owned]
        if not extra:
            return list(owned.values())
        owned.update((p['pid'], p) for p in extra)


# 같은 PID라도 시작 시각이 다르면 새 프로세스이므로 종료 대상으로 취급하지 않는다.
def alive(item):
    current = process(item['pid'])
    return current is not None and current['start'] == item['start']


# SIGINT로 정상 정리 기회를 준 뒤 TERM/KILL로 단계적으로 종료한다. 각 단계에서 소유권을 다시 확인한다.
def stop_processes(roots):
    owned = descendants(roots)
    for sig, delay in [(signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 1)]:
        for item in owned:
            if alive(item):
                try:
                    os.kill(item['pid'], sig)
                except ProcessLookupError:
                    pass
        end = time.monotonic() + delay
        while time.monotonic() < end and any(alive(p) for p in owned):
            time.sleep(.1)
    remaining = [p['pid'] for p in owned if alive(p)]
    if remaining:
        raise RuntimeError('Could not stop session PIDs: ' + str(remaining))
    return len(owned)


# 로컬 HTTP 상태를 확인한다. robot_id가 있어도 로봇 준비 완료를 뜻하지는 않는다.
def remote_state(port):
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/state', timeout=1) as response:
            data = json.load(response)
        return data if data.get('robot') in ('robot_1', 'robot_2') else None
    except (OSError, ValueError):
        return None


# ROS 환경을 불러온 전용 프로세스 그룹으로 시작하고 stdout/stderr를 workspace 로그에 보관한다.
def spawn(workspace, logs, name, command, env):
    script = logs / (name + '.sh')
    script.write_text('set -eo pipefail\nsource /opt/ros/jazzy/setup.bash\nsource ' +
                      shlex.quote(str(workspace / 'install/setup.bash')) + '\nexec ' + shlex.join(command) + '\n')
    with (logs / (name + '.log')).open('ab', buffering=0) as output:
        child = subprocess.Popen(['bash', str(script)], cwd=workspace, env=env,
                                 stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                 start_new_session=True)
    item = process(child.pid)
    if not item:
        raise RuntimeError(name + ' exited at startup; check ' + str(logs))
    return item


# 파일 잠금으로 중복 클릭을 직렬화하고 Start/Remote/Stop/Status의 실행 범위를 분리한다.
def run(args):
    workspace = Path(args.workspace).expanduser().resolve()
    if not (workspace / 'src/ax_description').is_dir():
        raise RuntimeError('Not an AX workspace: ' + str(workspace))
    logs = workspace / 'log/desktop_session'
    logs.mkdir(parents=True, exist_ok=True)
    with (logs / 'session.lock').open('w') as lock:
        deadline = time.monotonic() + 180
        while True:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() > deadline:
                    raise RuntimeError('Another launcher is still busy')
                time.sleep(.2)
        found = discover(workspace)
        if args.action == 'status':
            return {'ok': True, 'launch_pids': [p['pid'] for p in found['launch']],
                    'remote_pids': [p['pid'] for p in found['remote']],
                    'terminal_pids': [p['pid'] for p in found['terminal']], 'remote_ready': bool(remote_state(args.port)),
                    'simulation_alive': bool(found['launch'] and found['gazebo'])}
        if args.action == 'stop':
            roots = [item for group in found.values() for item in group]
            stopped = stop_processes(roots)
            (logs / 'session.json').write_text(json.dumps({'running': False}))
            return {'ok': True, 'stopped': stopped}
        if not Path('/opt/ros/jazzy/setup.bash').is_file() or not (workspace / 'install/setup.bash').is_file():
            raise RuntimeError('Install ROS Jazzy and build the workspace first; see RUN_ON_ANOTHER_PC.md')
        release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
        if release.get('ID', '').strip('"') != 'ubuntu' or release.get('VERSION_ID', '').strip('"') != '24.04':
            raise RuntimeError('Select an Ubuntu 24.04 WSL distribution in the shortcut settings')
        if args.action == 'remote' and not (workspace / 'tools/start_remote.sh').is_file():
            raise RuntimeError('Remote files are missing; update the Ubuntu workspace first')
        if len(found['launch']) > 1 or len(found['remote']) > 1:
            raise RuntimeError('Duplicate sessions detected. Use AX Stop first')
        # Gazebo가 사라지고 launch만 남은 이전 실패 세션은 재사용하지 않는다.
        if found['launch'] and not found['gazebo']:
            stop_processes(found['launch'] + found['keyboard'])
            found = discover(workspace)
        switched_world = False
        selected_world = getattr(args, 'world', 'ax_test.world')
        launch_tool = args.tool
        if args.action == 'start' and getattr(args, 'switch_world', False):
            package_share = workspace / 'install/ax_description/share/ax_description'
            installed_world = (package_share / 'world_assets/aws_small_warehouse/aws_warehouse_harmonic.world'
                               if selected_world == 'aws_warehouse.world' else package_share / 'worlds' / selected_world)
            if not installed_world.is_file():
                raise RuntimeError('Selected world is not installed. Build ax_description in the Ubuntu workspace first')
            if found['launch']:
                arguments = shlex.split(found['launch'][0].get('args', ''))
                active_world = next((value.split(':=', 1)[1] for value in arguments if value.startswith('world:=')), 'ax_test.world')
                active_tool = next((value.split(':=', 1)[1] for value in arguments if value.startswith('tool:=')), 'epick')
                if active_world != selected_world or active_tool != launch_tool:
                    # 바로가기 시작은 설정된 기본 공구를 적용한다. 같은 월드라도
                    # 이전 세션의 집게를 재사용해 흡착판 선택을 무시하지 않는다.
                    stop_processes([item for group in found.values() for item in group])
                    found = discover(workspace)
                    if any(found.values()):
                        raise RuntimeError('Previous AX session did not stop; world switch cancelled')
                    switched_world = True
        if args.action == 'remote' and found['keyboard']:
            stop_processes(found['keyboard'])
        if args.action == 'remote' and remote_state(args.port) and not found['remote']:
            raise RuntimeError('Port is already used by another session; leave it running or select another port')
        env = {key: value for key, value in os.environ.items() if key in (
            'HOME', 'USER', 'LANG', 'DISPLAY', 'WAYLAND_DISPLAY', 'XDG_RUNTIME_DIR',
            'PULSE_SERVER', 'DBUS_SESSION_BUS_ADDRESS', 'ROS_DOMAIN_ID')}
        env.update(PATH='/usr/local/bin:/usr/bin:/bin', QT_QPA_PLATFORM='xcb', LIBGL_ALWAYS_SOFTWARE='1',
                   QT_X11_NO_MITSHM='1', LANG='C.UTF-8')
        env['GZ_PARTITION'] = (found['launch'][0]['env'].get('GZ_PARTITION', '') if found['launch'] else
                               'ax-desktop-' + hashlib.sha256(str(workspace).encode()).hexdigest()[:12])
        created = []
        reused = bool(found['launch'])
        remote_only = args.action == 'remote'
        try:
            terminal = logs / 'terminal.bash'
            terminal.write_text('source ~/.bashrc\nsource /opt/ros/jazzy/setup.bash\nsource ' +
                shlex.quote(str(workspace / 'install/setup.bash')) + '\ncd ' + shlex.quote(str(workspace)) +
                '\nexport GZ_PARTITION=' + shlex.quote(env['GZ_PARTITION']) +
                "\ntrap 'exit 0' TERM HUP\n")
            if not found['launch'] and not remote_only:
                if found['gazebo']:
                    raise RuntimeError('Orphan Gazebo processes found; use AX Stop first')
                found['launch'] = [spawn(workspace, logs, 'gazebo', ['ros2', 'launch', 'ax_control',
                    'simulation.launch.py', 'robots:=2', 'tool:=' + launch_tool,
                    'world:=' + getattr(args, 'world', 'ax_test.world')], env)]
                created.extend(found['launch'])
            if not remote_only:
                if not reused:
                    command = ('source /opt/ros/jazzy/setup.bash; source ' +
                        shlex.quote(str(workspace / 'install/setup.bash')) + '; python3 ' +
                        shlex.quote(str(workspace / 'tools/verify_unified_runtime.py')) + ' --world ' + shlex.quote(selected_world))
                    deadline = time.monotonic() + 110
                    ready = False
                    # 서비스 응답 대기 중인 검사기를 20초마다 죽이지 않는다.
                    # 한 검사기를 유지해 서버가 응답할 클라이언트가 사라지는 상황을 줄인다.
                    with (logs / 'readiness.log').open('w') as output:
                        checked = subprocess.Popen(['bash', '-c', command], cwd=workspace, env=env,
                            stdout=output, stderr=subprocess.STDOUT)
                        try:
                            while time.monotonic() < deadline and alive(found['launch'][0]):
                                if checked.poll() is not None:
                                    ready = checked.returncode == 0
                                    break
                                time.sleep(.5)
                        finally:
                            if checked.poll() is None:
                                # 먼저 시뮬레이션을 정리한 뒤 미완료 검사기를 종료한다.
                                stop_processes(created)
                                created.clear()
                                checked.terminate()
                                try:
                                    checked.wait(timeout=5)
                                except subprocess.TimeoutExpired:
                                    checked.kill()
                                    checked.wait()
                    if not ready:
                        raise RuntimeError('Robot startup check failed; see ' + str(logs / 'readiness.log'))
                receipt = {'ok': True, 'reused': reused, 'launch_pid': found['launch'][0]['pid'],
                           'terminal_rcfile': str(terminal), 'workspace': str(workspace),
                           'world_switched': switched_world}
                (logs / 'session.json').write_text(json.dumps(receipt, indent=2))
                return receipt
            if not found['remote']:
                found['remote'] = [spawn(workspace, logs, 'remote', ['python3',
                    str(workspace / 'tools/remote_control.py'), '--port', str(args.port)], env)]
                created.extend(found['remote'])
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                if not all(alive(p) for p in found['launch'] + found['remote']):
                    raise RuntimeError('Startup process exited; check logs in ' + str(logs))
                state = remote_state(args.port)
                if state and (remote_only or reused or state.get('arm_ready')):
                    break
                time.sleep(.3)
            else:
                raise RuntimeError('Startup timed out; check Gazebo and logs in ' + str(logs))
            receipt = {'ok': True, 'url': f'http://localhost:{args.port}', 'reused': reused,
                       'launch_pid': found['launch'][0]['pid'] if found['launch'] else None,
                       'remote_pid': found['remote'][0]['pid']}
            (logs / 'session.json').write_text(json.dumps(receipt, indent=2))
            return receipt
        except BaseException:
            if created:
                stop_processes(created)
            raise


# CLI 인자를 받아 마지막 줄에 JSON 결과를 출력한다. Windows 런처가 ok와 종료 코드를 함께 확인한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'remote', 'stop', 'status'])
    parser.add_argument('--workspace', default='~/ax-play-ws')
    parser.add_argument('--tool', choices=['robotiq', 'epick'], default='epick')
    parser.add_argument('--world', choices=['ax_test.world', 'planned/ax_warehouse.world', 'aws_warehouse.world'], default='ax_test.world')
    parser.add_argument('--switch-world', action='store_true', help='Switch the owned AX session if another world is active')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    try:
        print(json.dumps(run(args)))
    except Exception as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}))
        raise SystemExit(1)


if __name__ == '__main__':
    main()
