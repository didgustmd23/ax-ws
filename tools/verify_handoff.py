#!/usr/bin/env python3
"""Run the existing simulation in an isolated domain; preserve raw evidence.

Usage: source ROS/workspace setup, then
    python3 tools/verify_handoff.py [--dependencies-only] [--output-dir DIR] [--headless]
        [--domain-id N|auto] [--ready-timeout S]
Does not modify the robot, install packages, or terminate other simulations.
The domain is recorded with tools/sim (ax_sim.py allocation, AX_RUN_ROOT) and refused when
another run holds it or other ROS nodes are already on it; that only isolates from runs that
share the allocation folder and from nodes visible at start. The launch is tracked and stopped
with the existing owned-process tools (own_procs.py, ax_sim.stop), never by process group.
Importing this module only defines the parsing and decision functions; the
evidence folder, ROS environment and simulation start in main().
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SIM_TOOLS = ROOT / 'tools' / 'sim'
DEFAULT_DOMAIN_ID = 87

# Default launch: single AX_1 (robot_model:=ax1) with the primitive gripper, root namespace.
# Names follow src/ax_control/ax_control/model_contracts.py and config/ax1_controllers.yaml;
# tools/tests/test_verify_handoff.py checks that they stay equal.
REQUIRED_CONTROLLERS = ('joint_state_broadcaster', 'diff_drive_controller',
                        'arm_controller', 'finger_controller')
WHEEL_JOINTS = ('front_left_wheel_joint', 'rear_left_wheel_joint',
                'front_right_wheel_joint', 'rear_right_wheel_joint')
ARM_JOINTS = ('column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint',
              'gripper_rotation_joint')
FINGER_JOINTS = ('left_finger_joint', 'right_finger_joint')
# Wheels take velocity (rad/s); arm (rad) and fingers (m) take position.
EXPECTED_COMMAND_INTERFACES = tuple(
    [f'{joint}/velocity' for joint in WHEEL_JOINTS]
    + [f'{joint}/position' for joint in ARM_JOINTS + FINGER_JOINTS])
CONTROLLER_STATES = ('active', 'inactive', 'unconfigured', 'finalized')
# Reference interfaces that the chainable Jazzy diff_drive_controller lists among the command
# interfaces (confirmed in an isolated run). Only these two are kept apart from hardware.
CONTROLLER_REFERENCE_INTERFACES = ('diff_drive_controller/linear/velocity',
                                   'diff_drive_controller/angular/velocity')

# Readiness limit for the whole start-up, counted from just after the launch starts. Each
# probe's spin and CLI queries are limited to the remaining time (a query gets at least
# QUERY_MIN_TIMEOUT_S), and a probe that ends at or after the limit never counts as ready.
# The launch runs two spawners per robot with --controller-manager-timeout 120 and
# --switch-timeout 60, but the two values are not simply added:
# - Jazzy spawners serialize on a file lock (~/.ros/locks, shared by every spawner of the
#   user on this machine). A waiting spawner retries 5 x (20 s lock wait + 3 s) and then
#   exits, so the second spawner fails unless the first releases the lock within ~115 s.
#   The 120 s manager wait (Gazebo start, model spawn, plugin load) therefore has to fit in
#   that window for a successful start.
# - The spawner holding the lock second then works alone. The main spawner has three
#   controllers, each up to 3 service calls x 10 s (--service-call-timeout default) plus
#   one switch of up to 60 s: 3 x (30 + 60) = 270 s. The drive spawner needs 90 s.
# Worst successful start: 115 + 270 = 385 s; 35 s margin covers discovery of the command
# subscribers and the probe calls themselves. Past this the start-up is treated as failed.
READY_TIMEOUT_S = 420.0
READY_POLL_S = 5.0
QUERY_TIMEOUT_S = 20.0
QUERY_MIN_TIMEOUT_S = 0.5

# rosdep keys that are not ROS packages, and how they are checked. Any other declared
# dependency must be found in the ament index; unknown keys are reported as failures.
SYSTEM_DEPENDENCIES = {
    # simulation.launch.py imports yaml inside the Python that runs `ros2 launch`.
    'python3-yaml': 'yaml',
}

ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;?]*[ -/]*[@-~]')
COMMAND_INTERFACE_LINE = re.compile(
    r'^(?P<name>\S+)(?: \[[^\]]+\])? \[(?P<available>available|unavailable)\]'
    r' \[(?P<claimed>claimed|unclaimed)\]$')


def strip_ansi(text):
    """Remove terminal colour codes that ros2controlcli adds to its output."""
    return ANSI_ESCAPE.sub('', text or '')


def parse_controllers(text):
    """Parse `ros2 control list_controllers` output into {name: {type, state}}."""
    controllers, errors = {}, []
    for raw in strip_ansi(text).splitlines():
        line = raw.strip()
        if not line or line == 'No controllers are currently loaded!':
            continue
        fields = line.split()
        # Each row is "<name> <type> <state>"; anything else is not trusted as a state.
        if len(fields) != 3 or fields[2] not in CONTROLLER_STATES:
            errors.append(f'unparsed line: {line}')
        elif fields[0] in controllers:
            errors.append(f'duplicate controller: {fields[0]}')
        else:
            controllers[fields[0]] = {'type': fields[1], 'state': fields[2]}
    return {'controllers': controllers, 'errors': errors}


def evaluate_controllers(returncode, parsed, required=REQUIRED_CONTROLLERS):
    """Every required controller must be listed and exactly in state 'active'."""
    problems = []
    if returncode != 0:
        problems.append(f'list_controllers exited with {returncode}')
    problems += [f'list_controllers {e}' for e in parsed['errors']]
    states = {name: c['state'] for name, c in parsed['controllers'].items()}
    for name in required:
        if name not in states:
            problems.append(f'controller {name} not listed')
        elif states[name] != 'active':
            problems.append(f'controller {name} is {states[name]}')
    return {'returncode': returncode, 'states': states, 'required': list(required),
            'problems': problems, 'ok': not problems}


def parse_hardware_interfaces(text):
    """Parse the command section of `ros2 control list_hardware_interfaces`."""
    command, state, errors = {}, [], []
    section = None
    for raw in strip_ansi(text).splitlines():
        line = raw.strip()
        if not line:
            continue
        if line in ('command interfaces', 'state interfaces'):
            section = line.split()[0]
            continue
        if section == 'command':
            match = COMMAND_INTERFACE_LINE.match(line)
            if not match:
                errors.append(f'unparsed command interface line: {line}')
            elif match['name'] in command:
                errors.append(f'duplicate command interface: {match["name"]}')
            else:
                command[match['name']] = {'available': match['available'] == 'available',
                                          'claimed': match['claimed'] == 'claimed'}
        elif section == 'state':
            state.append(line.split()[0])
        else:
            errors.append(f'line before any section: {line}')
    if section is None:
        errors.append('no "command interfaces" section')
    return {'command': command, 'state': state, 'errors': errors}


def evaluate_interfaces(returncode, parsed, expected=EXPECTED_COMMAND_INTERFACES,
                        references=CONTROLLER_REFERENCE_INTERFACES):
    """The hardware command interfaces must equal the expected set, all available and claimed.

    Only the listed controller reference interfaces are kept apart; any other name, even
    with a controller-name prefix, is unexpected. No controller chain is configured, so a
    claimed reference interface is a failure.
    """
    references = {n: v for n, v in parsed['command'].items() if n in references}
    actual = {n: v for n, v in parsed['command'].items() if n not in references}
    missing = sorted(set(expected) - set(actual))
    unexpected = sorted(set(actual) - set(expected))
    unavailable = sorted(n for n in expected if n in actual and not actual[n]['available'])
    unclaimed = sorted(n for n in expected if n in actual and not actual[n]['claimed'])
    problems = []
    if returncode != 0:
        problems.append(f'list_hardware_interfaces exited with {returncode}')
    problems += [f'list_hardware_interfaces {e}' for e in parsed['errors']]
    problems += [f'command interface {n} missing' for n in missing]
    problems += [f'unexpected command interface {n}' for n in unexpected]
    problems += [f'command interface {n} unavailable' for n in unavailable]
    problems += [f'command interface {n} unclaimed' for n in unclaimed]
    problems += [f'controller reference interface {n} claimed (no chain configured)'
                 for n, v in sorted(references.items()) if v['claimed']]
    return {'returncode': returncode, 'expected': list(expected),
            'controller_references': sorted(references),
            'claimed': sorted(n for n, v in actual.items() if v['claimed']),
            'missing': missing, 'unexpected': unexpected, 'unavailable': unavailable,
            'unclaimed': unclaimed, 'problems': problems, 'ok': not problems}


def readiness_status(controllers, interfaces, subscriber_counts):
    """Ready only with active controllers, claimed interfaces and a subscriber per topic."""
    reasons = list(controllers['problems']) + list(interfaces['problems'])
    if not subscriber_counts:
        reasons.append('no command topics checked')
    reasons += [f'no subscriber on {topic}'
                for topic, count in sorted(subscriber_counts.items()) if not count]
    return {'ready': not reasons, 'reasons': reasons,
            'controllers_returncode': controllers['returncode'],
            'interfaces_returncode': interfaces['returncode'],
            'subscriber_counts': dict(subscriber_counts)}


def wait_until_ready(probe, launch_poll, timeout_s=READY_TIMEOUT_S, poll_s=READY_POLL_S,
                     clock=time.monotonic, sleep=time.sleep):
    """Poll probe(remaining_s) until ready, the launch exits, or timeout_s passes.

    probe() returns readiness_status() and should keep its own waits within remaining_s;
    launch_poll() returns None while the launch runs.
    """
    start = clock()
    deadline = start + timeout_s
    attempts = []

    def result(ready, reason):
        return {'ready': ready, 'reason': reason, 'timeout_s': timeout_s,
                'elapsed_s': clock() - start, 'attempts': attempts}

    def timed_out(reasons):
        return result(False, f'not ready within {timeout_s:g} s: ' + '; '.join(reasons))

    reasons = ['no probe completed']
    while True:
        code = launch_poll()
        if code is not None:
            return result(False, f'launch exited with code {code} before readiness')
        remaining = deadline - clock()
        if remaining <= 0:
            return timed_out(reasons)
        status = probe(remaining)
        elapsed = clock() - start
        attempts.append(dict(status, elapsed_s=elapsed))
        code = launch_poll()
        if code is not None:
            return result(False, f'launch exited with code {code} before readiness')
        # The limit is checked before the ready result: a probe that ends at or after the
        # limit (including one that ran past it) never leads to a command.
        if elapsed >= timeout_s:
            return timed_out(['ready only at or after the limit'] if status['ready']
                             else status['reasons'])
        if status['ready']:
            return result(True, None)
        reasons = status['reasons']
        sleep(max(0.0, min(poll_s, deadline - clock())))


def start_when_ready(probe, launch_poll, on_ready, **wait_options):
    """Call on_ready() (first command and motion test) only after readiness."""
    readiness = wait_until_ready(probe, launch_poll, **wait_options)
    if readiness['ready']:
        on_ready()
    return readiness


def final_verdict(result):
    """Return (exit code, failures): 0 pass, 2 motion outside tolerance, 1 incomplete."""
    checks = result.get('checks', {})
    motion = checks.get('motion', {})
    failures = []
    if 'error' in result:
        failures.append(f'error: {result["error"]}')
    if not checks.get('readiness', {}).get('ready'):
        failures.append('readiness: '
                        + str(checks.get('readiness', {}).get('reason', 'not checked')))
    for name in ('xacro', 'check_urdf'):
        if checks.get(name, {}).get('returncode') != 0:
            failures.append(f'{name} returncode {checks.get(name, {}).get("returncode")}')
    for name in ('controllers', 'hardware_interfaces'):
        if checks.get(name, {}).get('ok') is not True:
            failures.append(f'{name}: ' + '; '.join(
                checks.get(name, {}).get('problems', ['not checked'])))
    if not result.get('launch_alive_at_end'):
        failures.append('launch not alive at end')
    if result.get('cleanup', {}).get('result') != 'STOPPED':
        failures.append('cleanup: ' + str(result.get('cleanup', {}).get('result', 'not run')))
    if checks.get('scan', {}).get('received', 0) <= 0:
        failures.append('no front LiDAR scan received')
    if len(motion) != 7:
        failures.append(f'motion checked for {len(motion)} of 7 joints')
    failures += [f'{joint}: {v.get("status")}' for joint, v in motion.items()
                 if v.get('status') != 'motion_observed']
    if failures:
        return 1, failures
    # A distinct exit code keeps motion success separate from position accuracy.
    misses = [f'{joint} outside review tolerance: target={v.get("target")}'
              f' measured={v.get("measured_position")} error={v.get("absolute_target_error")}'
              f' tolerance={v.get("review_tolerance")} {v.get("unit", "")}'.rstrip()
              for joint, v in motion.items() if v.get('within_review_tolerance') is not True]
    return (2, misses) if misses else (0, [])


def declared_dependencies(src=ROOT / 'src'):
    """Return {package: [dependency names]} from the workspace package.xml files."""
    result = {}
    for manifest in sorted(Path(src).glob('*/package.xml')):
        package = ET.parse(manifest).getroot()
        result[package.findtext('name')] = sorted(
            {d.text.strip() for tag in ('depend', 'exec_depend', 'buildtool_depend')
             for d in package.findall(tag)})
    return result


def ament_prefix_lookup():
    """Return name -> install prefix or None, using the sourced ament index."""
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError

    def lookup(name):
        try:
            return get_package_prefix(name)
        except PackageNotFoundError:
            return None
    return lookup


def launch_python(which=shutil.which):
    """Return (interpreter, method): the Python that `ros2 launch` runs launch files in."""
    ros2 = which('ros2')
    if not ros2:
        return None, 'ros2 executable not found on PATH'
    try:
        with open(ros2, encoding='utf-8', errors='replace') as stream:
            first = stream.readline().strip()
    except OSError as e:
        return None, f'cannot read {ros2}: {e}'
    if not first.startswith('#!'):
        return None, f'{ros2} has no shebang'
    command = first[2:].split()
    if Path(command[0]).name == 'env' and len(command) > 1:
        interpreter = which(command[1])
    else:
        interpreter = command[0]
    if not interpreter:
        return None, f'interpreter of {ros2} ({first}) not found'
    return interpreter, f'shebang of {ros2}'


def check_python_module(python, module, runner=None):
    """Check that `python` can import `module` (isolated mode, no workspace cwd)."""
    runner = runner or subprocess.run
    code = ('import importlib, sys; m = importlib.import_module(sys.argv[1]); '
            'print(getattr(m, "__version__", "?"), getattr(m, "__file__", "?"))')
    method = f'{python} -I -c "import {module}"'
    if not python:
        return {'kind': 'system', 'method': method, 'ok': False,
                'detail': 'launch Python interpreter unknown'}
    try:
        p = runner([python, '-I', '-c', code, module], text=True, capture_output=True,
                   timeout=30)
        returncode, detail = p.returncode, (p.stdout or p.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        returncode, detail = None, repr(e)
    return {'kind': 'system', 'method': method, 'returncode': returncode,
            'ok': returncode == 0, 'detail': detail.splitlines()[-1] if detail else ''}


def check_dependencies(declared, lookup, python, runner=None):
    """Classify and check every declared dependency; nothing passes without a check."""
    result = {}
    for package, names in declared.items():
        result[package] = {}
        for name in names:
            if name in SYSTEM_DEPENDENCIES:
                result[package][name] = check_python_module(
                    python, SYSTEM_DEPENDENCIES[name], runner)
                continue
            prefix = lookup(name) if lookup else None
            entry = {'kind': 'ros', 'method': 'ament_index get_package_prefix',
                     'prefix': prefix, 'ok': prefix is not None}
            if prefix is None:
                entry['detail'] = ('ament index unavailable' if lookup is None else
                                   'not in the ament index and not a known system dependency')
            result[package][name] = entry
    return result


def dependencies_ok(dependencies):
    return all(entry['ok'] for deps in dependencies.values() for entry in deps.values())


def quat(p):
    q = p.get('orientation', {})
    return [float(q.get(k, 0 if k != 'w' else 1)) for k in ('w', 'x', 'y', 'z')]


def mul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def relative(poses, parent, child):
    a, b = poses[parent], poses[child]
    qa = quat(a)
    inv = [qa[0], -qa[1], -qa[2], -qa[3]]
    delta = [float(b.get('position', {}).get(k, 0))-float(a.get('position', {}).get(k, 0))
             for k in ('x', 'y', 'z')]
    return mul(inv, quat(b)), mul(mul(inv, [0]+delta), qa)[1:]


def launch_command(headless=False):
    """Simulation launch command; the launch's own defaults (ax1, GUI) apply unless headless."""
    command = ['ros2', 'launch', 'ax_control', 'simulation.launch.py']
    if headless:
        command.append('gui:=false')
    return command


def domain_argument(value):
    """--domain-id: 'auto' (free domain of ax_sim.py's 60-99 range) or an ID 0-232."""
    if value == 'auto':
        return None
    try:
        domain = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError(f'not an integer or auto: {value}')
    if not 0 <= domain <= 232:
        raise argparse.ArgumentTypeError(f'ROS_DOMAIN_ID out of range 0-232: {domain}')
    return domain


def positive_seconds(value):
    seconds = float(value)
    if not seconds > 0:
        raise argparse.ArgumentTypeError(f'must be > 0: {value}')
    return seconds


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--dependencies-only', action='store_true',
                        help='check declared dependencies and exit without simulation')
    parser.add_argument('--output-dir', type=Path,
                        help='evidence folder (default docs/evidence/<KST timestamp>)')
    parser.add_argument('--headless', action='store_true',
                        help='pass gui:=false to simulation.launch.py (default: launch default)')
    parser.add_argument('--domain-id', type=domain_argument, default=DEFAULT_DOMAIN_ID,
                        help=f'ROS_DOMAIN_ID or auto (default {DEFAULT_DOMAIN_ID})')
    parser.add_argument('--ready-timeout', type=positive_seconds, default=READY_TIMEOUT_S,
                        help=f'readiness limit in s (default {READY_TIMEOUT_S:g}, see source)')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    stamp = datetime.datetime.now(
        datetime.timezone(datetime.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S')
    out = (args.output_dir or ROOT / 'docs' / 'evidence' / stamp).resolve()
    out.mkdir(parents=True, exist_ok=False)
    partition = f'ax_handoff_{stamp}_{os.getpid()}'
    result = {'date_kst': stamp, 'ros_domain_id': None, 'gz_partition': partition,
              'headless': args.headless, 'checks': {}}
    checks = result['checks']

    def run(name, command, timeout=20, merge=True):
        """Run a command, save its output; stdout is returned separately when merge=False."""
        try:
            p = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT if merge else subprocess.PIPE,
                               timeout=timeout)
            text, err, rc = p.stdout, p.stderr or '', p.returncode
        except subprocess.TimeoutExpired as e:
            raw = e.stdout or b''
            text = raw.decode(errors='replace') if isinstance(raw, bytes) else raw
            err, rc = f'timeout after {timeout} s', 124
        except OSError as e:
            text, err, rc = '', repr(e), 127
        (out / f'{name}.txt').write_text(text, encoding='utf-8')
        if not merge:
            (out / f'{name}.stderr.txt').write_text(err, encoding='utf-8')
        return rc, text

    def write_result():
        (out / 'results.json').write_text(json.dumps(result, ensure_ascii=False, indent=2),
                                          encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        print('Evidence:', out, flush=True)

    try:
        lookup = ament_prefix_lookup()
    except ImportError as e:
        lookup = None
        checks['ament_index_error'] = repr(e)
    python, python_method = launch_python()
    checks['launch_python'] = {'interpreter': python, 'method': python_method}
    dependencies = check_dependencies(declared_dependencies(), lookup, python)
    checks['dependencies'] = dependencies
    if args.dependencies_only:
        ok = dependencies_ok(dependencies)
        result['verdict'] = {'exit_code': 0 if ok else 1, 'failures': [
            f'{package}: {name} ({entry.get("detail", "check failed")})'
            for package, deps in dependencies.items()
            for name, entry in deps.items() if not entry['ok']]}
        write_result()
        return result['verdict']['exit_code']

    launch = node = log = None
    sim = domain = None
    try:
        # Existing tools/sim helpers: domain allocation records and owned-process cleanup.
        sys.path.insert(0, str(SIM_TOOLS))
        import ax_sim
        import axrun
        sim = ax_sim
        run_id = out.name
        domain = ax_sim.allocate(run_id, out, args.domain_id)
        if domain is None:
            raise RuntimeError(f'ROS_DOMAIN_ID {args.domain_id or "60-99"} is held by another'
                               f' run ({axrun.run_root() / "alloc"})')
        result['ros_domain_id'] = domain
        os.environ['ROS_DOMAIN_ID'] = str(domain)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ['GZ_PARTITION'] = partition
        token = uuid.uuid4().hex
        (out / 'token').write_text(token + '\n')
        axrun.write_json(out / 'run.json', {
            'run_id': run_id, 'kind': 'verify_handoff', 'status': 'STARTING',
            'domain': domain, 'partition': partition, 'started': time.strftime('%F %T')})
        # Refuse a domain that already has participants instead of mixing graphs.
        rc, text = run('domain_graph_before_start',
                       [sys.executable, str(SIM_TOOLS / 'comm_probe.py'), '--graph-only'], 30)
        try:
            others = json.loads(text.strip().splitlines()[-1])['other_nodes']
        except (ValueError, IndexError, KeyError):
            others = None
        # A ros2cli daemon only caches the graph and joins no simulation topic; recorded only.
        daemons = [n for n in others or [] if n.startswith('/_ros2cli_daemon_')]
        others = None if others is None else [n for n in others if n not in daemons]
        checks['domain_graph_before_start'] = {'returncode': rc, 'other_nodes': others,
                                               'ignored_ros2cli_daemons': daemons}
        if rc or others is None:
            raise RuntimeError('cannot check ROS nodes on the domain before start')
        if others:
            raise RuntimeError(f'ROS_DOMAIN_ID {domain} already has nodes: {others}')

        rc, _ = run('xacro', ['xacro', str(ROOT / 'src/ax_description/urdf/ax.urdf.xacro'),
                              '-o', str(out / 'ax.urdf')])
        checks['xacro'] = {'returncode': rc}
        if rc:
            raise RuntimeError('Xacro conversion failed')
        rc, _ = run('check_urdf', ['check_urdf', str(out / 'ax.urdf')])
        checks['check_urdf'] = {'returncode': rc}
        tree = ET.parse(out / 'ax.urdf').getroot()
        result['links'] = len(tree.findall('link'))
        result['joints'] = len(tree.findall('joint'))
        log = (out / 'simulation.log').open('w', encoding='utf-8')
        helper = [sys.executable, str(SIM_TOOLS / 'own_procs.py')]
        subprocess.run(helper + ['init', str(out / 'own')], check=True)
        # From here every child (launch, ros2 control CLI and the ros2 daemon it may spawn)
        # carries the token, so ax_sim.stop also stops a daemon started by this run.
        os.environ['AX_OWN_TOKEN'] = token
        command = launch_command(args.headless)
        checks['launch_command'] = command
        # AX_OWN_TOKEN (set above) marks every launch descendant, even detached ones.
        launch = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        launch_started = time.monotonic()
        subprocess.run(helper + ['register', str(out / 'own'), str(launch.pid),
                                 str(os.getpid())])
        run_info = axrun.read_json(out / 'run.json')
        run_info.update(status='RUNNING',
                        launch=[launch.pid, axrun.own_procs.start_time(launch.pid)])
        axrun.write_json(out / 'run.json', run_info)
        ax_sim.set_record(domain, run_id, 'RUNNING')
        commands = checks.setdefault('commands_sent', [])

        def log_command(what):
            commands.append({'command': what,
                             'after_launch_s': round(time.monotonic() - launch_started, 3)})
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Float64
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, DurabilityPolicy
        rclpy.init()
        node = rclpy.create_node('handoff_verifier')
        scans, static_frames = [], set()

        def scan_cb(msg):
            if len(scans) < 300:
                scans.append({'received_monotonic': time.monotonic(),
                              'frame_id': msg.header.frame_id, 'count': len(msg.ranges),
                              'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
                              'range_min': msg.range_min, 'range_max': msg.range_max,
                              'finite_ranges': sum(math.isfinite(v) for v in msg.ranges)})

        node.create_subscription(LaserScan, '/ax/robot_1/sensors/front_lidar/scan', scan_cb,
                                 qos_profile_sensor_data)
        node.create_subscription(
            TFMessage, '/tf_static',
            lambda m: static_frames.update(t.child_frame_id for t in m.transforms),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)

        # Gazebo merges the fixed support into base_link and the fixed upper mount
        # into column_link. Their fixed-origin rotations are identity.
        channels = [('column_yaw', 'column_yaw_joint', 'base_link', 'column_link', 0.3),
                    ('shoulder', 'shoulder_joint', 'column_link', 'upper_arm_link', -0.4),
                    ('elbow', 'elbow_joint', 'upper_arm_link', 'forearm_link', 0.5),
                    ('wrist', 'wrist_joint', 'forearm_link', 'wrist_link', 0.3),
                    ('gripper_rotation', 'gripper_rotation_joint', 'wrist_link',
                     'gripper_base_link', 0.3),
                    ('left_finger', 'left_finger_joint', 'gripper_base_link',
                     'left_finger_link', 0.015),
                    ('right_finger', 'right_finger_joint', 'gripper_base_link',
                     'right_finger_link', 0.015)]
        # ros2_control contract: arm joints via arm_controller (JointTrajectoryController),
        # primitive fingers via /gripper/command -> gripper_controller -> finger_controller.
        arm = [ch for ch in channels if 'finger' not in ch[0]]
        pubs = {'/arm_controller/joint_trajectory': node.create_publisher(
                    JointTrajectory, '/arm_controller/joint_trajectory', 10),
                '/gripper/command': node.create_publisher(Float64, '/gripper/command', 10)}

        def send_arm(targets, seconds):
            msg = JointTrajectory(joint_names=[j for _, j, *_ in arm])
            msg.points = [JointTrajectoryPoint(positions=[float(v) for v in targets],
                                               time_from_start=Duration(sec=seconds))]
            pubs['/arm_controller/joint_trajectory'].publish(msg)
            log_command(f'arm {list(targets)} in {seconds} s')

        def send_gripper(value):
            pubs['/gripper/command'].publish(Float64(data=value))
            log_command(f'gripper {value}')

        def query_control(prefix, deadline=None):
            """Query controller states and command interfaces; record both return codes."""
            def timeout():
                if deadline is None:
                    return QUERY_TIMEOUT_S
                return max(QUERY_MIN_TIMEOUT_S,
                           min(QUERY_TIMEOUT_S, deadline - time.monotonic()))
            rc_c, text_c = run(f'{prefix}controllers',
                               ['ros2', 'control', 'list_controllers'], timeout(),
                               merge=False)
            rc_h, text_h = run(f'{prefix}hardware_interfaces',
                               ['ros2', 'control', 'list_hardware_interfaces'], timeout(),
                               merge=False)
            return (evaluate_controllers(rc_c, parse_controllers(text_c)),
                    evaluate_interfaces(rc_h, parse_hardware_interfaces(text_h)))

        def probe(remaining_s):
            deadline = time.monotonic() + remaining_s
            spin(min(1.0, remaining_s))
            controllers, interfaces = query_control('ready_', deadline)
            return readiness_status(controllers, interfaces,
                                    {t: p.get_subscription_count() for t, p in pubs.items()})

        def motion_test():
            send_arm([0.0] * len(arm), 2)
            send_gripper(0.0)
            spin(5)
            # Each joint must have exactly one active command owner (claimed interface).
            checks['controllers'], checks['hardware_interfaces'] = query_control('')
            before = snapshot('poses_before')
            send_arm([target for *_, target in arm], 4)
            send_gripper(0.015)
            spin(8)
            after = snapshot('poses_after')
            checks['motion'] = measure_motion(channels, before, after)
            spin(3)

        def snapshot(name):
            rc, raw = run(name, ['gz', 'topic', '-e', '-n', '1', '--json-output',
                                 '-t', '/world/empty/pose/info'], 15)
            if rc:
                return {}
            try:
                obj, _ = json.JSONDecoder().raw_decode(raw[raw.index('{'):])
                return {p['name'].split('::')[-1]: p for p in obj.get('pose', [])}
            except (ValueError, KeyError):
                return {}

        checks['ready_wait_started_after_launch_s'] = round(
            time.monotonic() - launch_started, 3)
        readiness = start_when_ready(probe, launch.poll, motion_test,
                                     timeout_s=args.ready_timeout)
        checks['readiness'] = readiness
        if not readiness['ready']:
            # No command was sent: the motion test needs active controllers.
            raise RuntimeError(f'Simulation not ready: {readiness["reason"]}')
        elapsed = (scans[-1]['received_monotonic'] - scans[0]['received_monotonic']
                   if len(scans) > 1 else 0)
        checks['scan'] = {'received': len(scans),
                          'observed_hz': (len(scans)-1)/elapsed if elapsed else None,
                          'first': scans[0] if scans else None}
        checks['tf_static'] = sorted(static_frames)
        checks['ros_topics'] = node.get_topic_names_and_types()
        checks['ros_publisher_counts'] = {
            topic: node.count_publishers(topic)
            for topic in ('/joint_states', '/clock', '/odom',
                          '/ax/robot_1/sensors/front_lidar/scan')}
        run('gz_topics', ['gz', 'topic', '-l'])
        run('ros_nodes', ['ros2', 'node', 'list', '--no-daemon'])
        result['launch_alive_at_end'] = launch.poll() is None
    except Exception as e:
        result['error'] = repr(e)
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        if launch is not None:
            # Actual state before cleanup, also when the run stopped early (e.g. not ready).
            result['launch_returncode_before_cleanup'] = launch.poll()
            result.setdefault('launch_alive_at_end', launch.poll() is None)
        if sim is not None and (out / 'run.json').exists():
            # ax_sim.stop signals only this run's owned processes (SIGINT, SIGTERM, SIGKILL)
            # and releases the domain record once nothing owned is running.
            stop_code = sim.stop(out)
            result['cleanup'] = dict(sim.read_json(out / 'stop.json') or {},
                                     stop_returncode=stop_code)
        elif sim is not None and domain is not None:
            sim.set_record(domain, out.name, release=True)
        if launch is not None:
            try:
                result['launch_returncode'] = launch.wait(timeout=10)   # reap the root
            except subprocess.TimeoutExpired:
                result['launch_returncode'] = None
        if log is not None:
            log.close()
        code, failures = final_verdict(result)
        result['verdict'] = {'exit_code': code, 'failures': failures}
        write_result()
    for failure in failures:
        print('FAIL', failure, file=sys.stderr)
    return code


def measure_motion(channels, before, after):
    """Compare relative Gazebo link poses before and after the motion command."""
    motion = {}
    for c, j, parent, child, target in channels:
        if not all(n in poses for poses in (before, after) for n in (parent, child)):
            motion[j] = {'status': 'unverified', 'reason': 'parent/child poses unavailable'}
            continue
        qa, pa = relative(before, parent, child)
        qb, pb = relative(after, parent, child)
        dot = abs(sum(a*b for a, b in zip(qa, qb)))
        angle = 2*math.acos(min(1.0, dot))
        distance = math.sqrt(sum((a-b)**2 for a, b in zip(pa, pb)))
        moved = distance > 0.001 if 'finger' in c else angle > 0.01
        motion[j] = {'status': 'motion_observed' if moved else 'no_motion_observed',
                     'target': target, 'relative_angle_change_rad': angle,
                     'relative_position_change_m': distance,
                     'before_relative_position': pa, 'after_relative_position': pb,
                     'before_relative_quaternion_wxyz': qa, 'after_relative_quaternion_wxyz': qb}
        if 'finger' in c:
            actual = pb[1]-0.025 if c == 'left_finger' else -pb[1]-0.025
            unit, tolerance = 'm', 0.003
        else:
            component = 3 if c == 'column_yaw' else 1
            actual = 2*math.atan2(qb[component], qb[0])
            actual = (actual + math.pi) % (2*math.pi) - math.pi
            unit, tolerance = 'rad', 0.1
        motion[j].update({'measured_position': actual, 'unit': unit,
                          'absolute_target_error': abs(actual-target),
                          'review_tolerance': tolerance,
                          'within_review_tolerance': abs(actual-target) <= tolerance})
    return motion


if __name__ == '__main__':
    sys.exit(main())
