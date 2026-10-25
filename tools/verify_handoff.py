#!/usr/bin/env python3
"""기존 시뮬레이션을 격리 domain에서 실행해 인계 검증을 하고 원시 증거를 남긴다.

사용법: ROS·작업공간 환경을 source한 뒤
    python3 tools/verify_handoff.py [--dependencies-only] [--output-dir DIR] [--headless]
        [--domain-id N|auto] [--ready-timeout S]
로봇 모델 수정, 패키지 설치, 다른 시뮬레이션 종료를 하지 않는다.
domain은 tools/sim 할당 기록(ax_sim.py, AX_RUN_ROOT)에 등록하며, 다른 실행이 잡고 있거나
시작 시점에 다른 ROS 노드가 보이면 거부한다. 같은 할당 폴더를 쓰는 실행과 시작 시점에 보이는
노드에 대해서만 격리된다. launch는 기존 소유 프로세스 도구(own_procs.py, ax_sim.stop)로
추적·종료하며 프로세스 그룹 단위로 종료하지 않는다.
이 모듈을 import하면 파싱·판정 함수만 정의된다. 증거 폴더 생성, ROS 환경 설정, 시뮬레이션
실행은 main()에서만 일어난다.
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

# 기본 launch 구성: 단일 AX_1(robot_model:=ax1), primitive 집게, 루트 namespace.
# 이름은 src/ax_control/ax_control/model_contracts.py와 config/ax1_controllers.yaml을 따르며,
# tools/tests/test_verify_handoff.py가 둘이 같은지 확인한다.
REQUIRED_CONTROLLERS = ('joint_state_broadcaster', 'diff_drive_controller',
                        'arm_controller', 'finger_controller')
WHEEL_JOINTS = ('front_left_wheel_joint', 'rear_left_wheel_joint',
                'front_right_wheel_joint', 'rear_right_wheel_joint')
ARM_JOINTS = ('column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint',
              'gripper_rotation_joint')
FINGER_JOINTS = ('left_finger_joint', 'right_finger_joint')
# 바퀴는 속도(rad/s), 팔(rad)과 손가락(m)은 위치 명령을 받는다.
EXPECTED_COMMAND_INTERFACES = tuple(
    [f'{joint}/velocity' for joint in WHEEL_JOINTS]
    + [f'{joint}/position' for joint in ARM_JOINTS + FINGER_JOINTS])
CONTROLLER_STATES = ('active', 'inactive', 'unconfigured', 'finalized')
# 체인 가능한 Jazzy diff_drive_controller가 명령 인터페이스 목록에 함께 내보내는 참조
# 인터페이스(격리 실행에서 확인). 하드웨어 인터페이스와 따로 구분하는 것은 이 두 개뿐이다.
CONTROLLER_REFERENCE_INTERFACES = ('diff_drive_controller/linear/velocity',
                                   'diff_drive_controller/angular/velocity')

# 전체 시작 준비 제한시간. launch 시작 직후부터 센다. 각 준비 조회의 spin과 CLI 조회는 남은
# 시간 안으로 줄이며(조회 하나는 최소 QUERY_MIN_TIMEOUT_S), 제한시간과 같거나 넘은 시점에
# 끝난 조회는 준비 완료로 인정하지 않는다.
# launch는 로봇마다 spawner 2개를 --controller-manager-timeout 120, --switch-timeout 60으로
# 실행하지만 두 값을 단순히 더하지 않는다.
# - Jazzy spawner는 파일 잠금(~/.ros/locks, 이 PC 사용자의 모든 spawner가 공유)으로 순서대로
#   실행된다. 기다리는 spawner는 5회 × (잠금 대기 20 s + 3 s) 재시도 후 종료하므로, 첫
#   spawner가 약 115 s 안에 잠금을 놓지 않으면 두 번째 spawner가 실패한다. 따라서 성공하는
#   시작이라면 120 s 관리자 대기(Gazebo 시작, 모델 생성, 플러그인 로드)도 이 구간 안에 끝난다.
# - 두 번째로 잠금을 잡은 spawner는 혼자 동작한다. 주 spawner는 제어기 3개를 다루며, 각각
#   서비스 호출 최대 3회 × 10 s(--service-call-timeout 기본값)와 전환 최대 60 s가 걸려
#   3 × (30 + 60) = 270 s다. 주행 spawner는 90 s다.
# 성공하는 시작의 최악값은 115 + 270 = 385 s이고, 35 s 여유는 명령 구독자 발견과 조회 자체의
# 시간을 덮는다. 이 시간을 넘기면 시작 실패로 처리한다.
READY_TIMEOUT_S = 420.0
READY_POLL_S = 5.0
QUERY_TIMEOUT_S = 20.0
QUERY_MIN_TIMEOUT_S = 0.5

# ROS 패키지가 아닌 rosdep 키와 그 확인 방법(import할 모듈). 그 밖의 선언 의존성은 ament
# index에서 찾아야 하며, 알 수 없는 키는 실패로 보고한다.
SYSTEM_DEPENDENCIES = {
    # simulation.launch.py는 `ros2 launch`를 실행하는 Python 안에서 yaml을 import한다.
    'python3-yaml': 'yaml',
}

ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;?]*[ -/]*[@-~]')
COMMAND_INTERFACE_LINE = re.compile(
    r'^(?P<name>\S+)(?: \[[^\]]+\])? \[(?P<available>available|unavailable)\]'
    r' \[(?P<claimed>claimed|unclaimed)\]$')


# ros2controlcli 출력에 붙는 터미널 색상 코드를 제거한다.
def strip_ansi(text):
    return ANSI_ESCAPE.sub('', text or '')


# `ros2 control list_controllers` 출력을 {이름: {type, state}}와 파싱 오류 목록으로 바꾼다.
def parse_controllers(text):
    controllers, errors = {}, []
    for raw in strip_ansi(text).splitlines():
        line = raw.strip()
        if not line or line == 'No controllers are currently loaded!':
            continue
        fields = line.split()
        # 각 행은 "<이름> <형식> <상태>"다. 그 밖의 형식은 상태로 믿지 않는다.
        if len(fields) != 3 or fields[2] not in CONTROLLER_STATES:
            errors.append(f'unparsed line: {line}')
        elif fields[0] in controllers:
            errors.append(f'duplicate controller: {fields[0]}')
        else:
            controllers[fields[0]] = {'type': fields[1], 'state': fields[2]}
    return {'controllers': controllers, 'errors': errors}


# 조회 종료 코드와 파싱 결과로 필수 제어기가 모두 목록에 있고 정확히 'active'인지 판정한다.
def evaluate_controllers(returncode, parsed, required=REQUIRED_CONTROLLERS):
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


# `ros2 control list_hardware_interfaces` 출력에서 명령 인터페이스의 사용 가능·점유 상태를 읽는다.
def parse_hardware_interfaces(text):
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


# 하드웨어 명령 인터페이스가 예상 목록과 정확히 같고 모두 사용 가능·점유 상태인지 판정한다.
# 지정한 제어기 참조 인터페이스만 따로 분리하며, 제어기 이름 접두사가 같아도 그 밖의 이름은
# 예상 밖 인터페이스다. 제어기 체인을 구성하지 않았으므로 점유된 참조 인터페이스는 실패다.
def evaluate_interfaces(returncode, parsed, expected=EXPECTED_COMMAND_INTERFACES,
                        references=CONTROLLER_REFERENCE_INTERFACES):
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


# 제어기 활성, 인터페이스 점유, 명령 토픽별 구독자가 모두 확인될 때만 준비 완료로 판정한다.
def readiness_status(controllers, interfaces, subscriber_counts):
    reasons = list(controllers['problems']) + list(interfaces['problems'])
    if not subscriber_counts:
        reasons.append('no command topics checked')
    reasons += [f'no subscriber on {topic}'
                for topic, count in sorted(subscriber_counts.items()) if not count]
    return {'ready': not reasons, 'reasons': reasons,
            'controllers_returncode': controllers['returncode'],
            'interfaces_returncode': interfaces['returncode'],
            'subscriber_counts': dict(subscriber_counts)}


# 준비 완료, launch 종료, 제한시간 초과 중 하나가 될 때까지 probe(남은 시간)를 반복 호출한다.
# probe()는 readiness_status() 결과를 돌려주고 자기 대기를 남은 시간 안으로 줄여야 한다.
# launch_poll()은 launch가 실행 중이면 None을 돌려준다.
def wait_until_ready(probe, launch_poll, timeout_s=READY_TIMEOUT_S, poll_s=READY_POLL_S,
                     clock=time.monotonic, sleep=time.sleep):
    start = clock()
    deadline = start + timeout_s
    attempts = []

    # 대기 결과(준비 여부·사유·경과 시간·조회 기록)를 만든다.
    def result(ready, reason):
        return {'ready': ready, 'reason': reason, 'timeout_s': timeout_s,
                'elapsed_s': clock() - start, 'attempts': attempts}

    # 제한시간 초과 결과를 마지막 미준비 사유와 함께 만든다.
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

        # 준비 결과보다 제한시간을 먼저 본다. 제한시간과 같거나 넘은 시점에 끝난 조회(조회
        # 도중 제한시간을 넘긴 경우 포함)는 명령 전송으로 이어지지 않는다.
        if elapsed >= timeout_s:
            return timed_out(['ready only at or after the limit'] if status['ready']
                             else status['reasons'])
        if status['ready']:
            return result(True, None)
        reasons = status['reasons']
        sleep(max(0.0, min(poll_s, deadline - clock())))


# 준비 완료가 확인된 뒤에만 on_ready()(초기 명령과 동작 시험)를 호출한다.
def start_when_ready(probe, launch_poll, on_ready, **wait_options):
    readiness = wait_until_ready(probe, launch_poll, **wait_options)
    if readiness['ready']:
        on_ready()
    return readiness


# 결과 전체로 (종료 코드, 실패 사유)를 정한다. 0 통과, 1 준비·조회 실패 또는 미완료,
# 2 동작은 확인했으나 목표 오차 초과.
def final_verdict(result):
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
    if checks.get('runtime_joint_limits', {}).get('matches_checked_urdf') is not True:
        failures.append('runtime robot_description limits not confirmed equal to the checked URDF')

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

    # 동작 확인과 위치 정확도를 종료 코드로 구분한다. 코드 2에는 관절별 오차 사유를 남긴다.
    misses = [f'{joint} outside review tolerance: target={v.get("target")}'
              f' measured={v.get("measured_position")} error={v.get("absolute_target_error")}'
              f' tolerance={v.get("review_tolerance")} {v.get("unit", "")}'.rstrip()
              for joint, v in motion.items() if v.get('within_review_tolerance') is not True]
    return (2, misses) if misses else (0, [])


# 작업공간 package.xml들에서 {패키지: [의존성 이름]}을 읽는다.
def declared_dependencies(src=ROOT / 'src'):
    result = {}
    for manifest in sorted(Path(src).glob('*/package.xml')):
        package = ET.parse(manifest).getroot()
        result[package.findtext('name')] = sorted(
            {d.text.strip() for tag in ('depend', 'exec_depend', 'buildtool_depend')
             for d in package.findall(tag)})
    return result


# source된 ament index로 패키지 이름 → 설치 경로(없으면 None)를 찾는 함수를 만든다.
def ament_prefix_lookup():
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError

    # 패키지 하나의 설치 경로를 찾고, 없으면 None을 돌려준다.
    def lookup(name):
        try:
            return get_package_prefix(name)
        except PackageNotFoundError:
            return None
    return lookup


# `ros2` 실행 파일의 shebang에서 launch 파일을 실행하는 Python과 확인 방법을 찾는다.
def launch_python(which=shutil.which):
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


# 지정한 Python이 모듈을 import할 수 있는지 격리 모드(-I)로 확인한다.
def check_python_module(python, module, runner=None):
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


# 선언된 의존성을 시스템·ROS 패키지로 나눠 모두 확인한다. 확인하지 않은 항목은 통과시키지 않는다.
def check_dependencies(declared, lookup, python, runner=None):
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


# 의존성 확인 결과가 모두 통과인지 돌려준다.
def dependencies_ok(dependencies):
    return all(entry['ok'] for deps in dependencies.values() for entry in deps.values())


# Gazebo 자세 JSON에서 쿼터니언을 [w, x, y, z]로 꺼낸다(없으면 단위 쿼터니언).
def quat(p):
    q = p.get('orientation', {})
    return [float(q.get(k, 0 if k != 'w' else 1)) for k in ('w', 'x', 'y', 'z')]


# 두 쿼터니언 [w, x, y, z]의 곱을 계산한다.
def mul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


# 부모 링크 기준 자식 링크의 상대 회전(쿼터니언)과 상대 위치를 계산한다.
def relative(poses, parent, child):
    a, b = poses[parent], poses[child]
    qa = quat(a)
    inv = [qa[0], -qa[1], -qa[2], -qa[3]]
    delta = [float(b.get('position', {}).get(k, 0))-float(a.get('position', {}).get(k, 0))
             for k in ('x', 'y', 'z')]
    return mul(inv, quat(b)), mul(mul(inv, [0]+delta), qa)[1:]


# URDF XML 문자열에서 팔 관절별 limit(lower·upper·effort·velocity)을 읽는다. 없는 관절은 None이다.
def arm_joint_limits(urdf_text):
    root = ET.fromstring(urdf_text)
    result = {}
    for name in ARM_JOINTS:
        limit = root.find(f"joint[@name='{name}']/limit")
        result[name] = None if limit is None else {
            key: float(limit.get(key)) for key in ('lower', 'upper', 'effort', 'velocity')
            if limit.get(key) is not None}
    return result


# `ros2 param get` 출력("String value is: ...")에서 문자열 값만 꺼낸다. 형식이 다르면 None이다.
def parameter_string(text):
    marker = 'String value is: '
    start = (text or '').find(marker)
    return None if start < 0 else text[start + len(marker):].strip()


# 시뮬레이션 launch 명령을 만든다. headless가 아니면 launch 기본값(ax1, GUI)을 그대로 쓴다.
def launch_command(headless=False):
    command = ['ros2', 'launch', 'ax_control', 'simulation.launch.py']
    if headless:
        command.append('gui:=false')
    return command


# --domain-id 값을 해석한다. 'auto'는 ax_sim.py의 60~99 범위 자동 할당(None), 그 밖에는 0~232.
def domain_argument(value):
    if value == 'auto':
        return None
    try:
        domain = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError(f'not an integer or auto: {value}')
    if not 0 <= domain <= 232:
        raise argparse.ArgumentTypeError(f'ROS_DOMAIN_ID out of range 0-232: {domain}')
    return domain


# 0보다 큰 초 단위 값만 받는다(--ready-timeout).
def positive_seconds(value):
    seconds = float(value)
    if not seconds > 0:
        raise argparse.ArgumentTypeError(f'must be > 0: {value}')
    return seconds


# 명령행 인자를 해석한다.
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


# 의존성 확인 후 격리 domain에서 launch를 시작하고, 준비 확인 → 초기 명령 → 동작 시험 →
# 소유 프로세스 정리 → 최종 판정까지 수행해 종료 코드를 돌려준다.
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

    # 명령을 실행하고 출력을 증거 폴더에 저장한다. merge=False면 stderr를 따로 저장하고
    # stdout만 돌려준다.
    def run(name, command, timeout=20, merge=True):
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

    # results.json을 저장하고 같은 내용을 표준 출력에 낸다.
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
        # 기존 tools/sim 도구로 domain 할당 기록과 소유 프로세스 정리를 한다.
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

        # 이미 참여자가 있는 domain은 그래프가 섞이지 않도록 거부한다.
        rc, text = run('domain_graph_before_start',
                       [sys.executable, str(SIM_TOOLS / 'comm_probe.py'), '--graph-only'], 30)
        try:
            others = json.loads(text.strip().splitlines()[-1])['other_nodes']
        except (ValueError, IndexError, KeyError):
            others = None

        # ros2cli daemon은 그래프를 캐시할 뿐 시뮬레이션 토픽에 참여하지 않으므로 기록만 한다.
        daemons = [n for n in others or [] if n.startswith('/_ros2cli_daemon_')]
        others = None if others is None else [n for n in others if n not in daemons]

        checks['domain_graph_before_start'] = {'returncode': rc, 'other_nodes': others,
                                               'ignored_ros2cli_daemons': daemons}
        if rc or others is None:
            raise RuntimeError('cannot check ROS nodes on the domain before start')
        if others:
            raise RuntimeError(f'ROS_DOMAIN_ID {domain} already has nodes: {others}')

        # launch가 실제로 읽는 설치본(ament index의 ax_description share)을 검사한다.
        # symlink 설치면 src 파일과 같고, 검증용 사본 설치를 source하면 그 사본이다.
        from ament_index_python.packages import get_package_share_directory
        xacro_file = Path(get_package_share_directory('ax_description')) / 'urdf/ax.urdf.xacro'
        checks['description_source'] = {'xacro': str(xacro_file),
                                        'resolved': str(xacro_file.resolve())}

        rc, _ = run('xacro', ['xacro', str(xacro_file), '-o', str(out / 'ax.urdf')])
        checks['xacro'] = {'returncode': rc}
        if rc:
            raise RuntimeError('Xacro conversion failed')

        rc, _ = run('check_urdf', ['check_urdf', str(out / 'ax.urdf')])
        checks['check_urdf'] = {'returncode': rc}

        tree = ET.parse(out / 'ax.urdf').getroot()
        checked_limits = arm_joint_limits((out / 'ax.urdf').read_text(encoding='utf-8'))
        result['links'] = len(tree.findall('link'))
        result['joints'] = len(tree.findall('joint'))

        log = (out / 'simulation.log').open('w', encoding='utf-8')
        helper = [sys.executable, str(SIM_TOOLS / 'own_procs.py')]
        subprocess.run(helper + ['init', str(out / 'own')], check=True)

        # 여기부터 모든 자식(launch, ros2 control CLI와 그것이 띄울 수 있는 ros2 daemon)이
        # 토큰을 가지므로, 이 실행이 띄운 daemon도 ax_sim.stop이 함께 정리한다.
        os.environ['AX_OWN_TOKEN'] = token
        command = launch_command(args.headless)
        checks['launch_command'] = command

        # 위에서 설정한 AX_OWN_TOKEN이 분리된 자손까지 포함해 launch의 모든 자손을 표시한다.
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

        # 보낸 명령과 launch 시작 후 경과 시간을 기록한다.
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

        # 전방 LiDAR 메시지의 수신 시각과 요약을 최대 300개까지 기록한다.
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

        # 지정한 시간 동안 검증 노드의 콜백을 처리한다.
        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)

        # Gazebo는 고정 받침을 base_link에, 고정 상단 장착부를 column_link에 합친다.
        # 두 고정 관절 원점의 회전은 단위 회전이다.
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

        # ros2_control 계약: 팔 관절은 arm_controller(JointTrajectoryController)로, primitive
        # 손가락은 /gripper/command → gripper_controller → finger_controller로 명령한다.
        arm = [ch for ch in channels if 'finger' not in ch[0]]
        pubs = {'/arm_controller/joint_trajectory': node.create_publisher(
                    JointTrajectory, '/arm_controller/joint_trajectory', 10),
                '/gripper/command': node.create_publisher(Float64, '/gripper/command', 10)}

        # 팔 5축 목표를 궤적 점 하나로 보내고 전송을 기록한다.
        def send_arm(targets, seconds):
            msg = JointTrajectory(joint_names=[j for _, j, *_ in arm])
            msg.points = [JointTrajectoryPoint(positions=[float(v) for v in targets],
                                               time_from_start=Duration(sec=seconds))]
            pubs['/arm_controller/joint_trajectory'].publish(msg)
            log_command(f'arm {list(targets)} in {seconds} s')

        # 손가락 목표 변위(m)를 보내고 전송을 기록한다.
        def send_gripper(value):
            pubs['/gripper/command'].publish(Float64(data=value))
            log_command(f'gripper {value}')

        # 제어기 상태와 명령 인터페이스를 조회해 판정하고, 두 조회의 종료 코드를 기록한다.
        def query_control(prefix, deadline=None):
            # 조회 하나의 제한시간을 남은 시간 안으로 줄인다(최소 QUERY_MIN_TIMEOUT_S).
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

        # 남은 시간 안에서 구독자 발견을 기다리고 제어기·인터페이스를 조회해 준비 상태를 만든다.
        def probe(remaining_s):
            deadline = time.monotonic() + remaining_s
            spin(min(1.0, remaining_s))
            controllers, interfaces = query_control('ready_', deadline)
            return readiness_status(controllers, interfaces,
                                    {t: p.get_subscription_count() for t, p in pubs.items()})

        # 준비 완료 뒤 초기 명령을 보내고, 제어기 상태를 다시 확인한 다음 목표 자세 전후의
        # Gazebo 링크 자세로 관절 동작을 측정한다.
        def motion_test():
            send_arm([0.0] * len(arm), 2)
            send_gripper(0.0)
            spin(5)

            # 관절마다 명령 주체(점유된 인터페이스)가 정확히 하나여야 한다.
            checks['controllers'], checks['hardware_interfaces'] = query_control('')
            before = snapshot('poses_before')

            send_arm([target for *_, target in arm], 4)
            send_gripper(0.015)
            spin(8)
            after = snapshot('poses_after')
            checks['motion'] = measure_motion(channels, before, after)
            spin(3)

        # Gazebo 월드의 링크 자세를 한 번 받아 {링크 이름: 자세}로 돌려준다(실패 시 빈 사전).
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
            # 명령은 보내지 않았다. 동작 시험에는 활성 제어기가 필요하다.
            raise RuntimeError(f'Simulation not ready: {readiness["reason"]}')

        # 실행 중인 모델(robot_state_publisher의 robot_description)의 팔 관절 제한을 기록하고,
        # 위에서 검사한 URDF와 같은지 확인한다.
        rc, text = run('runtime_robot_description',
                       ['ros2', 'param', 'get', '/robot_state_publisher', 'robot_description'],
                       QUERY_TIMEOUT_S, merge=False)

        description = parameter_string(text)
        try:
            runtime_limits = arm_joint_limits(description) if description else None
        except ET.ParseError:
            runtime_limits = None

        checks['runtime_joint_limits'] = {
            'returncode': rc, 'limits': runtime_limits, 'checked_urdf_limits': checked_limits,
            'matches_checked_urdf': rc == 0 and runtime_limits == checked_limits}

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
            # 정리 직전의 실제 상태를 기록한다. 준비 실패 등으로 일찍 멈춘 경우도 포함한다.
            result['launch_returncode_before_cleanup'] = launch.poll()
            result.setdefault('launch_alive_at_end', launch.poll() is None)

        if sim is not None and (out / 'run.json').exists():
            # ax_sim.stop은 이 실행의 소유 프로세스에만 SIGINT·SIGTERM·SIGKILL을 보내고,
            # 실행 중인 소유 프로세스가 없으면 domain 기록을 해제한다.
            stop_code = sim.stop(out)
            result['cleanup'] = dict(sim.read_json(out / 'stop.json') or {},
                                     stop_returncode=stop_code)
        elif sim is not None and domain is not None:
            sim.set_record(domain, out.name, release=True)

        if launch is not None:
            try:
                result['launch_returncode'] = launch.wait(timeout=10)   # launch 루트 프로세스 회수
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


# 동작 명령 전후의 Gazebo 상대 링크 자세를 비교해 관절별 동작 여부와 목표 오차를 계산한다.
def measure_motion(channels, before, after):
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
