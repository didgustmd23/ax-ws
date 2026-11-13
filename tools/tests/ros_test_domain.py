# ROS 통신 시험이 쓰는 ROS_DOMAIN_ID를 시험별 전용 범위에서 잡고, 같은 이름 노드가 이미 있는지 확인하는 공용 도우미.
# 팀 격리 시뮬레이션(141~231)과 같은 domain을 쓰면 같은 이름 노드(/map_server, /ax/<robot>/amcl 등)와 섞여
# 시험 판정이 뒤틀리고 상대 실행에도 명령이 갈 수 있다. 그래서 시험은 아래 배정 범위만 쓴다.
#
# 범위 배정(겹치지 않음)
#   test_localization_startup           101~110
#   test_localization_supervisor        111~125
#   test_b3_corridor_observer_ros       126~133
#   test_b2_health_raw·test_mapping_session  134~137
#   test_a3_local_task_executor·test_a3_lease_check·test_a3_mock_robots  138~140
#
# 사용 예
#   domain, lock = reserve_domain(SUPERVISOR_DOMAINS)      # 잠금은 시험이 끝날 때 release(lock)
#   names = conflicting_nodes(domain, SUPERVISOR_NAMES)    # 비어 있지 않으면 conflict_message로 실패
#   services = conflicting_services(domain, r'^/ax/[^/]+/executor/')  # 계약 경로 서비스·Action이 이미 있는지
import fcntl
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

# 팀 격리 시뮬레이션 범위. 시험 범위는 이 범위와 겹치면 안 된다.
TEAM_ISOLATION = range(141, 232)

# 시험별 전용 범위
STARTUP_DOMAINS = range(101, 111)
SUPERVISOR_DOMAINS = range(111, 126)
CORRIDOR_OBSERVER_DOMAINS = range(126, 134)
HEALTH_AND_SESSION_DOMAINS = range(134, 138)
A3_DOMAINS = range(138, 141)

# 같은 PC의 시험 실행끼리 겹치지 않게 하는 잠금 파일 폴더(domain마다 파일 하나)
LOCK_DIR = Path(tempfile.gettempdir()) / 'ax_test_domains'

# 그래프 검사: 지정 domain에서 이름 패턴에 맞는 노드(mode nodes) 또는 서비스(mode services, Action 내부 서비스 포함)를
# 최대 wait_s 동안 찾는다(로컬 탐색에 0.2~2.7 s 걸린 사례가 있음). 찾으면 바로 끝내고 한 줄에 하나씩 출력한다.
GRAPH_PROBE = '''
import re
import sys
import time
import rclpy

pattern = re.compile(sys.argv[1])
wait_s = float(sys.argv[2])
mode = sys.argv[3]
rclpy.init()
node = rclpy.create_node("ax_test_domain_probe")
end = time.monotonic() + wait_s
found = []

while time.monotonic() < end and not found:
    rclpy.spin_once(node, timeout_sec=0.05)

    if mode == "services":
        names = [name for name, _ in node.get_service_names_and_types()]
    else:
        names = [namespace.rstrip("/") + "/" + name for name, namespace in node.get_node_names_and_namespaces()]

    found = [name for name in names if pattern.match(name)]

print("\\n".join(found))
node.destroy_node()
rclpy.shutdown()
'''


# 환경변수 항목(KEY=VALUE)을 가진 살아 있는 프로세스 pid 목록
def processes_with(item):
    found = []

    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue

        try:
            environ = (entry / 'environ').read_bytes().split(b'\0')
        except OSError:
            continue

        if item.encode() in environ:
            found.append(int(entry.name))

    return found


# AX_TEST_DOMAIN_LOG가 지정되면 예약·해제를 한 줄씩 남긴다(동시 실행 겹침 확인용, 시험 동작은 바꾸지 않음).
# 형식: 벽시계 s, 사건(reserve·release), domain, pid, AX_RUN_TAG
def log_event(event, domain):
    path = os.environ.get('AX_TEST_DOMAIN_LOG')

    if not path:
        return

    with open(path, 'a') as handle:
        handle.write(f'{time.time():.3f}\t{event}\t{domain}\t{os.getpid()}\t{os.environ.get("AX_RUN_TAG", "-")}\n')


# 이 domain을 쓰는 프로세스 중 exclude(pid 모음)에 없는 것. 자기 프로세스는 항상 뺀다.
def other_domain_users(domain, exclude=()):
    own = set(exclude) | {os.getpid()}

    return sorted(set(processes_with(f'ROS_DOMAIN_ID={domain}')) - own)


# 범위에서 domain 하나를 잡는다. 잠금 파일(fcntl)로 동시 실행끼리 겹치지 않게 하고, 그 ROS_DOMAIN_ID를 쓰는
# 다른 프로세스가 있으면 다음 값으로 넘어간다. 시작 위치는 pid로 흩는다. (domain, 잠금 파일) 또는 RuntimeError.
def reserve_domain(domains):
    domains = list(domains)

    if set(domains) & set(TEAM_ISOLATION):
        raise ValueError(f'시험 domain 범위가 팀 격리 범위 {TEAM_ISOLATION.start}~{TEAM_ISOLATION.stop - 1}와 겹침')

    LOCK_DIR.mkdir(parents=True, exist_ok=True)
    start = os.getpid() % len(domains)

    for domain in domains[start:] + domains[:start]:
        handle = open(LOCK_DIR / f'{domain}.lock', 'w')

        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.close()
            continue

        if other_domain_users(domain):
            handle.close()
            continue

        log_event('reserve', domain)

        return domain, handle

    raise RuntimeError(f'시험 domain {domains[0]}~{domains[-1]}이 모두 사용 중')


# reserve_domain이 돌려준 잠금을 푼다(None이면 아무것도 하지 않음).
def release(lock):
    if lock is not None and not lock.closed:
        log_event('release', Path(lock.name).stem)
        lock.close()


# 지정 domain에서 그래프를 탐색해 패턴에 맞는 이름을 돌려준다(mode: nodes 또는 services).
def probe_graph(domain, pattern, wait_s, mode):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    output = subprocess.run([sys.executable, '-c', GRAPH_PROBE, pattern, str(wait_s), mode], env=env,
                            capture_output=True, text=True, timeout=wait_s + 30).stdout

    return sorted(set(output.split()))


# 지정 domain에서 보이는 노드 중 이름 패턴(정규식 문자열)에 맞는 것을 돌려준다.
def conflicting_nodes(domain, pattern, wait_s=5.0):
    return probe_graph(domain, pattern, wait_s, 'nodes')


# 지정 domain에서 보이는 서비스(Action 내부 서비스 포함) 중 이름 패턴에 맞는 것을 돌려준다.
def conflicting_services(domain, pattern, wait_s=5.0):
    return probe_graph(domain, pattern, wait_s, 'services')


# 같은 이름 노드·서비스가 있을 때의 실패 메시지
def conflict_message(domain, names, kind='노드'):
    return f'domain {domain} 사용 중: 같은 이름 {kind} {", ".join(names)}가 이미 있음. 다른 실행과 섞이므로 시험하지 않음'


# 모듈 전체(프로세스 안 rclpy.init과 하위 프로세스)가 쓸 domain을 잡아 os.environ에 넣는다(rclpy.init은 init 시점의
# 환경변수를 읽는다). service_pattern이 있으면 그 경로의 서비스·Action이 이미 있는지 먼저 보고, 있으면 잠금을 풀고
# 원인이 보이는 실패(AssertionError)로 끝낸다. 돌려준 상태는 leave_module_domain으로 되돌린다.
def enter_module_domain(domains, service_pattern=None):
    domain, lock = reserve_domain(domains)

    if service_pattern:
        found = conflicting_services(domain, service_pattern)

        if found:
            release(lock)
            raise AssertionError(conflict_message(domain, found, '서비스'))

    keys = ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')
    previous = {key: os.environ.get(key) for key in keys}
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')

    return {'domain': domain, 'lock': lock, 'previous': previous}


# enter_module_domain 이전의 환경변수로 되돌리고 잠금을 푼다.
def leave_module_domain(state):
    if state is None:
        return

    for key, value in state['previous'].items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value

    release(state['lock'])


# 범위 사이 겹침이 없는지 확인한다(모듈 단위 시험용).
def assigned_ranges():
    return {
        'test_localization_startup': STARTUP_DOMAINS,
        'test_localization_supervisor': SUPERVISOR_DOMAINS,
        'test_b3_corridor_observer_ros': CORRIDOR_OBSERVER_DOMAINS,
        'test_b2_health_raw·test_mapping_session': HEALTH_AND_SESSION_DOMAINS,
        'test_a3_*': A3_DOMAINS,
    }
