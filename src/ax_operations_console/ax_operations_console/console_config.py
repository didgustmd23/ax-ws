"""관제 화면 설정 읽기·검사(ROS 없음).

- 로봇 목록 파일(YAML): source(현재 dev_config만)와 robots[robot_id·role·spare]
- Topic 파일(YAML): profiles.<이름>.tasks와 robot_topics(로봇별 상대 이름). 개발 profile만 /ax/dev/ 작업 Topic을 쓴다.
- 바인딩 주소·포트·판정 수치 검사
"""
import ipaddress
from pathlib import Path
import re

import yaml

ROBOT_ID = re.compile(r'^robot_[0-9]+$')
TOPIC = re.compile(r'^/[a-z0-9_]+(/[a-z0-9_]+)*$')
RELATIVE = re.compile(r'^[a-z0-9_]+(/[a-z0-9_]+)*$')
ROLES = ('MANIPULATOR', 'TRANSPORTER')
ROBOT_SOURCES = ('dev_config',)
REQUIRED_ROBOT_TOPICS = ('actuation', 'safety', 'health', 'executor', 'pose', 'battery')
MAX_ROBOTS = 16
MAX_FILE_BYTES = 64 * 1024


class ConfigError(ValueError):
    """설정 거부."""


# 크기 제한 안에서 YAML 읽기
def read_yaml(path):
    data_path = Path(path)

    if not data_path.is_file():
        raise ConfigError(f'설정 파일 없음: {path}')

    if data_path.stat().st_size > MAX_FILE_BYTES:
        raise ConfigError(f'설정 파일이 너무 큼: {path}')

    data = yaml.safe_load(data_path.read_text(encoding='utf-8'))

    if not isinstance(data, dict):
        raise ConfigError(f'최상위가 사전이 아님: {path}')

    return data


# 로봇 목록 검사 → (source, robots)
def load_robots(path):
    data = read_yaml(path)
    source = data.get('source')

    if source not in ROBOT_SOURCES:
        raise ConfigError(f'로봇 목록 출처는 {ROBOT_SOURCES} 중 하나: {source!r}')

    robots = data.get('robots')
    if not isinstance(robots, list) or not 1 <= len(robots) <= MAX_ROBOTS:
        raise ConfigError(f'robots는 1~{MAX_ROBOTS}개 목록이어야 함')

    seen, out = set(), []

    for entry in robots:
        if not isinstance(entry, dict) or set(entry) - {'robot_id', 'role', 'spare'}:
            raise ConfigError(f'로봇 항목 형식 오류: {entry!r}')

        robot_id, role, spare = entry.get('robot_id'), entry.get('role'), entry.get('spare', False)

        if type(robot_id) is not str or not ROBOT_ID.match(robot_id):
            raise ConfigError(f'robot_id 형식 오류: {robot_id!r}')
        if robot_id in seen:
            raise ConfigError(f'robot_id 중복: {robot_id}')
        if role not in ROLES:
            raise ConfigError(f'{robot_id} 역할은 {ROLES} 중 하나: {role!r}')
        if type(spare) is not bool:
            raise ConfigError(f'{robot_id} spare는 true/false: {spare!r}')

        seen.add(robot_id)
        out.append({'robot_id': robot_id, 'role': role, 'spare': spare})

    return source, out


# Topic profile 검사 → {'tasks': 절대 이름, 'robot': {kind: 상대 이름}}
def load_topics(path, profile):
    data = read_yaml(path)
    profiles = data.get('profiles')

    if not isinstance(profiles, dict) or profile not in profiles:
        raise ConfigError(f'Topic profile 없음: {profile!r}')

    entry = profiles[profile]
    tasks = entry.get('tasks') if isinstance(entry, dict) else None
    robot = data.get('robot_topics')

    if type(tasks) is not str or not TOPIC.match(tasks):
        raise ConfigError(f'{profile}.tasks 형식 오류: {tasks!r}')

    # 운영 profile은 개발 이름공간을, 개발 profile은 운영 작업 Topic을 쓰지 않는다
    if profile == 'operational' and tasks.startswith('/ax/dev/'):
        raise ConfigError('운영 profile에 /ax/dev/ Topic 사용 불가')
    if profile != 'operational' and not tasks.startswith('/ax/dev/'):
        raise ConfigError(f'개발 profile({profile})의 작업 Topic은 /ax/dev/ 아래여야 함')

    if not isinstance(robot, dict) or set(robot) != set(REQUIRED_ROBOT_TOPICS):
        raise ConfigError(f'robot_topics는 {REQUIRED_ROBOT_TOPICS}를 정확히 가져야 함')

    for kind, name in robot.items():
        if type(name) is not str or not RELATIVE.match(name):
            raise ConfigError(f'robot_topics.{kind} 상대 이름 형식 오류: {name!r}')

    return {'tasks': tasks, 'robot': dict(robot)}


# 로봇 Topic 절대 이름
def robot_topic(robot_id, relative):
    return f'/ax/{robot_id}/{relative}'


# 바인딩 주소 검사 → 외부 노출 여부
def check_bind(address, port):
    try:
        ip = ipaddress.ip_address(address)
    except ValueError as error:
        raise ConfigError(f'bind_address는 IP 주소여야 함: {address!r}') from error

    if type(port) is not int or not 1024 <= port <= 65535:
        raise ConfigError(f'port는 1024~65535: {port!r}')

    if port in (8765, 9090):
        raise ConfigError(f'port {port}는 다른 도구(tandem 관제·rosbridge 기본)와 겹쳐 쓰지 않음')

    return not ip.is_loopback


# 판정 수치 검사
def check_timing(future_tolerance_s, display_grace_s):
    if not 0.0 <= future_tolerance_s <= 1.0:
        raise ConfigError(f'future_tolerance_s는 0~1.0: {future_tolerance_s}')

    if not 0.0 <= display_grace_s <= 2.0:
        raise ConfigError(f'display_grace_s는 0~2.0: {display_grace_s}')


# P5 PGM 머리(폭·높이·최댓값) 읽기
def pgm_size(data):
    tokens, pos = [], 0

    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1

        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] != b'\n':
                pos += 1
            continue

        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1

        if start == pos:
            raise ConfigError('PGM 머리 불완전')
        tokens.append(data[start:pos])

    if tokens[0] != b'P5' or int(tokens[3]) != 255:
        raise ConfigError('지도는 P5 8비트 PGM만 지원')

    width, height = int(tokens[1]), int(tokens[2])
    if len(data) < pos + 1 + width * height:
        raise ConfigError('PGM 본문 길이 부족')

    return width, height


# 지도 YAML(map_server 형식) → (화면용 info, PGM 바이트)
def load_map(map_yaml):
    meta = read_yaml(map_yaml)
    image = Path(map_yaml).parent / str(meta.get('image', ''))

    if not image.is_file() or image.suffix != '.pgm':
        raise ConfigError(f'지도 이미지가 PGM 파일이 아님: {image}')

    data = image.read_bytes()
    width, height = pgm_size(data)
    origin = meta.get('origin')

    if not isinstance(origin, list) or len(origin) < 2:
        raise ConfigError('지도 origin 형식 오류')

    info = {'resolution': float(meta['resolution']),
            'origin': [float(origin[0]), float(origin[1])],
            'width': width, 'height': height, 'name': Path(map_yaml).parent.name}
    return info, data
