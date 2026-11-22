"""개발 상태 출력 노드의 설정·입력 검사(ROS 없음).

- 출력 Topic은 /ax/dev/ 아래만 허용한다. /ax/fleet·/ax/robot_ 운영 이름공간에는 내지 않는다.
- 입력은 개발 fixture JSON 또는 D2 원장(SQLite, 읽기 전용)이다. 원장 상태는 복원 규칙
  (RESTORED_UNRECONCILED·진행 중 단계 미확정)을 메모리에서만 적용하고 원장에 쓰지 않는다.
"""
import json
import re
import sqlite3
from pathlib import Path

from . import group_model as gm
from .dev_fixtures import clock
from .group_events import AssignRoles
from .group_machine import restore_state
from .registry import GroupRegistry
from .task_store import TaskStore

DEV_TOPIC_PREFIX = '/ax/dev/'
DEFAULT_OUTPUT_TOPIC = '/ax/dev/d2_mock/fleet_tasks'
FORBIDDEN_TOPIC_PARTS = ('/ax/fleet', '/ax/robot_')
TOPIC_PATTERN = re.compile(r'^/[a-z0-9_]+(/[a-z0-9_]+)+$')
FIXTURE_KEYS = ('fixture_id', 'map_revision', 'locations', 'tasks', 'assignments')
MAX_FIXTURE_BYTES = 256 * 1024
MAX_FIXTURE_TASKS = 32


class ConfigError(ValueError):
    """노드 설정·입력 거부."""


# 출력 Topic 검사: 개발 이름공간만, 운영 이름 포함 금지
def validate_output_topic(topic):
    if type(topic) is not str or not TOPIC_PATTERN.match(topic):
        raise ConfigError(f'출력 Topic 형식 오류: {topic!r}')

    if not topic.startswith(DEV_TOPIC_PREFIX):
        raise ConfigError(f'출력 Topic은 {DEV_TOPIC_PREFIX} 아래만 허용: {topic}')

    for part in FORBIDDEN_TOPIC_PARTS:
        if part in topic:
            raise ConfigError(f'운영 이름공간 이름을 포함한 출력 Topic 거부: {topic}')

    return topic


# fixture JSON을 읽어 작업을 수용·배정한 registry를 만든다(실행 시작·전송 없음)
def load_fixture(path):
    data_path = Path(path)
    if data_path.stat().st_size > MAX_FIXTURE_BYTES:
        raise ConfigError(f'fixture 크기 상한 초과: {MAX_FIXTURE_BYTES} bytes')

    try:
        data = json.loads(data_path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise ConfigError(f'fixture 읽기 실패: {error}') from error

    if not isinstance(data, dict) or set(data) != set(FIXTURE_KEYS):
        raise ConfigError(f'fixture 키는 정확히 {FIXTURE_KEYS}')

    fixture_id = data['fixture_id']
    if type(fixture_id) is not str or not gm.ID_PATTERN.match(fixture_id):
        raise ConfigError(f'fixture_id 형식 오류: {fixture_id!r}')

    locations = data['locations']
    if not isinstance(locations, dict) or not all(
            type(k) is str and type(v) is int and v >= 0 for k, v in locations.items()):
        raise ConfigError('locations는 {지점 ID: revision} 사전이어야 한다')

    tasks = data['tasks']
    if not isinstance(tasks, list) or len(tasks) > MAX_FIXTURE_TASKS:
        raise ConfigError(f'tasks는 최대 {MAX_FIXTURE_TASKS}개 목록')

    if not isinstance(data['assignments'], dict):
        raise ConfigError('assignments는 {task_id: 배정} 사전이어야 한다')

    registry = GroupRegistry(data['map_revision'], locations, max_tasks=MAX_FIXTURE_TASKS)
    for task in tasks:
        registry.admit(gm.parse_spec(task))

    for task_id, raw in data['assignments'].items():
        if task_id not in registry.states:
            raise ConfigError(f'배정 대상 작업 없음: {task_id}')
        decision = registry.apply(task_id, AssignRoles(gm.parse_assignment(raw)), clock(0.0))
        if registry.states[task_id].assignment is None:
            raise ConfigError(f'배정 거부: {task_id} {decision.reasons}')

    return fixture_id, list(registry.states.values())


# 원장을 읽기 전용으로 열어 상태를 읽고 복원 규칙을 메모리에서만 적용한다
def load_store_states(path):
    uri = Path(path).resolve().as_uri() + '?mode=ro'

    # 읽기 전용 URI 연결
    def connect(_path, **kwargs):
        return sqlite3.connect(uri, uri=True, **kwargs)

    store = TaskStore(str(path), connect=connect)
    try:
        return [restore_state(s) for s in store.load_states()]
    finally:
        store.close()
