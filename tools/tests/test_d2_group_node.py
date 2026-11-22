"""D2 개발 출력 노드의 설정·입력 검사(ROS 없음).

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_group_node.py
"""
import hashlib
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / 'src' / 'ax_fleet_manager'
sys.path.insert(0, str(PKG))

from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager.dev_fixtures import spec_dict  # noqa: E402
from ax_fleet_manager.node_config import (  # noqa: E402
    DEFAULT_OUTPUT_TOPIC, MAX_FIXTURE_BYTES, ConfigError, load_fixture, load_store_states,
    validate_output_topic,
)
from ax_fleet_manager.snapshot import task_list_fields  # noqa: E402
from ax_fleet_manager.task_store import StoreError  # noqa: E402

FIXTURE = PKG / 'config' / 'dev_fixture_transfer.json'


# 기본 출력 Topic은 개발 이름공간
def test_default_topic_is_dev_namespace():
    assert validate_output_topic(DEFAULT_OUTPUT_TOPIC) == '/ax/dev/d2_mock/fleet_tasks'


# 운영 이름공간·형식 오류 Topic 거부
@pytest.mark.parametrize('topic', [
    '/ax/fleet/tasks', '/ax/robot_1/fleet/tasks', '/ax/dev/ax/fleet/tasks',
    '/ax/dev/x/ax/robot_2/y', 'ax/dev/tasks', '/other/tasks', '/ax/dev/../fleet',
    '/ax/dev/', '/ax/dev/Tasks', '', None,
])
def test_rejects_operational_or_invalid_topic(topic):
    with pytest.raises(ConfigError):
        validate_output_topic(topic)


# 패키지 fixture: 작업 1개, 명시 배정(robot_id는 fixture 자료), 실행 시작 안 함
def test_packaged_fixture_loads():
    fixture_id, states = load_fixture(FIXTURE)
    assert fixture_id == 'dev_fixture_transfer'
    assert len(states) == 1
    state = states[0]
    assert state.task_state == gm.TASK_ASSIGNED
    assert state.spec.input_source_id == 'd2_dev_fixture'
    listed = task_list_fields(states)
    assert listed[0]['transfer_state'] == gm.TRANSFER_NONE
    assert all(run.status == 'PENDING' for run in state.steps.values())


# fixture 형식 검사
def write(tmp_path, data):
    path = tmp_path / 'fixture.json'
    path.write_text(json.dumps(data), encoding='utf-8')
    return path


# 정상 fixture 사전
def base_fixture():
    return json.loads(FIXTURE.read_text(encoding='utf-8'))


# fixture 거부 사례
@pytest.mark.parametrize('change', [
    lambda d: d.update(extra=1),
    lambda d: d.pop('locations'),
    lambda d: d.update(fixture_id='bad id'),
    lambda d: d.update(locations={'station_src': -1, 'station_dst': 1}),
    lambda d: d['assignments'].update(task_missing=d['assignments']['task_dev_1']),
    lambda d: d['assignments']['task_dev_1'].update(transporter_robot_id='robot_1'),
    lambda d: d.update(tasks=[spec_dict(f't{i}', f'b{i}') for i in range(40)]),
    lambda d: d['tasks'][0].update(map_revision=9),
    lambda d: d['tasks'][0].update(pick_pose=[0, 0, 0]),
])
def test_fixture_rejections(tmp_path, change):
    data = base_fixture()
    change(data)
    with pytest.raises((ConfigError, gm.SpecError)):
        load_fixture(write(tmp_path, data))


# 크기 상한
def test_fixture_size_limit(tmp_path):
    path = tmp_path / 'big.json'
    path.write_text(' ' * (MAX_FIXTURE_BYTES + 1), encoding='utf-8')
    with pytest.raises(ConfigError):
        load_fixture(path)


# 원장 읽기는 파일을 바꾸지 않고, 복원 규칙을 메모리에서만 적용한다
def test_store_read_only_and_restore_rules(tmp_path):
    sys.path.insert(0, str(ROOT / 'tools' / 'tests'))
    from test_d2_workflow_scenarios import begin, make_scenario

    runtime, mock, _, _ = make_scenario(tmp_path)
    mock.overrides[('task_a', 'load_pick_and_stow')] = gm.OUTCOME_FAILED
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()
    from ax_fleet_manager.group_events import FailTerminate
    runtime.handle('task_a', FailTerminate('fail_evt_a'))
    runtime.store.close()

    path = tmp_path / 'd2.sqlite'
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    states = load_store_states(path)
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before
    assert 'RESTORED_UNRECONCILED' in states[0].inhibit
    listed = task_list_fields(states)
    assert listed[0]['state'] == gm.TASK_FAILED
    assert listed[0]['transfer_state'] == gm.TRANSFER_IN_PROGRESS


# 없는 원장은 만들지 않고 거부한다
def test_missing_store_rejected(tmp_path):
    path = tmp_path / 'missing.sqlite'
    with pytest.raises(StoreError):
        load_store_states(path)
    assert not path.exists()
