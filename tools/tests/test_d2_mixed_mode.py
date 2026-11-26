"""D2 혼합 개발 모드(DEVELOPMENT_MIXED) 시험(ROS 없음, 모의 상대).

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_mixed_mode.py
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager import receipt as rc  # noqa: E402
from ax_fleet_manager.dev_fixtures import (  # noqa: E402
    catalog_dict, make_assignment, make_spec,
)
from ax_fleet_manager.group_events import (  # noqa: E402
    AssignRoles, DispatchReserved, DispatchSent, GoalResponse, ObservationReceived, StartTask,
    StepResult, TaskStatusSeen,
)
from ax_fleet_manager.inventory import parse_catalog  # noqa: E402
from ax_fleet_manager.live_adapters import D1NavigationPort  # noqa: E402
from ax_fleet_manager.node_config import node_mode  # noqa: E402
from ax_fleet_manager.ports import MockLocalTaskPort, MockNavigationPort  # noqa: E402
from ax_fleet_manager.runtime import GroupRuntime  # noqa: E402
from ax_fleet_manager.snapshot import source_prefix  # noqa: E402
from ax_fleet_manager.task_store import TaskStore, state_from_dict, state_to_dict  # noqa: E402
from test_d2_group_machine import Flow, run_until  # noqa: E402
from test_d2_workflow_scenarios import LOCATIONS, Clock  # noqa: E402

MOCK = gm.PROVENANCE_MOCK
LIVE = gm.PROVENANCE_LIVE
MIXED = gm.MODE_DEVELOPMENT_MIXED
FIRST_TRIAL = ('ACTUATION', 'NAVIGATION')


class MixedFlow(Flow):
    """혼합 모드 시험 구동기. live로 연 종류의 결과·응답·관측은 LIVE 출처로 넣는다."""

    # 혼합 모드 작업 상태로 시작한다
    def __init__(self, live_kinds=FIRST_TRIAL, **kwargs):
        super().__init__(**kwargs)
        self.live_kinds = tuple(live_kinds)
        self.state = gm.new_group_state(self.spec, MIXED, self.live_kinds)

    # 단계 근거 종류의 출처(주행은 NAVIGATION, 그 밖은 LOCAL_TASK=항상 MOCK)
    def step_provenance(self, step_key):
        operation = gm.step_by_key(self.spec, step_key).operation
        if operation == 'NAVIGATE_TO' and 'NAVIGATION' in self.live_kinds:
            return LIVE

        return MOCK

    # 의도 전송·수락. 수락 응답 출처는 단계 종류를 따른다
    def send(self, intent):
        uuid = f'uuid_{intent.step_key}'
        self.apply(DispatchReserved(intent.execution['execution_id'], uuid))
        self.apply(DispatchSent(intent.execution['execution_id']))
        self.apply(GoalResponse(uuid, True, self.step_provenance(intent.step_key)))

        return uuid

    # 단계 결과. 출처는 단계 종류를 따른다
    def finish(self, step_key, outcome=gm.OUTCOME_SUCCEEDED, hold=0, load=0, robot=None):
        return self.apply(StepResult(f'uuid_{step_key}', robot or self.robot(step_key),
                                     outcome, self.step_provenance(step_key), '', hold, load))

    # 관측. live로 연 종류는 LIVE 출처로 낸다
    def observe(self, kind, robot, value, **kwargs):
        kwargs.setdefault('provenance', LIVE if kind in self.live_kinds else MOCK)
        return self.apply(ObservationReceived(
            self.source.make(kind, robot, value, self.now(), **kwargs)))


# 모드 검사: 혼합 모드 수용, 운영 모드는 계속 거부
def test_check_mode_accepts_mixed():
    gm.check_mode(MIXED)
    gm.check_mode(gm.MODE_DEVELOPMENT_MOCK)

    with pytest.raises(gm.SpecError) as error:
        gm.check_mode(gm.MODE_OPERATIONAL)
    assert error.value.code == 'OPERATIONAL_MODE_UNSUPPORTED'

    with pytest.raises(gm.SpecError) as error:
        gm.new_group_state(make_spec(), gm.MODE_OPERATIONAL, ('ACTUATION',))
    assert error.value.code == 'OPERATIONAL_MODE_UNSUPPORTED'


# live 허용 종류 검사: 정렬 tuple, 기본 빈 값, 열 수 없는 이름·형식·중복·모의 모드 거부
@pytest.mark.parametrize('mode, kinds, expected', [
    (MIXED, [], ()),
    (MIXED, ['NAVIGATION', 'ACTUATION'], ('ACTUATION', 'NAVIGATION')),
    (MIXED, ('HOLD',), ('HOLD',)),
    (gm.MODE_DEVELOPMENT_MOCK, [], ()),
])
def test_parse_live_kinds_ok(mode, kinds, expected):
    assert gm.parse_live_kinds(mode, kinds) == expected
    assert node_mode(mode, kinds) == expected


@pytest.mark.parametrize('mode, kinds, code', [
    (MIXED, ['LOAD'], 'LIVE_KIND_NOT_OPENABLE'),
    (MIXED, ['PLACEMENT'], 'LIVE_KIND_NOT_OPENABLE'),
    (MIXED, ['ARM_RETRACTED'], 'LIVE_KIND_NOT_OPENABLE'),
    (MIXED, ['LOCAL_TASK'], 'LIVE_KIND_NOT_OPENABLE'),
    (MIXED, ['actuation'], 'LIVE_KIND_NOT_OPENABLE'),
    (MIXED, ['ACTUATION', 'ACTUATION'], 'DUPLICATE_LIVE_KIND'),
    (MIXED, 'ACTUATION', 'INVALID_LIVE_KINDS'),
    (MIXED, [1], 'INVALID_LIVE_KINDS'),
    (MIXED, None, 'INVALID_LIVE_KINDS'),
    (gm.MODE_DEVELOPMENT_MOCK, ['ACTUATION'], 'LIVE_KINDS_REQUIRE_MIXED_MODE'),
])
def test_parse_live_kinds_rejects(mode, kinds, code):
    with pytest.raises(gm.SpecError) as error:
        gm.parse_live_kinds(mode, kinds)
    assert error.value.code == code

    with pytest.raises(gm.SpecError):
        gm.new_group_state(make_spec(), mode, kinds)


# 출처 판정: 혼합 모드는 종류마다 출처 하나, 모의·운영 모드는 기존과 같다
def test_provenance_acceptable_matrix():
    kinds = ('ACTUATION',)

    assert gm.provenance_acceptable(MIXED, LIVE, 'ACTUATION', kinds)
    assert not gm.provenance_acceptable(MIXED, MOCK, 'ACTUATION', kinds)
    assert gm.provenance_acceptable(MIXED, MOCK, 'LOAD', kinds)
    assert not gm.provenance_acceptable(MIXED, LIVE, 'LOAD', kinds)
    assert gm.provenance_acceptable(MIXED, MOCK, 'ACTUATION', ())
    assert not gm.provenance_acceptable(MIXED, LIVE, 'ACTUATION', ())
    assert not gm.provenance_acceptable(MIXED, 'OTHER', 'ACTUATION', kinds)
    assert not gm.provenance_acceptable(MIXED, MOCK)
    assert not gm.provenance_acceptable(MIXED, LIVE, None, kinds)
    assert not gm.provenance_acceptable(MIXED, MOCK, None, ())
    assert not gm.provenance_acceptable(MIXED, LIVE, None, ())
    assert not gm.provenance_acceptable(MIXED, MOCK, 1, ())
    assert gm.provenance_acceptable(gm.MODE_DEVELOPMENT_MOCK, MOCK)
    assert not gm.provenance_acceptable(gm.MODE_DEVELOPMENT_MOCK, LIVE)

    assert gm.provenance_acceptable(gm.MODE_DEVELOPMENT_MOCK, MOCK, 'ACTUATION', kinds)
    assert not gm.provenance_acceptable(gm.MODE_DEVELOPMENT_MOCK, LIVE, 'ACTUATION', kinds)
    assert gm.provenance_acceptable(gm.MODE_OPERATIONAL, LIVE)


# 혼합 모드 관측: 연 종류(ACTUATION)는 LIVE만, 열지 않은 종류(LOAD)는 MOCK만 받는다
def test_mixed_observation_provenance_per_kind():
    flow = MixedFlow(live_kinds=('ACTUATION',))
    flow.start()
    m, t = flow.assignment.robots()

    rejected = flow.observe('ACTUATION', m, gm.ACTUATION_STOPPED, provenance=MOCK)
    assert 'PROVENANCE_REJECTED' in rejected.reasons
    assert ('ACTUATION', m) not in flow.state.observations

    accepted = flow.observe('ACTUATION', m, gm.ACTUATION_STOPPED, provenance=LIVE)
    assert 'PROVENANCE_REJECTED' not in accepted.reasons
    assert flow.state.observations[('ACTUATION', m)].provenance == LIVE

    load_live = flow.observe('LOAD', t, gm.LOAD_EMPTY, object_id=flow.spec.object_id,
                             provenance=LIVE)
    assert 'PROVENANCE_REJECTED' in load_live.reasons

    load_mock = flow.observe('LOAD', t, gm.LOAD_EMPTY, object_id=flow.spec.object_id,
                             provenance=MOCK)
    assert 'PROVENANCE_REJECTED' not in load_mock.reasons


# 혼합 모드 ACTUATION 정지 근거: LIVE 정지만 잠금 단계로 이어진다
@pytest.mark.parametrize('provenance, locks', [(LIVE, True), (MOCK, False)])
def test_mixed_actuation_gates_lock(provenance, locks):
    flow = MixedFlow(live_kinds=('ACTUATION',))
    flow.start()
    flow.send_all()
    flow.wait(0.05)
    for robot in flow.assignment.robots():
        flow.observe('ACTUATION', robot, gm.ACTUATION_STOPPED, provenance=provenance)
    flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')

    assert flow.intended('src_lock_manipulator') is locks


# 혼합 모드 NAVIGATION: 주행 결과·수락 응답은 LIVE만, 로컬 작업(A3) 결과는 LIVE 거절
def test_mixed_navigation_results_and_local_task():
    flow = MixedFlow(live_kinds=('NAVIGATION',))
    flow.start()
    flow.send_all()
    assert flow.state.steps['src_nav_manipulator'].status == 'ACCEPTED'

    robot = flow.robot('src_nav_manipulator')
    mock = flow.apply(StepResult('uuid_src_nav_manipulator', robot, gm.OUTCOME_SUCCEEDED,
                                 MOCK, '', 0, 0))
    assert 'PROVENANCE_REJECTED' in mock.reasons
    assert flow.state.steps['src_nav_manipulator'].status == 'ACCEPTED'

    live = flow.apply(StepResult('uuid_src_nav_manipulator', robot, gm.OUTCOME_SUCCEEDED,
                                 LIVE, '', 0, 0))
    assert 'PROVENANCE_REJECTED' not in live.reasons
    assert flow.state.steps['src_nav_manipulator'].status == 'SUCCEEDED'

    # 주행 완료 → 정지 근거(ACTUATION은 열지 않아 MOCK) → 잠금 의도. 로컬 작업 응답은 LIVE 거절
    flow.finish('src_nav_transporter')
    flow.wait(0.05)
    flow.evidence('stopped')
    intent = [i for i in flow.unsent() if i.step_key == 'src_lock_manipulator'][0]
    flow.apply(DispatchReserved(intent.execution['execution_id'], 'uuid_src_lock_manipulator'))
    flow.apply(DispatchSent(intent.execution['execution_id']))
    reply = flow.apply(GoalResponse('uuid_src_lock_manipulator', True, LIVE))
    assert 'PROVENANCE_REJECTED' in reply.reasons
    assert flow.state.steps['src_lock_manipulator'].status == 'SENT'

    # 로컬 작업 결과: 혼합 모드여도 LIVE는 받지 않는다
    flow.apply(GoalResponse('uuid_src_lock_manipulator', True, MOCK))
    local = flow.apply(StepResult('uuid_src_lock_manipulator', intent.robot_id,
                                  gm.OUTCOME_SUCCEEDED, LIVE, '', 0, 0))
    assert 'PROVENANCE_REJECTED' in local.reasons
    assert flow.state.steps['src_lock_manipulator'].status == 'ACCEPTED'


# 혼합 모드 상태 사건: 주행 단계는 NAVIGATION 출처, 로컬 작업 단계는 MOCK만
def test_mixed_task_status_kind():
    flow = MixedFlow(live_kinds=('NAVIGATION',))
    flow.start()
    flow.send_all()
    run = flow.state.steps['src_nav_manipulator']
    execution = {'task_id': flow.spec.task_id,
                 'assignment_id': flow.assignment.assignment_id,
                 'assignment_generation': flow.assignment.assignment_generation,
                 'execution_id': run.execution_id}
    robot = flow.robot('src_nav_manipulator')

    live = flow.apply(TaskStatusSeen(robot, 's1', 1, execution, 'ACTIVE', -1, LIVE))
    mock = flow.apply(TaskStatusSeen(robot, 's1', 2, execution, 'ACTIVE', -1, MOCK))

    assert 'PROVENANCE_REJECTED' not in live.reasons
    assert 'PROVENANCE_REJECTED' in mock.reasons


# 혼합 모드 끝까지: ACTUATION·NAVIGATION LIVE + 나머지 MOCK으로 완료, 근거 요약 누적
def test_mixed_full_flow_evidence_summary():
    flow = MixedFlow()
    run_until(flow, 'dst_unlock')

    assert flow.state.task_state == gm.TASK_COMPLETED
    used = set(flow.state.evidence_used)
    assert {('ACTUATION', LIVE), ('NAVIGATION', LIVE), ('LOCAL_TASK', MOCK),
            ('LOAD', MOCK), ('HOLD', MOCK)} <= used
    assert ('ACTUATION', MOCK) not in used and ('NAVIGATION', MOCK) not in used
    assert list(flow.state.evidence_used) == sorted(used)


# 모의 모드 끝까지: 근거 요약은 전부 MOCK
def test_mock_full_flow_evidence_all_mock():
    flow = Flow()
    run_until(flow, 'dst_unlock')

    assert flow.state.task_state == gm.TASK_COMPLETED
    assert flow.state.evidence_used
    assert all(provenance == MOCK for _, provenance in flow.state.evidence_used)


# 완료 사건 기록: COMPLETED로 바뀐 판정에서 한 번만 모드·근거 요약을 남긴다
@pytest.mark.parametrize('make_flow', [lambda: MixedFlow(), lambda: Flow()])
def test_completed_event_records_evidence(tmp_path, make_flow):
    flow = make_flow()
    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    store.create_task(flow.state)
    previous = flow.state
    run_until(flow, 'dst_unlock')

    for decision in flow.decisions:
        store.record_decision(flow.spec.task_id, previous.revision, decision, 'Event', {})
        previous = decision.next_state

    completed = [p for kind, p in store.events(flow.spec.task_id) if kind == 'Completed']
    assert completed == [{'mode': flow.state.mode,
                          'live_kinds': list(flow.state.live_kinds),
                          'evidence_used': [list(p) for p in flow.state.evidence_used]}]
    store.close()


# D1 어댑터 supported: 켜고 혼합 모드에서 NAVIGATION을 열었을 때만
def test_d1_port_supported_conditions():
    catalog = parse_catalog(catalog_dict({'station_src': 'STATION', 'station_dst': 'STORAGE'}))
    port = D1NavigationPort({}, catalog, enabled=True)

    assert port.supported(MIXED, ('NAVIGATION',))
    assert port.supported(MIXED, ('ACTUATION', 'NAVIGATION'))
    assert not port.supported(MIXED, ('ACTUATION',))
    assert not port.supported(MIXED, ())
    assert not port.supported(gm.MODE_DEVELOPMENT_MOCK, ('NAVIGATION',))
    assert not port.supported(gm.MODE_OPERATIONAL, ('NAVIGATION',))
    assert not D1NavigationPort({}, catalog).supported(MIXED, ('NAVIGATION',))

    assert not MockLocalTaskPort().supported(gm.MODE_OPERATIONAL)


# 모의 포트: 담당 근거 종류를 live로 열면 거절(종류마다 출처 하나). 로컬 작업은 열 수 없어 계속 허용
@pytest.mark.parametrize('live_kinds, nav_ok', [
    (('NAVIGATION',), False), (('ACTUATION', 'NAVIGATION'), False), (('ACTUATION',), True),
    ((), True),
])
def test_mock_ports_supported_per_kind(live_kinds, nav_ok):
    assert MockNavigationPort().supported(MIXED, live_kinds) is nav_ok
    assert MockLocalTaskPort().supported(MIXED, live_kinds)
    assert MockNavigationPort().supported(gm.MODE_DEVELOPMENT_MOCK)
    assert MockLocalTaskPort().supported(gm.MODE_DEVELOPMENT_MOCK)
    assert (MockNavigationPort.evidence_kind, MockLocalTaskPort.evidence_kind) == (
        'NAVIGATION', 'LOCAL_TASK')


# runtime: 혼합 모드 + NAVIGATION에서 모의 주행 포트는 보내지 않는다(PORT_UNSUPPORTED)
def test_runtime_rejects_mock_navigation_when_live(tmp_path):
    nav = MockNavigationPort()
    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    runtime = GroupRuntime(store, {'LOCAL_TASK': MockLocalTaskPort(), 'NAVIGATION': nav},
                           Clock(), map_revision=1, locations=LOCATIONS, mode=MIXED,
                           live_kinds=('NAVIGATION',))
    runtime.admit(make_spec())
    runtime.handle('task_dev_1', AssignRoles(make_assignment('robot_m7', 'robot_t3')))
    runtime.handle('task_dev_1', StartTask())

    failed = [p for kind, p in store.events('task_dev_1') if kind == 'DispatchFailed']
    assert nav.requests == []
    assert failed and all(p['payload']['reason'] == 'PORT_UNSUPPORTED' for p in failed)
    store.close()


# runtime: 혼합 모드라도 NAVIGATION을 열지 않으면 D1 포트로 보내지 않는다(PORT_UNSUPPORTED)
@pytest.mark.parametrize('live_kinds, sent', [((), False), (('NAVIGATION',), True)])
def test_runtime_port_supported_uses_live_kinds(tmp_path, live_kinds, sent):

    class Client:
        """D1 NavigationClient 흉내(호출 기록만)."""

        # 호출 기록을 준비한다
        def __init__(self):
            self.calls = []

        # submit 기록 후 UUID를 돌려준다
        def submit(self, **kwargs):
            self.calls.append(kwargs)
            return {'goal_uuid': f'd1_{len(self.calls)}'}

        # 취소 기록
        def cancel(self, execution_id):
            self.calls.append(('cancel', execution_id))

    catalog = parse_catalog(catalog_dict({'station_src': 'STATION', 'station_dst': 'STORAGE'}))
    clients = {'robot_m7': Client(), 'robot_t3': Client()}
    port = D1NavigationPort(clients, catalog, enabled=True)
    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    runtime = GroupRuntime(store, {'LOCAL_TASK': MockLocalTaskPort(), 'NAVIGATION': port},
                           Clock(), map_revision=catalog.map_revision,
                           locations=catalog.revisions(), mode=MIXED, live_kinds=live_kinds)
    runtime.admit(make_spec())
    runtime.handle('task_dev_1', AssignRoles(make_assignment('robot_m7', 'robot_t3')))
    runtime.handle('task_dev_1', StartTask())

    events = store.events('task_dev_1')
    failed = [p for kind, p in events if kind == 'DispatchFailed']
    assert bool(clients['robot_m7'].calls) is sent
    if sent:
        assert not failed
    else:
        assert failed and failed[0]['payload']['reason'] == 'PORT_UNSUPPORTED'
    store.close()


# 원장: live_kinds·근거 요약 영속·복원, 모드·live_kinds 불일치 열기 거부
def test_store_keeps_mode_and_rejects_mismatch(tmp_path):
    path = str(tmp_path / 'd2.sqlite')
    store = TaskStore(path)
    runtime = GroupRuntime(store, {}, Clock(), map_revision=1, locations=LOCATIONS,
                           mode=MIXED, live_kinds=['NAVIGATION', 'ACTUATION'])
    runtime.admit(make_spec())
    store.close()

    store = TaskStore(path)
    reopened = GroupRuntime(store, {}, Clock(), map_revision=1, locations=LOCATIONS,
                            mode=MIXED, live_kinds=FIRST_TRIAL)
    state = reopened.registry.states['task_dev_1']
    assert (state.mode, state.live_kinds) == (MIXED, FIRST_TRIAL)
    store.close()

    for mode, kinds in ((gm.MODE_DEVELOPMENT_MOCK, ()), (MIXED, ()), (MIXED, ('ACTUATION',))):
        store = TaskStore(path)
        before = store.events('task_dev_1')
        with pytest.raises(gm.SpecError) as error:
            GroupRuntime(store, {}, Clock(), map_revision=1, locations=LOCATIONS,
                         mode=mode, live_kinds=kinds)
        assert error.value.code == 'STORE_MODE_MISMATCH'
        assert store.events('task_dev_1') == before
        store.close()


# 기존 모의 원장은 혼합 모드로 열지 않는다
def test_mock_store_cannot_open_as_mixed(tmp_path):
    path = str(tmp_path / 'd2.sqlite')
    store = TaskStore(path)
    GroupRuntime(store, {}, Clock(), map_revision=1, locations=LOCATIONS).admit(make_spec())
    store.close()

    store = TaskStore(path)
    with pytest.raises(gm.SpecError) as error:
        GroupRuntime(store, {}, Clock(), map_revision=1, locations=LOCATIONS, mode=MIXED)
    assert error.value.code == 'STORE_MODE_MISMATCH'
    store.close()


# 이전 상태 JSON(live_kinds·evidence_used 없음)은 빈 값으로 읽는다. 저장·복원은 왕복한다
def test_state_json_compat_and_round_trip():
    flow = MixedFlow()
    run_until(flow, 'src_unlock')
    data = state_to_dict(flow.state)

    restored = state_from_dict(data)
    assert restored.live_kinds == FIRST_TRIAL
    assert restored.evidence_used == flow.state.evidence_used

    old = {k: v for k, v in state_to_dict(Flow().state).items()
           if k not in ('live_kinds', 'evidence_used')}
    legacy = state_from_dict(old)
    assert (legacy.mode, legacy.live_kinds, legacy.evidence_used) == (
        gm.MODE_DEVELOPMENT_MOCK, (), ())

    with pytest.raises(gm.SpecError):
        state_from_dict(dict(old, live_kinds=['ACTUATION']))


# 출력 source_id 접두어: 작업 상태의 모드로 정한다
def test_source_prefix_from_states():
    mock_state = Flow().state
    mixed_state = MixedFlow().state

    assert source_prefix([]) == 'd2_dev_mock:'
    assert source_prefix([mock_state]) == 'd2_dev_mock:'
    assert source_prefix([mock_state, mixed_state]) == 'd2_dev_mixed:'


# 입고: 혼합 모드는 정답 기반 관측을 모의 모드와 같게 다룬다(allow_ground_truth 존중)
@pytest.mark.parametrize('mode, allow, rejected', [
    (gm.MODE_DEVELOPMENT_MOCK, True, False),
    (MIXED, True, False),
    (MIXED, False, True),
    (gm.MODE_OPERATIONAL, True, True),
])
def test_receipt_ground_truth_in_mixed(mode, allow, rejected):
    config = rc.ReceiptConfig(receipt_zones=('zone_in',), allow_ground_truth=allow)
    obs = rc.parse_observation({
        'observation_id': 'ob-1', 'zone_id': 'zone_in', 'object_id': 'obj-1',
        'object_class': 'box_small', 'identity_confidence': 0.9,
        'source': rc.SOURCE_GROUND_TRUTH, 'source_id': 'gt', 'valid': True,
        'stamp_sim_s': 100.0})

    reason = rc.observation_rejection(obs, gm.ClockSnapshot(100.0, 100.0), config, mode)

    assert (reason == 'PROVENANCE_REJECTED') is rejected
