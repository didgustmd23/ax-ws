"""개발·시험용 명시 작업/배정 fixture와 모의 관측 생성기.

여기 값은 실제 창고·지도·물품 원장에서 확인한 자료가 아니다. 출처는 input_source_id와
provenance=MOCK으로 표시하며 운영 근거로 쓰지 않는다. robot_id는 호출자가 정한다.
"""
from dataclasses import asdict

from .group_model import (
    PROVENANCE_MOCK, Assignment, ClockSnapshot, Observation, parse_assignment, parse_spec,
    standard_transfer_plan,
)

FIXTURE_SOURCE_ID = 'd2_dev_fixture'


# 표준 계획을 입력 schema(사전 목록)로 바꾼다
def standard_plan_dicts():
    steps = []
    for step in standard_transfer_plan():
        data = asdict(step)
        data['depends_on'] = list(step.depends_on)
        steps.append(data)

    return steps


# 개발 작업 입력 사전. 모든 값은 시험용 명시 자료이며 실환경 값이 아니다
def spec_dict(task_id='task_dev_1', object_id='box_dev_1', **overrides):
    data = {
        'team_id': 'team_dev',
        'task_id': task_id,
        'object_id': object_id,
        'inventory_revision': 1,
        'source_location_id': 'station_src',
        'source_location_revision': 1,
        'destination_location_id': 'station_dst',
        'destination_location_revision': 1,
        'map_revision': 1,
        'root_execution_id': f'root_{task_id}',
        'task_revision': 1,
        'schedule_revision': 0,
        'priority': 5,
        'source_transfer_id': f'{task_id}_xfer_src',
        'destination_transfer_id': f'{task_id}_xfer_dst',
        'input_source_id': FIXTURE_SOURCE_ID,
        'steps': standard_plan_dicts(),
    }
    data.update(overrides)

    return data


# 검증된 GroupTaskSpec fixture
def make_spec(task_id='task_dev_1', object_id='box_dev_1', **overrides):
    return parse_spec(spec_dict(task_id, object_id, **overrides))


# 검증된 배정 fixture. robot_id는 인자로 받는다(코드에 1/2를 고정하지 않음)
def make_assignment(manipulator, transporter, generation=1, assignment_id=None):
    return parse_assignment({
        'assignment_id': assignment_id or f'asg_{manipulator}_{transporter}_{generation}',
        'assignment_generation': generation,
        'manipulator_robot_id': manipulator,
        'transporter_robot_id': transporter,
    })


class MockObservationSource:
    """모의 관측 생성기. 생산자 세션·revision을 따로 유지해 STATE 소비 규칙을 시험한다."""

    # 출처 이름과 세션을 정한다
    def __init__(self, source_id='d2_mock_observer', session_id='mock_session_1'):
        self.source_id = source_id
        self.session_id = session_id
        self.revisions = {}

    # 관측 하나를 만든다. 같은 (종류, 로봇)마다 revision이 1씩 증가한다
    def make(self, kind, robot_id, value, now, object_id='', valid=True,
             age_s=0.0, provenance=PROVENANCE_MOCK, session_id=None, revision=None):
        key = (kind, robot_id)
        if revision is None:
            revision = self.revisions.get(key, 0) + 1
            self.revisions[key] = revision

        return Observation(
            kind=kind, robot_id=robot_id, object_id=object_id, value=value,
            source_id=self.source_id, session_id=session_id or self.session_id,
            revision=revision, valid=valid,
            stamp_sim_s=now.sim_time_s - age_s, received_boottime_s=now.boottime_s,
            provenance=provenance)


# 개발 등록 지점 카탈로그 사전. purposes는 {지점 ID: 용도}. 값은 시험용 가정값이며 실환경 값이 아니다
def catalog_dict(purposes, capacity=100, classes=('*',), map_revision=1, revision=1,
                 catalog_id='d2_dev_catalog'):
    locations = {}
    for index, (location_id, purpose) in enumerate(sorted(purposes.items())):
        pose = {'x': float(index), 'y': 0.0, 'yaw': 0.0}
        locations[location_id] = {
            'location_revision': revision,
            'purpose': purpose,
            'zone_id': f'zone_{location_id}',
            'capacity': capacity,
            'allowed_object_classes': list(classes),
            'max_load_kg': 5.0,
            'active': True,
            'pose': pose,
            'stop_poses': {'MANIPULATOR': dict(pose, y=0.6), 'TRANSPORTER': dict(pose, y=-0.6)},
            'wait_location_id': '',
        }

    return {'catalog_id': catalog_id, 'map_revision': map_revision, 'locations': locations}


# 시험용 시각
def clock(boottime_s, sim_time_s=None, time_valid=True):
    return ClockSnapshot(boottime_s, boottime_s if sim_time_s is None else sim_time_s,
                         time_valid)


__all__ = ['Assignment', 'FIXTURE_SOURCE_ID', 'MockObservationSource', 'catalog_dict', 'clock',
           'make_assignment', 'make_spec', 'spec_dict', 'standard_plan_dicts']
