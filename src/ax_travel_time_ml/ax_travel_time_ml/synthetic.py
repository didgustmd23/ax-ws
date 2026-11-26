"""합성 ETA 표본 생성기(코드 경로 시험 전용).

합성 표본은 mode·provenance를 모두 SYNTHETIC으로 표시한다. 이 데이터의 학습·평가 결과는
AX 주행 성능의 근거가 아니다(meta의 performance_evidence=False).
생성 규칙(알려진 함수 + 잡음): 소요 = 거리/v_role + 회전×2.0 + 적재×1.5 + 정규 잡음(σ 0.5 s).
첫 구간(출발지 NULL)은 거리 1~6 m를 임의로 정한다. D2처럼 작업마다 두 역할 × 두 구간(출발지로,
목적지로) 4행을 만든다.
"""
import math

import numpy as np

from .dataset import SYNTHETIC

ROLES = {'MANIPULATOR': ('robot_1', 0.45), 'TRANSPORTER': ('robot_2', 0.55)}
STATION_POSES = {
    'loc-syn-receipt-a': (0.0, 0.0), 'loc-syn-receipt-b': (0.0, 4.0),
    'loc-syn-storage-1': (6.0, 0.0), 'loc-syn-storage-2': (6.0, 3.0),
    'loc-syn-storage-3': (10.0, 1.0), 'loc-syn-storage-4': (10.0, 5.0),
}
ROLE_OFFSET_Y = {'MANIPULATOR': 0.6, 'TRANSPORTER': -0.6}
MAP_REVISION = 1


# 합성 카탈로그(dev_intake.json의 catalog 형식)
def make_catalog():
    locations = {}
    for location_id, (x, y) in STATION_POSES.items():
        locations[location_id] = {
            'location_revision': 1,
            'pose': {'x': x, 'y': y, 'yaw': 0.0},
            'stop_poses': {role: {'x': x, 'y': y + dy, 'yaw': 0.0}
                           for role, dy in ROLE_OFFSET_Y.items()},
        }

    return {'catalog': {'catalog_id': 'synthetic', 'map_revision': MAP_REVISION,
                        'locations': locations}}


# 역할 정지 자세 사이 거리
def _distance(role, a, b):
    (ax, ay), (bx, by) = STATION_POSES[a], STATION_POSES[b]

    return math.hypot(bx - ax, (by + ROLE_OFFSET_Y[role]) - (ay + ROLE_OFFSET_Y[role]))


# 행 하나(시각·결과 열 포함)
def _row(task, step_key, role, from_id, to_id, carrying, sent, duration, path, turns):
    robot_id = ROLES[role][0]

    return {
        'execution_id': f'{task}:{step_key}:{role}', 'task_id': task, 'step_key': step_key,
        'robot_id': robot_id, 'role': role, 'attempt_index': 0,
        'mode': SYNTHETIC, 'live_kinds': '[]', 'provenance': SYNTHETIC,
        'status': 'SUCCEEDED', 'outcome': 0, 'error_code': '',
        'from_location_id': from_id, 'to_location_id': to_id, 'to_location_revision': 1,
        'map_revision': MAP_REVISION, 'object_state': 3 if carrying else 1,
        'carrying_object': carrying,
        'sent_boottime_s': sent, 'sent_sim_s': sent,
        'accepted_boottime_s': sent + 0.1, 'accepted_sim_s': sent + 0.1,
        'finished_boottime_s': sent + duration, 'finished_sim_s': sent + duration,
        'duration_s': duration, 'duration_sim_s': duration,
        'path_length_m': path, 'turn_count': turns, 'corridor_wait_s': None,
        'obstacle_density': None,
    }


# 합성 행 목록. with_path_features=False면 경로 열을 원장 현재 상태처럼 NULL로 둔다
def make_rows(n_tasks, seed, with_path_features=False):
    rng = np.random.RandomState(int(seed))
    receipts = [k for k in STATION_POSES if 'receipt' in k]
    storages = [k for k in STATION_POSES if 'storage' in k]
    rows = []

    for index in range(n_tasks):
        task = f'task-syn-{index:04d}'
        source = receipts[rng.randint(len(receipts))]
        destination = storages[rng.randint(len(storages))]
        clock = 100.0 * index

        for role, (_, speed) in ROLES.items():
            # 첫 구간: 출발지 미기록, 거리 임의
            first = float(rng.uniform(1.0, 6.0))
            turns = int(rng.randint(0, 3))
            duration = first / speed + 2.0 * turns + float(rng.normal(0.0, 0.5))
            duration = max(duration, 0.5)
            rows.append(_row(task, 'go_source', role, None, source, 0, clock, duration,
                             first * 1.1 if with_path_features else None,
                             turns if with_path_features else None))

            # 목적지 구간: 운송 역할만 적재
            carrying = 1 if role == 'TRANSPORTER' else 0
            sent = clock + duration + 20.0
            distance = _distance(role, source, destination)
            turns = int(rng.randint(0, 4))
            duration = (distance / speed + 2.0 * turns + 1.5 * carrying
                        + float(rng.normal(0.0, 0.5)))
            duration = max(duration, 0.5)
            rows.append(_row(task, 'go_destination', role, source, destination, carrying, sent,
                             duration, distance * 1.1 if with_path_features else None,
                             turns if with_path_features else None))

    return rows
