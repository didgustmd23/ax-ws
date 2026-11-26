"""ETA 기준선·특징·분할·지표 시험(xgboost 불필요).

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_eta_ml_baseline.py
"""
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_travel_time_ml'))

from ax_travel_time_ml import baseline as bl  # noqa: E402
from ax_travel_time_ml import evaluation as ev  # noqa: E402
from ax_travel_time_ml import features as ft  # noqa: E402
from ax_travel_time_ml import synthetic  # noqa: E402


# 특징 사전 하나(기본값 + 덮어쓰기)
def _feature(**override):
    base = {
        'role': 'TRANSPORTER', 'robot_id': 'robot_2', 'from_location_id': 'a',
        'to_location_id': 'b', 'object_state': '3', 'first_leg': 0.0, 'carrying_object': 1.0,
        'attempt_index': 0.0, 'straight_distance_m': None, 'prev_nav_duration_s': None,
        'path_length_m': None, 'turn_count': None, 'corridor_wait_s': None,
        'obstacle_density': None,
    }
    base.update(override)

    return base


# 카탈로그(두 지점, 역할 정지 자세)
def _catalog():
    return ft.LocationCatalog({'catalog': {'map_revision': 2, 'locations': {
        'a': {'location_revision': 1, 'pose': {'x': 0, 'y': 0},
              'stop_poses': {'TRANSPORTER': {'x': 0, 'y': -0.6}}},
        'b': {'location_revision': 3, 'pose': {'x': 3, 'y': 4}},
    }}}, sha256='x')


# B1 → B2 → B3 → B4 → UNKNOWN 순으로 내려간다(min_n 경계 포함)
def test_hierarchy_levels():
    b1 = [_feature()] * 5
    b2 = [_feature(from_location_id='c')] * 4 + [_feature(from_location_id='d')]
    model = bl.Baseline.fit(b1 + b2, [10.0] * 5 + [20.0] * 5, min_n=5)

    assert model.predict(_feature()) == (10.0, 'BASELINE_B1', [])

    eta, source, reasons = model.predict(_feature(from_location_id='c'))
    assert (eta, source) == (15.0, 'BASELINE_B2')
    assert reasons == ['B1_INSUFFICIENT']

    eta, source, reasons = model.predict(_feature(to_location_id='z', straight_distance_m=6.7))
    assert source == 'BASELINE_B3'
    assert 'B3_LOWER_BOUND_ONLY' in reasons
    assert eta == pytest.approx(6.7 / bl.V_MAX_MPS + bl.ACCEL_TIME_S)

    eta, source, _ = model.predict(_feature(to_location_id='z'))
    assert source == 'BASELINE_B4'

    eta, source, reasons = model.predict(_feature(role='MANIPULATOR', to_location_id='z'))
    assert (eta, source) == (None, 'UNKNOWN')
    assert reasons[-1] == 'NO_BASELINE'


# 적합한 직선 기준선과 하한 클램프
def test_line_fit_and_clamp():
    rows = [_feature(to_location_id=f't{i}', straight_distance_m=float(i)) for i in range(1, 7)]
    targets = [2.0 * i + 3.0 for i in range(1, 7)]
    model = bl.Baseline.fit(rows, targets, min_n=5)

    assert model.line['lower_bound'] is False
    assert model.line['v_eff_mps'] == pytest.approx(0.5)
    assert model.line['t0_s'] == pytest.approx(3.0)

    reasons = []
    assert bl.clamp(1.0, 6.7, reasons) == pytest.approx(11.0)
    assert reasons == ['CLAMPED_TO_LOWER_BOUND']
    assert bl.clamp(1.0, None, []) == 1.0


# 저장 사전 왕복
def test_baseline_round_trip():
    model = bl.Baseline.fit([_feature()] * 5, [7.0] * 5)
    again = bl.Baseline.from_dict(model.to_dict())

    assert again.predict(_feature()) == model.predict(_feature())


# 카탈로그 거리: 역할 정지 자세 우선, 출발지 없음·revision 불일치는 None
def test_catalog_distance():
    catalog = _catalog()

    assert catalog.distance('TRANSPORTER', 'a', 'b', 3, 2) == pytest.approx(math.hypot(3, 4.6))
    assert catalog.distance('MANIPULATOR', 'a', 'b', 3, 2) == pytest.approx(5.0)
    assert catalog.distance('TRANSPORTER', None, 'b', 3, 2) is None
    assert catalog.distance('TRANSPORTER', 'a', 'b', 1, 2) is None
    assert catalog.distance('TRANSPORTER', 'a', 'b', 3, 9) is None
    assert catalog.distance('TRANSPORTER', 'a', 'zz', None, 2) is None

    with pytest.raises(ft.CatalogError):
        ft.LocationCatalog({'nothing': 1})


# 직전 주행 소요는 같은 작업·같은 로봇의 앞서 끝난 행만
def test_previous_durations_no_leak():
    rows = synthetic.make_rows(2, 0)
    previous = ft.previous_durations(rows)

    first, second = rows[0], rows[1]
    assert previous[first['execution_id']] is None
    assert previous[second['execution_id']] == first['duration_sim_s']

    # 다른 작업의 행은 sim 시각이 앞서도 쓰지 않는다
    other_task_first = rows[4]
    assert previous[other_task_first['execution_id']] is None


# 특징 명세: 전부 NULL 열 제외, 미지 범주·필수 결측 사유, 열 순서 검사
def test_feature_spec():
    features = ft.build_feature_rows(synthetic.make_rows(3, 0))
    spec = ft.FeatureSpec.fit(features)

    assert set(ft.CURRENTLY_NULL) <= set(spec.dropped_all_null)
    assert 'straight_distance_m' in spec.dropped_all_null

    vector, reasons = spec.vector(features[0])
    assert reasons == []
    assert len(vector) == len(spec.columns)

    _, reasons = spec.vector(dict(features[0], to_location_id='new', carrying_object=None))
    assert reasons == ['MISSING_FEATURE:carrying_object', 'UNSEEN_CATEGORY:to_location_id']

    data = spec.to_dict()
    assert ft.FeatureSpec.from_dict(data).columns == spec.columns

    data['columns'] = list(reversed(data['columns']))
    with pytest.raises(ValueError):
        ft.FeatureSpec.from_dict(data)


# 작업 단위 분할: 누설 없음, 결정적, 작업 수 부족은 예외
def test_group_split():
    rows = synthetic.make_rows(20, 0)
    train, test, tasks = ev.group_split(rows, seed=7)

    assert {r['task_id'] for r in train}.isdisjoint({r['task_id'] for r in test})
    assert len(train) + len(test) == len(rows)
    assert ev.group_split(rows, seed=7)[2] == tasks
    assert len(tasks) == 5

    with pytest.raises(ev.InsufficientData):
        ev.group_split(synthetic.make_rows(9, 0), seed=7)


# 지표 계산과 작업 합(예측 없는 행이 있는 작업은 합에서 뺀다)
def test_metrics():
    result = ev.error_metrics([1.0, 2.0, 3.0], [2.0, None, 1.0])
    assert result == {'n': 2, 'n_missing': 1, 'mae_s': 1.5,
                      'rmse_s': pytest.approx(math.sqrt(2.5))}

    rows = [{'task_id': 't1'}, {'task_id': 't1'}, {'task_id': 't2'}]
    summed = ev.task_sum_metrics(rows, [1.0, 2.0, 5.0], [2.0, 2.0, None])
    assert summed['n'] == 1
    assert summed['mae_s'] == 1.0
    assert summed['n_tasks_incomplete'] == 1
