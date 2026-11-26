"""ETA 학습 도구 CSV 계약·필터·해시 시험(xgboost 불필요).

원장 열(task_store.ETA_SAMPLE_FIELDS)과 export 스크립트 출력은 읽기만 한다.
실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_eta_ml_dataset.py
"""
import io
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_travel_time_ml'))
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))
sys.path.insert(0, str(ROOT / 'tools'))

from ax_travel_time_ml import dataset as ds  # noqa: E402
from ax_travel_time_ml import synthetic  # noqa: E402


# 합성 행을 개발 혼합 LIVE 표본처럼 바꾼다(export 왕복용)
def _live_rows(n_tasks=3, seed=0):
    rows = synthetic.make_rows(n_tasks, seed)
    for row in rows:
        row['mode'] = ds.DEFAULT_MODE
        row['provenance'] = ds.DEFAULT_PROVENANCE

    return rows


# 계약: 원장 열 순서와 같다
def test_columns_match_ledger_contract():
    from ax_fleet_manager.task_store import ETA_SAMPLE_FIELDS

    assert tuple(ds.EXPECTED_COLUMNS) == tuple(ETA_SAMPLE_FIELDS)


# 계약: export 스크립트의 CSV를 그대로 읽고 값이 왕복한다
def test_reads_export_script_output():
    import export_d2_eta_samples as export

    rows = _live_rows()
    stream = io.StringIO()
    export.write_csv(rows, stream)

    parsed = ds.parse_csv_bytes(stream.getvalue().encode('utf-8'))

    assert len(parsed) == len(rows)
    assert parsed[0]['from_location_id'] is None
    assert parsed[1]['carrying_object'] == 0
    assert parsed[3]['duration_sim_s'] == pytest.approx(rows[3]['duration_sim_s'])
    assert parsed[0]['path_length_m'] is None


# 필수 열 누락·중복 실행 ID·정수 열 소수는 거부
def test_rejects_bad_csv():
    text = ds.rows_to_csv_text(_live_rows(1))

    header, *body = text.splitlines()
    missing = ','.join(header.split(',')[:-1]) + '\n'
    with pytest.raises(ds.DatasetError, match='obstacle_density'):
        ds.parse_csv_bytes(missing.encode())

    duplicated = '\n'.join([header, body[0], body[0]]) + '\n'
    with pytest.raises(ds.DatasetError, match='중복'):
        ds.parse_csv_bytes(duplicated.encode())

    rows = _live_rows(1)
    rows[0]['attempt_index'] = 1.5
    with pytest.raises(ds.DatasetError, match='정수'):
        ds.parse_csv_bytes(ds.rows_to_csv_text(rows).encode())


# 필터는 사유별로 세고, 목표값 결측·0 이하를 뺀다
def test_filter_reasons():
    rows = _live_rows(2)
    rows[0]['mode'] = 'DEVELOPMENT_MOCK'
    rows[1]['provenance'] = 'MOCK'
    rows[2]['status'] = 'FAILED'
    rows[3]['duration_sim_s'] = None
    rows[4]['duration_sim_s'] = 0.0

    kept, excluded = ds.filter_rows(rows)

    assert len(kept) == 3
    assert excluded == {'INVALID_TARGET': 2, 'MODE_MISMATCH': 1, 'PROVENANCE_MISMATCH': 1,
                        'STATUS_MISMATCH': 1}


# 정규화 해시는 행 순서와 무관하고 값이 바뀌면 달라진다
def test_normalized_hash():
    rows = _live_rows(3)
    shuffled = list(reversed(rows))

    assert ds.normalized_sha256(rows) == ds.normalized_sha256(shuffled)

    rows[0] = dict(rows[0], duration_sim_s=rows[0]['duration_sim_s'] + 1e-9)
    assert ds.normalized_sha256(rows) != ds.normalized_sha256(shuffled)


# 출처 판정: 합성 표시 전부·일부·없음
def test_data_origin():
    synthetic_rows = synthetic.make_rows(1, 0)
    live = _live_rows(1)

    assert ds.data_origin(synthetic_rows) == 'synthetic'
    assert ds.data_origin(live) == 'live'
    assert ds.data_origin(synthetic_rows + live) == 'mixed'
