"""ETA 예측기 복귀 경로·CLI 종료 코드 시험. xgboost import를 실패로 바꿔 미설치 환경을 흉내 낸다.

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_eta_ml_predict.py
"""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_travel_time_ml'))

from ax_travel_time_ml import cli  # noqa: E402
from ax_travel_time_ml import dataset as ds  # noqa: E402
from ax_travel_time_ml import features as ft  # noqa: E402
from ax_travel_time_ml import model as md  # noqa: E402
from ax_travel_time_ml import synthetic  # noqa: E402
from ax_travel_time_ml import training  # noqa: E402
from ax_travel_time_ml.predict import EtaPredictor, MetaError  # noqa: E402

SYNTH_FILTERS = {'mode': ds.SYNTHETIC, 'provenance': ds.SYNTHETIC, 'status': 'SUCCEEDED'}


# xgboost import 실패를 흉내 낸다
@pytest.fixture
def no_xgboost(monkeypatch):
    # import 시도를 항상 ModelUnavailable로
    def fail():
        raise md.ModelUnavailable('시험: xgboost 없음', 'XGBOOST_MISSING')

    monkeypatch.setattr(md, 'import_xgboost', fail)


# 합성 CSV·카탈로그 파일을 만든다
def _write_inputs(tmp_path, n_tasks=20, seed=3):
    csv_path = tmp_path / 'samples.csv'
    csv_path.write_text(ds.rows_to_csv_text(synthetic.make_rows(n_tasks, seed)), encoding='utf-8')

    catalog_path = tmp_path / 'catalog.json'
    catalog_path.write_text(json.dumps(synthetic.make_catalog()), encoding='utf-8')

    return csv_path, catalog_path


# 목적지 구간 행 하나(예측 입력)
def _query(**override):
    row = {'role': 'TRANSPORTER', 'robot_id': 'robot_2', 'from_location_id': 'loc-syn-receipt-a',
           'to_location_id': 'loc-syn-storage-1', 'to_location_revision': 1, 'map_revision': 1,
           'object_state': 3, 'carrying_object': 1, 'attempt_index': 0}
    row.update(override)

    return row


# xgboost 없음: 기준선만 저장, 예측은 기준선, CLI는 4
def test_train_without_xgboost_saves_baseline(tmp_path, no_xgboost):
    csv_path, catalog_path = _write_inputs(tmp_path)
    catalog = ft.LocationCatalog.load(catalog_path)

    meta = training.train_pipeline(csv_path, tmp_path / 'm', catalog, filters=SYNTH_FILTERS)

    assert meta['model']['status'] == 'UNAVAILABLE'
    assert meta['adopted'] is False
    assert meta['performance_evidence'] is False
    assert meta['data']['origin'] == 'synthetic'
    assert not (tmp_path / 'm' / md.MODEL_FILE).exists()

    result = EtaPredictor(tmp_path / 'm', catalog, allow_synthetic=True).predict(_query())
    assert result.source.startswith('BASELINE_')
    assert result.eta_s > 0
    assert 'MODEL_NOT_LOADED:UNAVAILABLE' in result.reasons

    code = cli.train_main(['--csv', str(csv_path), '--catalog', str(catalog_path),
                           '--out', str(tmp_path / 'm2'), '--synthetic'])
    assert code == 4


# meta에는 TRAINED인데 xgboost가 없으면 XGBOOST_MISSING으로 복귀
def test_trained_meta_without_xgboost(tmp_path, no_xgboost):
    csv_path, catalog_path = _write_inputs(tmp_path)
    catalog = ft.LocationCatalog.load(catalog_path)
    training.train_pipeline(csv_path, tmp_path / 'm', catalog, filters=SYNTH_FILTERS)

    meta_path = tmp_path / 'm' / 'meta.json'
    meta = json.loads(meta_path.read_text())
    meta['model'].update({'status': 'TRAINED', 'file': md.MODEL_FILE, 'xgboost_version': '2.0.3'})
    meta_path.write_text(json.dumps(meta))

    predictor = EtaPredictor(tmp_path / 'm', catalog, allow_synthetic=True)
    result = predictor.predict(_query())

    assert predictor.load_reasons == ['MODEL_NOT_LOADED:XGBOOST_MISSING']
    assert result.source.startswith('BASELINE_')


# 합성 모델·카탈로그 불일치·미지 지도 revision·미지 범주는 모두 사유로 남는다
def test_blocker_reasons(tmp_path, no_xgboost):
    csv_path, catalog_path = _write_inputs(tmp_path)
    catalog = ft.LocationCatalog.load(catalog_path)
    training.train_pipeline(csv_path, tmp_path / 'm', catalog, filters=SYNTH_FILTERS)

    strict = EtaPredictor(tmp_path / 'm', catalog)
    assert 'SYNTHETIC_MODEL' in strict.predict(_query()).reasons

    other = ft.LocationCatalog(synthetic.make_catalog(), sha256='other')
    assert 'CATALOG_MISMATCH' in EtaPredictor(tmp_path / 'm', other).predict(_query()).reasons

    result = strict.predict(_query(map_revision=99, to_location_id='loc-new'))
    assert 'MAP_REVISION_UNSEEN' in result.reasons
    assert 'UNSEEN_CATEGORY:to_location_id' in result.reasons


# 기준선도 못 내면 UNKNOWN(값 없음)
def test_unknown_when_nothing_matches(tmp_path, no_xgboost):
    csv_path, _ = _write_inputs(tmp_path)
    training.train_pipeline(csv_path, tmp_path / 'm', None, filters=SYNTH_FILTERS)

    result = EtaPredictor(tmp_path / 'm', None).predict(_query(role='INSPECTOR'))

    assert result.eta_s is None
    assert result.source == 'UNKNOWN'
    assert 'NO_BASELINE' in result.reasons


# 표본 부족(작업 수 < 10)은 지표 없이 종료 코드 3, meta를 쓰지 않는다
def test_insufficient_data_exit_code(tmp_path, no_xgboost):
    csv_path, _ = _write_inputs(tmp_path, n_tasks=5)

    code = cli.train_main(['--csv', str(csv_path), '--out', str(tmp_path / 'm'), '--synthetic'])

    assert code == 3
    assert not (tmp_path / 'm' / 'meta.json').exists()


# 기본 필터(혼합 LIVE)는 합성 표본을 학습에 쓰지 않는다
def test_default_filter_excludes_synthetic(tmp_path, no_xgboost):
    csv_path, _ = _write_inputs(tmp_path)

    assert cli.train_main(['--csv', str(csv_path), '--out', str(tmp_path / 'm')]) == 3


# meta 없음·CSV 열 누락은 종료 코드 2
def test_input_errors(tmp_path):
    csv_path, _ = _write_inputs(tmp_path)

    with pytest.raises(MetaError):
        EtaPredictor(tmp_path / 'missing')

    assert cli.evaluate_main(['--csv', str(csv_path), '--model-dir', str(tmp_path / 'x')]) == 2

    bad = tmp_path / 'bad.csv'
    bad.write_text('execution_id,task_id\n1,t\n', encoding='utf-8')
    assert cli.train_main(['--csv', str(bad), '--out', str(tmp_path / 'm')]) == 2
