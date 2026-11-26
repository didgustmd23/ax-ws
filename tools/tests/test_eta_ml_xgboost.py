"""XGBoost 학습·재현성·로드·복귀 시험. xgboost가 없으면 import에서 실패한다(skip하지 않는다).

합성 데이터의 지표는 코드 경로 확인용이며 AX 주행 성능의 근거가 아니다.
실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_eta_ml_xgboost.py
"""
import json
import math
import sys
from pathlib import Path

import xgboost  # noqa: F401  미설치면 이 시험 파일은 실패해야 한다

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_travel_time_ml'))

from ax_travel_time_ml import dataset as ds  # noqa: E402
from ax_travel_time_ml import features as ft  # noqa: E402
from ax_travel_time_ml import model as md  # noqa: E402
from ax_travel_time_ml import synthetic  # noqa: E402
from ax_travel_time_ml import training  # noqa: E402
from ax_travel_time_ml.predict import EtaPredictor  # noqa: E402

SYNTH_FILTERS = {'mode': ds.SYNTHETIC, 'provenance': ds.SYNTHETIC, 'status': 'SUCCEEDED'}


# 경로 특징을 채운 합성 입력으로 학습한다
def _train(tmp_path, name, seed=11, n_tasks=80):
    csv_path = tmp_path / 'samples.csv'
    if not csv_path.exists():
        rows = synthetic.make_rows(n_tasks, 5, with_path_features=True)
        csv_path.write_text(ds.rows_to_csv_text(rows), encoding='utf-8')
        (tmp_path / 'catalog.json').write_text(json.dumps(synthetic.make_catalog()))

    catalog = ft.LocationCatalog.load(tmp_path / 'catalog.json')
    meta = training.train_pipeline(csv_path, tmp_path / name, catalog, seed=seed,
                                   filters=SYNTH_FILTERS)

    return meta, catalog


# 목적지 구간 예측 입력
def _query():
    return {'role': 'TRANSPORTER', 'robot_id': 'robot_2', 'from_location_id': 'loc-syn-receipt-a',
            'to_location_id': 'loc-syn-storage-3', 'to_location_revision': 1, 'map_revision': 1,
            'object_state': 3, 'carrying_object': 1, 'attempt_index': 0,
            'path_length_m': 11.0, 'turn_count': 2}


# 같은 seed·데이터면 모델 파일 바이트와 지표가 같다
def test_deterministic_training(tmp_path):
    first, _ = _train(tmp_path, 'a')
    second, _ = _train(tmp_path, 'b')

    assert first['model']['status'] == 'TRAINED'
    assert (tmp_path / 'a' / md.MODEL_FILE).read_bytes() == \
        (tmp_path / 'b' / md.MODEL_FILE).read_bytes()
    assert first['metrics'] == second['metrics']
    assert first['data'] == second['data']


# 합성 규칙을 경로 특징으로 따라가면 기준선보다 낫다(코드 경로 확인, 성능 근거 아님)
def test_model_beats_baseline_on_synthetic(tmp_path):
    meta, catalog = _train(tmp_path, 'm')

    assert meta['adopted'] is True
    assert meta['performance_evidence'] is False
    assert meta['metrics']['model']['mae_s'] < meta['metrics']['baseline']['mae_s']
    assert 'path_length_m' not in meta['features']['dropped_all_null']

    served = EtaPredictor(tmp_path / 'm', catalog, allow_synthetic=True).predict(_query())
    assert served.source == 'MODEL'
    assert math.isfinite(served.eta_s) and served.eta_s > 0

    strict = EtaPredictor(tmp_path / 'm', catalog).predict(_query())
    assert strict.source.startswith('BASELINE_')
    assert 'SYNTHETIC_MODEL' in strict.reasons


# 저장·로드 왕복 예측이 같다
def test_save_load_round_trip(tmp_path):
    _train(tmp_path, 'm')
    booster = md.load(tmp_path / 'm' / md.MODEL_FILE)
    again = md.load(tmp_path / 'm' / md.MODEL_FILE)

    vector = [[0.0] * booster.num_features()]
    assert md.predict(booster, vector) == md.predict(again, vector)


# 손상 파일·주버전 불일치·비정상 출력은 기준선으로 복귀
def test_model_fallbacks(tmp_path, monkeypatch):
    _, catalog = _train(tmp_path, 'm')
    model_dir = tmp_path / 'm'

    monkeypatch.setattr(md, 'predict', lambda regressor, matrix: [float('nan')])
    result = EtaPredictor(model_dir, catalog, allow_synthetic=True).predict(_query())
    assert 'MODEL_OUTPUT_INVALID' in result.reasons
    assert result.source.startswith('BASELINE_')
    monkeypatch.undo()

    meta_path = model_dir / 'meta.json'
    meta = json.loads(meta_path.read_text())
    meta['model']['xgboost_version'] = '1.7.6'
    meta_path.write_text(json.dumps(meta))
    predictor = EtaPredictor(model_dir, catalog, allow_synthetic=True)
    assert predictor.load_reasons == ['MODEL_NOT_LOADED:XGBOOST_MAJOR_MISMATCH']

    meta['model']['xgboost_version'] = xgboost.__version__
    meta_path.write_text(json.dumps(meta))
    (model_dir / md.MODEL_FILE).write_text('{broken', encoding='utf-8')
    predictor = EtaPredictor(model_dir, catalog, allow_synthetic=True)
    assert predictor.load_reasons == ['MODEL_NOT_LOADED:LOAD_FAILED']
    assert predictor.predict(_query()).source.startswith('BASELINE_')
