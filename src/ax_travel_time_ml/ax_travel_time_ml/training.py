"""학습·평가 흐름: CSV → 필터 → task_id 분할 → 특징·기준선·XGBoost → 보류 세트 지표 → meta.json.

- 최종 모델은 학습 분할로만 학습한다(보고 지표와 같은 모델).
- xgboost가 없으면 기준선만 저장하고 model.status=UNAVAILABLE로 남긴다(성공으로 보고하지 않는다).
- adopted: 보류 세트 MAE가 기준선보다 작을 때만 True. 합성 데이터면 performance_evidence=False.
"""
import hashlib
import json
import math
import subprocess
from pathlib import Path

from . import baseline as bl
from . import dataset as ds
from . import evaluation as ev
from . import features as ft
from . import model as md
from .predict import META_FILE, EtaPredictor

META_SCHEMA = 1
DEFAULT_SEED = 20261126


# 현재 git commit(저장소 밖이거나 실패하면 None)
def _git_commit():
    try:
        result = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True,
                                timeout=5, cwd=Path(__file__).resolve().parent, check=False)
    except (OSError, subprocess.SubprocessError):
        return None

    if result.returncode != 0:
        return None

    return result.stdout.strip() or None


# 문자열 목록 해시(평가 작업 목록 기록용)
def _list_sha256(values):
    return hashlib.sha256('\n'.join(values).encode('utf-8')).hexdigest()


# 학습을 수행하고 meta 사전을 돌려준다. 작업 수가 부족하면 InsufficientData
def train_pipeline(csv_path, out_dir, catalog=None, seed=DEFAULT_SEED, min_n=bl.DEFAULT_MIN_N,
                   test_fraction=ev.DEFAULT_TEST_FRACTION, filters=None, params=None):
    filters = filters or {'mode': ds.DEFAULT_MODE, 'provenance': ds.DEFAULT_PROVENANCE,
                          'status': ds.DEFAULT_STATUS}

    # 1. 읽기·필터·분할
    rows, raw_sha = ds.read_csv(csv_path)
    kept, excluded = ds.filter_rows(rows, **filters)
    train_rows, test_rows, test_tasks = ev.group_split(kept, seed, test_fraction)

    # 2. 특징·기준선(학습 분할만)
    train_features = ft.build_feature_rows(train_rows, catalog)
    train_y = [row[ds.TARGET] for row in train_rows]

    spec = ft.FeatureSpec.fit(train_features)
    baseline = bl.Baseline.fit(train_features, train_y, min_n)

    # 3. XGBoost(없으면 기준선만)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    model_info = {'status': 'UNAVAILABLE', 'file': None, 'params': None, 'xgboost_version': None}

    try:
        matrix = [spec.vector(f)[0] for f in train_features]
        regressor = md.train(matrix, train_y, seed, params)
        md.save(regressor, out_dir / md.MODEL_FILE)
        model_info = {'status': 'TRAINED', 'file': md.MODEL_FILE,
                      'params': md.merged_params(seed, params),
                      'xgboost_version': md.installed_version()}
    except md.ModelUnavailable as error:
        model_info['error'] = str(error)

    # 4. meta(지표 전) 저장 → 예측기로 보류 세트 평가(복귀 경로 포함)
    meta = {
        'schema': META_SCHEMA,
        'tool': 'ax_travel_time_ml',
        'git_commit': _git_commit(),
        'seed': int(seed),
        'target': ds.TARGET,
        'data': {
            'origin': ds.data_origin(kept),
            'raw_sha256': raw_sha,
            'filtered_sha256': ds.normalized_sha256(kept),
            'train_sha256': ds.normalized_sha256(train_rows),
            'test_sha256': ds.normalized_sha256(test_rows),
            'rows_total': len(rows),
            'rows_kept': len(kept),
            'excluded': excluded,
            'filters': filters,
            'n_train': len(train_rows),
            'n_test': len(test_rows),
        },
        'split': {'method': 'task_id_group', 'test_fraction': test_fraction,
                  'n_test_tasks': len(test_tasks), 'test_tasks_sha256': _list_sha256(test_tasks)},
        'catalog': {'sha256': catalog.sha256 if catalog else None,
                    'map_revision': catalog.map_revision if catalog else None},
        'map_revisions': sorted({row['map_revision'] for row in train_rows}),
        'features': spec.to_dict(),
        'currently_null_columns': list(ft.CURRENTLY_NULL),
        'baseline': baseline.to_dict(),
        'model': model_info,
        'adopted': False,
        'performance_evidence': False,
    }
    _write_meta(out_dir, meta)

    metrics = evaluate_rows(out_dir, test_rows, catalog, allow_synthetic=True,
                            force_model=True)
    meta['metrics'] = metrics

    base_mae = metrics['baseline']['mae_s']
    model_mae = metrics['model']['mae_s'] if metrics['model'] else None
    meta['adopted'] = bool(model_mae is not None and base_mae is not None
                           and metrics['model']['n_missing'] == 0 and model_mae < base_mae)
    meta['performance_evidence'] = meta['data']['origin'] == 'live'
    _write_meta(out_dir, meta)

    return meta


# meta.json 쓰기(정렬된 키, 들여쓰기)
def _write_meta(out_dir, meta):
    (Path(out_dir) / META_FILE).write_text(
        json.dumps(meta, ensure_ascii=False, indent=2, sort_keys=True) + '\n', encoding='utf-8')


# 행 목록 평가: 기준선 단독·모델 단독(force_model)·실제 예측기(복귀 포함)의 지표
def evaluate_rows(model_dir, rows, catalog=None, allow_synthetic=False, force_model=False):
    predictor = EtaPredictor(model_dir, catalog, allow_synthetic=allow_synthetic)
    feature_rows = ft.build_feature_rows(rows, catalog)
    previous = ft.previous_durations(rows)
    actual = [row[ds.TARGET] for row in rows]

    # 기준선 단독
    baseline_pred = [predictor.baseline.predict(f)[0] for f in feature_rows]

    # 모델 단독(채택 여부와 무관하게 모델 출력을 본다. 특징 사유가 있으면 값 없음)
    model_pred = None
    if predictor.regressor is not None:
        vectors = [predictor.spec.vector(feature) for feature in feature_rows]
        usable = [index for index, (_, reasons) in enumerate(vectors) if not reasons]
        values = md.predict(predictor.regressor, [vectors[i][0] for i in usable]) if usable else []

        model_pred = [None] * len(feature_rows)
        for index, value in zip(usable, values):
            if math.isfinite(value) and value > 0.0:
                model_pred[index] = bl.clamp(value, feature_rows[index]['straight_distance_m'], [])

    # 실제 예측기 경로(채택·출처 검사 포함). 학습 직후 평가에서는 아직 adopted=False이므로 건너뛴다
    sources = {}
    served = []
    if not force_model:
        for row in rows:
            result = predictor.predict(row, previous[row['execution_id']])
            served.append(result.eta_s)
            sources[result.source] = sources.get(result.source, 0) + 1

    return {
        'n_rows': len(rows),
        'data_origin': ds.data_origin(rows),
        'data_sha256': ds.normalized_sha256(rows),
        'baseline': ev.error_metrics(actual, baseline_pred),
        'baseline_task_sum': ev.task_sum_metrics(rows, actual, baseline_pred),
        'model': ev.error_metrics(actual, model_pred) if model_pred is not None else None,
        'model_task_sum': (ev.task_sum_metrics(rows, actual, model_pred)
                           if model_pred is not None else None),
        'served': ev.error_metrics(actual, served) if served else None,
        'served_sources': dict(sorted(sources.items())),
        'model_load_reasons': predictor.load_reasons,
        'performance_evidence': ds.data_origin(rows) == 'live',
    }
