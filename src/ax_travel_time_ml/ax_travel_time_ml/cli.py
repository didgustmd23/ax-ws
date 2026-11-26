"""명령행 진입점: ax_eta_train, ax_eta_evaluate, ax_eta_synth.

종료 코드: 0 성공, 2 입력 오류(CSV·카탈로그·meta), 3 표본 부족(INSUFFICIENT_DATA, 지표 없음),
4 xgboost 없이 기준선만 저장(모델 학습 안 됨).
"""
import argparse
import json
import sys
from pathlib import Path

from . import baseline as bl
from . import dataset as ds
from . import evaluation as ev
from . import features as ft
from . import synthetic
from . import training
from .predict import MetaError

SYNTHETIC_NOTICE = '합성 데이터 결과다. AX 주행 성능의 근거로 쓰지 않는다.'


# 카탈로그 경로가 있으면 읽는다
def _catalog(path):
    return ft.LocationCatalog.load(path) if path else None


# 표준 오류 출력
def _log(name, message):
    print(f'[{name}] {message}', file=sys.stderr)


# 학습 명령
def train_main(argv=None):
    parser = argparse.ArgumentParser(description='D2 ETA 기준선·XGBoost 학습(task_id 분할 평가 포함)')
    parser.add_argument('--csv', required=True, help='export_d2_eta_samples.py CSV')
    parser.add_argument('--out', required=True, help='출력 폴더(meta.json, model.json)')
    parser.add_argument('--catalog', help='지점 카탈로그 JSON(읽기 전용, 직선거리 특징)')
    parser.add_argument('--seed', type=int, default=training.DEFAULT_SEED)
    parser.add_argument('--min-n', type=int, default=bl.DEFAULT_MIN_N)
    parser.add_argument('--test-fraction', type=float, default=ev.DEFAULT_TEST_FRACTION)
    parser.add_argument('--synthetic', action='store_true',
                        help='mode·provenance=SYNTHETIC 표본으로 학습(코드 시험용)')
    args = parser.parse_args(argv)

    if args.synthetic:
        filters = {'mode': ds.SYNTHETIC, 'provenance': ds.SYNTHETIC, 'status': ds.DEFAULT_STATUS}
    else:
        filters = None

    try:
        meta = training.train_pipeline(args.csv, args.out, _catalog(args.catalog), args.seed,
                                       args.min_n, args.test_fraction, filters)
    except (ds.DatasetError, ft.CatalogError) as error:
        _log('ax_eta_train', error)
        return 2
    except ev.InsufficientData as error:
        _log('ax_eta_train', f'INSUFFICIENT_DATA: {error}. 지표를 만들지 않았다')
        return 3

    summary = {key: meta[key] for key in ('adopted', 'performance_evidence', 'metrics')}
    summary['model_status'] = meta['model']['status']
    summary['dropped_all_null'] = meta['features']['dropped_all_null']
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))

    if meta['data']['origin'] != 'live':
        _log('ax_eta_train', SYNTHETIC_NOTICE)

    if meta['model']['status'] != 'TRAINED':
        _log('ax_eta_train', f'모델 학습 안 됨: {meta["model"].get("error")}. 기준선만 저장')
        return 4

    return 0


# 평가 명령(학습과 별개의 CSV로 기준선·모델·실제 예측 경로 지표)
def evaluate_main(argv=None):
    parser = argparse.ArgumentParser(description='저장된 ETA 모델 평가')
    parser.add_argument('--csv', required=True)
    parser.add_argument('--model-dir', required=True)
    parser.add_argument('--catalog')
    parser.add_argument('--synthetic', action='store_true', help='합성 표본·합성 모델 허용')
    parser.add_argument('-o', '--output', help='metrics JSON 경로')
    args = parser.parse_args(argv)

    if args.synthetic:
        filters = {'mode': ds.SYNTHETIC, 'provenance': ds.SYNTHETIC, 'status': ds.DEFAULT_STATUS}
    else:
        filters = {}

    try:
        rows, raw_sha = ds.read_csv(args.csv)
        kept, excluded = ds.filter_rows(rows, **filters)
        metrics = training.evaluate_rows(args.model_dir, kept, _catalog(args.catalog),
                                         allow_synthetic=args.synthetic)
    except (ds.DatasetError, ft.CatalogError, MetaError) as error:
        _log('ax_eta_evaluate', error)
        return 2

    metrics['raw_sha256'] = raw_sha
    metrics['excluded'] = excluded
    text = json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True)

    if args.output:
        Path(args.output).write_text(text + '\n', encoding='utf-8')
    print(text)

    if not metrics['performance_evidence']:
        _log('ax_eta_evaluate', SYNTHETIC_NOTICE)

    return 0


# 합성 CSV·카탈로그 생성 명령
def synth_main(argv=None):
    parser = argparse.ArgumentParser(description='합성 ETA 표본 생성(코드 시험 전용)')
    parser.add_argument('--out', required=True, help='CSV 경로')
    parser.add_argument('--catalog-out', help='합성 카탈로그 JSON 경로')
    parser.add_argument('--tasks', type=int, default=60)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--with-path-features', action='store_true',
                        help='경로 길이·회전 수 열도 채운다(원장은 현재 NULL)')
    args = parser.parse_args(argv)

    rows = synthetic.make_rows(args.tasks, args.seed, args.with_path_features)
    Path(args.out).write_text(ds.rows_to_csv_text(rows), encoding='utf-8')

    if args.catalog_out:
        Path(args.catalog_out).write_text(
            json.dumps(synthetic.make_catalog(), indent=2) + '\n', encoding='utf-8')

    _log('ax_eta_synth', f'{len(rows)}행. {SYNTHETIC_NOTICE}')

    return 0


# console_scripts 래퍼
def train_entry():
    sys.exit(train_main())


# console_scripts 래퍼
def evaluate_entry():
    sys.exit(evaluate_main())


# console_scripts 래퍼
def synth_entry():
    sys.exit(synth_main())
