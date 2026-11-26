"""ETA 예측: 조건을 만족하면 XGBoost, 아니면 계층형 기준선으로 복귀한다.

모델을 쓰지 않는 경우(사유 코드로 돌려준다, 예외를 삼키지 않는다):
- MODEL_NOT_LOADED:<원인>: meta·모델 파일 없음/손상, xgboost 없음, 주버전 불일치, 특징 순서 불일치
- MODEL_NOT_ADOPTED: 보류 세트에서 기준선보다 낫지 않았다
- SYNTHETIC_MODEL: 합성 데이터로 학습한 모델(allow_synthetic=True일 때만 사용, 시험용)
- CATALOG_MISMATCH: 학습 때와 다른 지점 카탈로그
- MAP_REVISION_UNSEEN: 학습에 없던 map_revision
- MISSING_FEATURE:*, UNSEEN_CATEGORY:*: 특징 결측·미지 범주
- MODEL_OUTPUT_INVALID: NaN·무한·0 이하 출력
"""
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from . import baseline as bl
from . import features as ft
from . import model as md

META_FILE = 'meta.json'


@dataclass
class EtaResult:
    """예측 결과. eta_s가 None이면 UNKNOWN(배정 판단에 쓰지 않는다)."""

    eta_s: float = None
    source: str = 'UNKNOWN'
    reasons: list = field(default_factory=list)


class EtaPredictor:
    """meta.json(+model.json) 폴더로 만든다. 모델이 없어도 기준선으로 동작한다."""

    # 폴더를 읽는다. meta가 없거나 깨지면 MetaError(기준선도 없으므로 쓸 수 없다)
    def __init__(self, model_dir, catalog=None, allow_synthetic=False):
        self.model_dir = Path(model_dir)
        self.catalog = catalog
        self.allow_synthetic = allow_synthetic

        self.meta = load_meta(self.model_dir)
        self.spec = ft.FeatureSpec.from_dict(self.meta['features'])
        self.baseline = bl.Baseline.from_dict(self.meta['baseline'])

        self.regressor = None
        self.load_reasons = []
        self.load_error = None
        self._load_model()

    # 모델 파일 로드 시도(실패 사유는 load_reasons에 남긴다)
    def _load_model(self):
        info = self.meta.get('model', {})
        if info.get('status') != 'TRAINED':
            self.load_reasons.append(f'MODEL_NOT_LOADED:{info.get("status", "NO_MODEL")}')
            return

        try:
            version = md.installed_version()
            if version is None:
                raise md.ModelUnavailable('xgboost 없음', 'XGBOOST_MISSING')

            if md.major_version(version) != md.major_version(info.get('xgboost_version')):
                raise md.ModelUnavailable(f'xgboost {version} ≠ 학습 {info.get("xgboost_version")}',
                                          'XGBOOST_MAJOR_MISMATCH')

            self.regressor = md.load(self.model_dir / info['file'])
        except md.ModelUnavailable as error:
            self.load_reasons.append(f'MODEL_NOT_LOADED:{error.code}')
            self.load_error = str(error)

    # 모델 사용 전 조건 검사(사유 목록, 비면 사용 가능)
    def _model_blockers(self, row, feature):
        reasons = list(self.load_reasons)

        if self.regressor is not None and not self.meta.get('adopted'):
            reasons.append('MODEL_NOT_ADOPTED')

        if self.meta['data']['origin'] != 'live' and not self.allow_synthetic:
            reasons.append('SYNTHETIC_MODEL')

        trained_catalog = self.meta['catalog']['sha256']
        current_catalog = self.catalog.sha256 if self.catalog else None
        if trained_catalog != current_catalog:
            reasons.append('CATALOG_MISMATCH')

        if row.get('map_revision') not in self.meta['map_revisions']:
            reasons.append('MAP_REVISION_UNSEEN')

        _, vector_reasons = self.spec.vector(feature)
        reasons.extend(vector_reasons)

        return reasons

    # 한 주행 구간의 ETA. row는 eta_samples 행과 같은 키(시각·결과 열은 없어도 된다)
    def predict(self, row, prev_nav_duration_s=None):
        feature = ft.raw_features(row, self.catalog, prev_nav_duration_s)
        blockers = self._model_blockers(row, feature)

        if not blockers:
            vector, _ = self.spec.vector(feature)
            value = md.predict(self.regressor, [vector])[0]

            if math.isfinite(value) and value > 0.0:
                reasons = []
                eta = bl.clamp(value, feature['straight_distance_m'], reasons)
                return EtaResult(eta, 'MODEL', reasons)

            blockers.append('MODEL_OUTPUT_INVALID')

        eta, source, reasons = self.baseline.predict(feature)

        return EtaResult(eta, source, blockers + reasons)


class MetaError(Exception):
    """meta.json이 없거나 형식이 다르다."""


# meta.json 읽기
def load_meta(model_dir):
    path = Path(model_dir) / META_FILE

    try:
        meta = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise MetaError(f'meta 읽기 실패: {path}: {error}') from error

    for key in ('features', 'baseline', 'data', 'catalog', 'map_revisions', 'model'):
        if key not in meta:
            raise MetaError(f'meta 키 없음: {key}')

    return meta
