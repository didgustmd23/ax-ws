"""XGBoost 회귀(Regressor) 학습·저장·로드. xgboost는 이 모듈에서만 필요할 때 import한다.

sklearn 래퍼(XGBRegressor)는 scikit-learn을 요구하므로 쓰지 않고, 같은 회귀 목적함수의
네이티브 API(xgboost.train·Booster)를 쓴다.

저장 형식: XGBoost 네이티브 JSON(`model.json`, pickle 사용 안 함). 특징·기준선·데이터 해시는
같은 폴더의 `meta.json`에 둔다(cli.py).
"""
import numpy as np

MODEL_FILE = 'model.json'

# 고정 매개변수(소표본이라 early stopping·탐색 없음. nthread=1은 재현용)
DEFAULT_PARAMS = {
    'objective': 'reg:squarederror',
    'num_boost_round': 200,
    'max_depth': 4,
    'eta': 0.1,
    'subsample': 0.8,
    'tree_method': 'hist',
    'nthread': 1,
}


class ModelUnavailable(Exception):
    """xgboost가 없거나 모델 파일을 쓸 수 없다. code는 복귀 사유 코드."""

    # 메시지와 사유 코드를 함께 둔다
    def __init__(self, message, code='LOAD_FAILED'):
        super().__init__(message)
        self.code = code


# xgboost 모듈을 돌려준다(없으면 ModelUnavailable)
def import_xgboost():
    try:
        import xgboost
    except ImportError as error:
        raise ModelUnavailable(f'xgboost import 실패: {error}', 'XGBOOST_MISSING') from error

    return xgboost


# 주버전 문자열(호환 검사용)
def major_version(version):
    return str(version).split('.')[0]


# 학습에 실제로 쓴 매개변수(기본값 + 덮어쓰기 + seed)
def merged_params(seed, params=None):
    merged = dict(DEFAULT_PARAMS)
    merged.update(params or {})
    merged['seed'] = int(seed)

    return merged


# 학습. X는 2차원 목록(NaN 허용, 결측으로 처리), y는 목표값 목록
def train(matrix, targets, seed, params=None):
    xgboost = import_xgboost()

    merged = merged_params(seed, params)
    rounds = merged.pop('num_boost_round')

    data = xgboost.DMatrix(np.asarray(matrix, dtype=float), label=np.asarray(targets, dtype=float),
                           missing=float('nan'), nthread=1)

    return xgboost.train(merged, data, num_boost_round=rounds)


# 모델 저장(네이티브 JSON)
def save(regressor, path):
    regressor.save_model(str(path))


# 모델 로드. 파일 없음·손상·xgboost 없음은 ModelUnavailable
def load(path):
    xgboost = import_xgboost()
    regressor = xgboost.Booster()

    try:
        regressor.load_model(str(path))
    except Exception as error:  # xgboost는 손상 파일에 XGBoostError 등 여러 예외를 낸다
        raise ModelUnavailable(f'모델 로드 실패: {path}: {error}') from error

    return regressor


# 예측(벡터 목록 → float 목록)
def predict(regressor, matrix):
    xgboost = import_xgboost()
    data = xgboost.DMatrix(np.asarray(matrix, dtype=float), missing=float('nan'), nthread=1)

    return [float(v) for v in regressor.predict(data)]


# 설치된 xgboost 버전(없으면 None)
def installed_version():
    try:
        return import_xgboost().__version__
    except ModelUnavailable:
        return None
