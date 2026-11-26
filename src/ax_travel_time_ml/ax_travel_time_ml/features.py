"""ETA 특징 생성: 지점 카탈로그 직선거리, 같은 작업의 직전 주행 소요, 범주 one-hot.

특징 구분(현재 원장 기준):
- 지금 쓸 수 있음: role, robot_id, from/to_location_id(출발지 NULL은 UNKNOWN_ORIGIN),
  첫 구간 여부, carrying_object, object_state, attempt_index
- 파생(조건부): straight_distance_m(카탈로그 역할별 정지 자세, 출발지 NULL·revision 불일치면 결측),
  prev_nav_duration_s(같은 작업·같은 로봇의 앞선 주행 소요. sim 시각은 세션마다 다시 시작하므로
  다른 작업과 비교하지 않는다)
- 현재 항상 NULL: path_length_m, turn_count, corridor_wait_s, obstacle_density.
  학습 세트에서 전부 결측인 열은 빼고 dropped_all_null로 기록한다(값이 생기면 재학습 때 들어간다)
"""
import hashlib
import json
import math

UNKNOWN_ORIGIN = 'UNKNOWN_ORIGIN'

CATEGORICAL = ('role', 'robot_id', 'from_location_id', 'to_location_id', 'object_state')
NUMERIC = (
    'first_leg', 'carrying_object', 'attempt_index', 'straight_distance_m',
    'prev_nav_duration_s', 'path_length_m', 'turn_count', 'corridor_wait_s', 'obstacle_density',
)
# 현재 원장에서 항상 NULL인 열(보고용)
CURRENTLY_NULL = ('path_length_m', 'turn_count', 'corridor_wait_s', 'obstacle_density')
# 모델 사용 전제(없으면 기준선으로 복귀)
MODEL_REQUIRED = ('role', 'to_location_id', 'carrying_object')


class CatalogError(Exception):
    """지점 카탈로그를 읽을 수 없거나 형식이 다르다."""


class LocationCatalog:
    """지점 ID → (revision, 역할별 정지 자세). map_revision을 함께 가진다."""

    # 카탈로그 사전에서 만든다. dev_intake.json처럼 "catalog" 아래에 있어도 된다
    def __init__(self, data, sha256=None):
        body = data.get('catalog', data) if isinstance(data, dict) else None
        if not isinstance(body, dict) or not isinstance(body.get('locations'), dict):
            raise CatalogError('catalog.locations 사전이 없다')

        self.map_revision = body.get('map_revision')
        self.sha256 = sha256
        self.locations = {}

        for location_id, entry in body['locations'].items():
            poses = entry.get('stop_poses') or {}
            fallback = entry.get('pose')
            self.locations[location_id] = {
                'revision': entry.get('location_revision'),
                'stop_poses': {role: (float(p['x']), float(p['y'])) for role, p in poses.items()},
                'pose': (float(fallback['x']), float(fallback['y'])) if fallback else None,
            }

    # 파일에서 읽는다(원본 바이트 해시 기록)
    @classmethod
    def load(cls, path):
        try:
            with open(path, 'rb') as stream:
                raw = stream.read()
            data = json.loads(raw.decode('utf-8'))
        except (OSError, ValueError) as error:
            raise CatalogError(f'카탈로그 읽기 실패: {path}: {error}') from error

        return cls(data, hashlib.sha256(raw).hexdigest())

    # 역할의 정지 자세(없으면 지점 자세). 지점·revision이 맞지 않으면 None
    def _point(self, location_id, role, revision=None):
        entry = self.locations.get(location_id)
        if entry is None:
            return None

        if revision is not None and entry['revision'] != revision:
            return None

        return entry['stop_poses'].get(role) or entry['pose']

    # 두 지점 사이 직선거리(m). 출발지 없음·미지 지점·revision 불일치면 None
    def distance(self, role, from_id, to_id, to_revision=None, map_revision=None):
        if from_id is None:
            return None

        if map_revision is not None and self.map_revision is not None \
                and map_revision != self.map_revision:
            return None

        start = self._point(from_id, role)
        end = self._point(to_id, role, to_revision)
        if start is None or end is None:
            return None

        return math.hypot(end[0] - start[0], end[1] - start[1])


# 같은 작업·같은 로봇에서 현재 전송 전에 끝난 주행 중 가장 늦게 끝난 것의 소요(없으면 None)
def previous_durations(rows):
    by_key = {}
    for row in rows:
        by_key.setdefault((row['task_id'], row['robot_id']), []).append(row)

    result = {}
    for row in rows:
        sent = row.get('sent_sim_s')
        best = None

        for other in by_key[(row['task_id'], row['robot_id'])]:
            done = other.get('finished_sim_s')
            if other is row or sent is None or done is None or done > sent:
                continue

            if best is None or done > best['finished_sim_s']:
                best = other

        result[row['execution_id']] = best['duration_sim_s'] if best else None

    return result


# 원시 행 → 특징 사전(범주는 문자열, 수치는 float 또는 None)
def raw_features(row, catalog=None, prev_duration=None):
    from_id = row.get('from_location_id')
    carrying = row.get('carrying_object')

    distance = None
    if catalog is not None:
        distance = catalog.distance(row['role'], from_id, row['to_location_id'],
                                    row.get('to_location_revision'), row.get('map_revision'))

    return {
        'role': row.get('role'),
        'robot_id': row.get('robot_id'),
        'from_location_id': from_id if from_id is not None else UNKNOWN_ORIGIN,
        'to_location_id': row.get('to_location_id'),
        'object_state': None if row.get('object_state') is None else str(row['object_state']),
        'first_leg': 1.0 if from_id is None else 0.0,
        'carrying_object': None if carrying is None else float(carrying),
        'attempt_index': None if row.get('attempt_index') is None else float(row['attempt_index']),
        'straight_distance_m': distance,
        'prev_nav_duration_s': prev_duration,
        'path_length_m': row.get('path_length_m'),
        'turn_count': None if row.get('turn_count') is None else float(row['turn_count']),
        'corridor_wait_s': row.get('corridor_wait_s'),
        'obstacle_density': row.get('obstacle_density'),
    }


# 수치 값이 쓸 수 있는지(None·NaN·무한 제외)
def _present(value):
    return value is not None and isinstance(value, float) and math.isfinite(value)


class FeatureSpec:
    """학습 세트에서 고정한 특징 열 순서·범주 목록·제외 열."""

    # 저장된 사전에서 복원하거나 fit으로 만든다
    def __init__(self, categories, numeric, dropped_all_null):
        self.categories = {name: list(values) for name, values in categories.items()}
        self.numeric = list(numeric)
        self.dropped_all_null = list(dropped_all_null)

    # 학습 특징 사전 목록으로 범주·수치 열을 정한다
    @classmethod
    def fit(cls, feature_rows):
        categories = {}
        for name in CATEGORICAL:
            values = sorted({str(f[name]) for f in feature_rows if f[name] is not None})
            categories[name] = values

        numeric = []
        dropped = []
        for name in NUMERIC:
            if any(_present(f[name]) for f in feature_rows):
                numeric.append(name)
            else:
                dropped.append(name)

        return cls(categories, numeric, dropped)

    # 열 이름 순서(모델 입력 순서)
    @property
    def columns(self):
        names = []
        for name in CATEGORICAL:
            names.extend(f'{name}={value}' for value in self.categories[name])

        return names + self.numeric

    # 특징 사전 하나를 벡터로. (벡터, 사유 목록). 사유가 있으면 모델을 쓰지 않는다
    def vector(self, feature):
        reasons = []

        for name in MODEL_REQUIRED:
            if feature.get(name) is None:
                reasons.append(f'MISSING_FEATURE:{name}')

        values = []
        for name in CATEGORICAL:
            current = feature.get(name)
            known = self.categories[name]

            if current is not None and str(current) not in known:
                reasons.append(f'UNSEEN_CATEGORY:{name}')

            values.extend(1.0 if current is not None and str(current) == v else 0.0 for v in known)

        for name in self.numeric:
            value = feature.get(name)
            values.append(float(value) if _present(value) else float('nan'))

        return values, reasons

    # 저장용 사전
    def to_dict(self):
        return {
            'categorical': self.categories,
            'numeric': self.numeric,
            'dropped_all_null': self.dropped_all_null,
            'columns': self.columns,
        }

    # 저장 사전에서 복원(열 순서가 다시 계산한 것과 다르면 오류)
    @classmethod
    def from_dict(cls, data):
        spec = cls(data['categorical'], data['numeric'], data['dropped_all_null'])
        if spec.columns != list(data['columns']):
            raise ValueError('특징 열 순서 불일치')

        return spec


# 행 목록 → 특징 사전 목록(직전 주행 소요는 같은 목록 안에서 계산)
def build_feature_rows(rows, catalog=None):
    previous = previous_durations(rows)

    return [raw_features(row, catalog, previous[row['execution_id']]) for row in rows]
