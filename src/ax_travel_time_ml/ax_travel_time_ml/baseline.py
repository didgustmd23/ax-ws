"""계층형 기준선 ETA(B1~B5). 설계 4.4의 "기존 방식"으로 쓴다(저장소에 별도 정의가 없어 이것을 정의로 삼는다).

위에서부터 처음 성공한 단계의 값을 쓴다.
- B1: (role, from, to, carrying) 중앙값, 표본 min_n 이상
- B2: (role, to, carrying) 중앙값, 표본 min_n 이상
- B3: 직선거리 d / v_eff + t0. 학습 세트에서 최소제곱 적합(표본 부족·비물리 값이면 하한 근사
      v=0.67 m/s, t0=1.0 s로 두고 lower_bound=True 표시)
- B4: role 중앙값, 표본 min_n 이상
- B5: UNKNOWN(값 없음, 사유 NO_BASELINE)
거리를 알면 모든 예측을 하한 d/0.67 + 1.0 s 이상으로 맞춘다. 0.67 m/s는 diff_drive 직선 속도 상한,
1.0 s는 가속 한계 0.67 m/s²로 상한 속도까지 걸리는 시간이며 실제 주행 시간의 추정이 아니다.
"""
import math
import statistics

DEFAULT_MIN_N = 5
V_MAX_MPS = 0.67
ACCEL_TIME_S = 1.0


# 집계 키 값(None은 문자열 'None'으로 구분해 JSON에 저장 가능하게)
def _key(*values):
    return '|'.join('None' if value is None else str(value) for value in values)


# B1·B2·B4 키
def _keys(feature):
    role = feature['role']
    carrying = feature['carrying_object']

    return {
        'B1': _key(role, feature['from_location_id'], feature['to_location_id'], carrying),
        'B2': _key(role, feature['to_location_id'], carrying),
        'B4': _key(role),
    }


# 거리 기반 하한(거리 모르면 None)
def lower_bound(distance_m):
    if distance_m is None or not math.isfinite(distance_m):
        return None

    return distance_m / V_MAX_MPS + ACCEL_TIME_S


# y = a·d + t0 최소제곱. 표본 부족·a≤0·t0<0이면 None
def _fit_line(pairs, min_n):
    if len(pairs) < min_n:
        return None

    mean_d = statistics.fmean(d for d, _ in pairs)
    mean_y = statistics.fmean(y for _, y in pairs)
    var_d = sum((d - mean_d) ** 2 for d, _ in pairs)
    if var_d <= 1e-9:
        return None

    slope = sum((d - mean_d) * (y - mean_y) for d, y in pairs) / var_d
    intercept = mean_y - slope * mean_d
    if slope <= 0.0 or intercept < 0.0:
        return None

    return {'v_eff_mps': 1.0 / slope, 't0_s': intercept, 'lower_bound': False, 'n': len(pairs)}


class Baseline:
    """학습 세트 통계로 만든 계층형 기준선."""

    # 저장 사전 또는 fit 결과로 만든다
    def __init__(self, medians, counts, line, min_n=DEFAULT_MIN_N):
        self.medians = medians
        self.counts = counts
        self.line = line
        self.min_n = min_n

    # 특징 사전 목록과 목표값으로 적합한다
    @classmethod
    def fit(cls, feature_rows, targets, min_n=DEFAULT_MIN_N):
        groups = {'B1': {}, 'B2': {}, 'B4': {}}
        pairs = []

        for feature, target in zip(feature_rows, targets):
            for level, key in _keys(feature).items():
                groups[level].setdefault(key, []).append(target)

            distance = feature['straight_distance_m']
            if distance is not None:
                pairs.append((distance, target))

        medians = {
            level: {key: statistics.median(values) for key, values in group.items()}
            for level, group in groups.items()
        }
        counts = {
            level: {key: len(values) for key, values in group.items()}
            for level, group in groups.items()
        }

        line = _fit_line(pairs, min_n)
        if line is None:
            line = {'v_eff_mps': V_MAX_MPS, 't0_s': ACCEL_TIME_S, 'lower_bound': True,
                    'n': len(pairs)}

        return cls(medians, counts, line, min_n)

    # 예측. (eta_s 또는 None, 출처 'BASELINE_B1'~'UNKNOWN', 사유 목록)
    def predict(self, feature):
        reasons = []
        keys = _keys(feature)
        eta = None
        source = 'UNKNOWN'

        for level in ('B1', 'B2'):
            if self.counts[level].get(keys[level], 0) >= self.min_n:
                eta, source = self.medians[level][keys[level]], f'BASELINE_{level}'
                break
            reasons.append(f'{level}_INSUFFICIENT')

        distance = feature['straight_distance_m']
        if eta is None and distance is not None:
            eta = distance / self.line['v_eff_mps'] + self.line['t0_s']
            source = 'BASELINE_B3'
            if self.line['lower_bound']:
                reasons.append('B3_LOWER_BOUND_ONLY')
        elif eta is None:
            reasons.append('B3_NO_DISTANCE')

        if eta is None and self.counts['B4'].get(keys['B4'], 0) >= self.min_n:
            eta, source = self.medians['B4'][keys['B4']], 'BASELINE_B4'
        elif eta is None:
            reasons.append('B4_INSUFFICIENT')

        if eta is None:
            reasons.append('NO_BASELINE')
            return None, 'UNKNOWN', reasons

        return clamp(eta, distance, reasons), source, reasons

    # 저장용 사전
    def to_dict(self):
        return {'min_n': self.min_n, 'medians': self.medians, 'counts': self.counts,
                'line': self.line}

    # 저장 사전에서 복원
    @classmethod
    def from_dict(cls, data):
        return cls(data['medians'], data['counts'], data['line'], data['min_n'])


# 거리 하한 적용(올리면 사유 기록)
def clamp(eta, distance_m, reasons):
    floor = lower_bound(distance_m)
    if floor is not None and eta < floor:
        reasons.append('CLAMPED_TO_LOWER_BOUND')
        return floor

    return eta
