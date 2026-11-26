"""task_id 그룹 분할과 지표(MAE·RMSE·작업별 주행 합 오차).

"전체 작업 완료시간"(설계 4.4)은 원장에 조작 단계 시간이 없어 계산하지 않는다. 대신 작업별
주행(NAVIGATE_TO) 소요 합의 오차를 대리 지표로 낸다.
"""
import math

import numpy as np

MIN_GROUPS = 10
DEFAULT_TEST_FRACTION = 0.25


class InsufficientData(Exception):
    """분할·평가에 필요한 작업 수가 부족하다(지표를 만들지 않는다)."""


# 작업 ID 단위로 학습·평가 행을 나눈다. 같은 작업은 한쪽에만 들어간다
def group_split(rows, seed, test_fraction=DEFAULT_TEST_FRACTION, min_groups=MIN_GROUPS):
    groups = sorted({row['task_id'] for row in rows})
    if len(groups) < min_groups:
        raise InsufficientData(f'작업 수 {len(groups)} < {min_groups}')

    order = np.random.RandomState(int(seed)).permutation(len(groups))
    n_test = max(1, int(round(len(groups) * test_fraction)))
    test_groups = {groups[i] for i in order[:n_test]}

    train = [row for row in rows if row['task_id'] not in test_groups]
    test = [row for row in rows if row['task_id'] in test_groups]

    return train, test, sorted(test_groups)


# MAE·RMSE(값 없는 예측은 제외하고 제외 수를 함께 낸다)
def error_metrics(actual, predicted):
    pairs = [(a, p) for a, p in zip(actual, predicted) if p is not None]
    if not pairs:
        return {'n': 0, 'n_missing': len(actual), 'mae_s': None, 'rmse_s': None}

    errors = [p - a for a, p in pairs]

    return {
        'n': len(pairs),
        'n_missing': len(actual) - len(pairs),
        'mae_s': sum(abs(e) for e in errors) / len(errors),
        'rmse_s': math.sqrt(sum(e * e for e in errors) / len(errors)),
    }


# 작업별 주행 합 오차(작업의 모든 행에 예측이 있을 때만)
def task_sum_metrics(rows, actual, predicted):
    totals = {}
    for row, a, p in zip(rows, actual, predicted):
        entry = totals.setdefault(row['task_id'], [0.0, 0.0, True])
        entry[0] += a
        if p is None:
            entry[2] = False
        else:
            entry[1] += p

    complete = [(a, p) for a, p, ok in totals.values() if ok]
    result = error_metrics([a for a, _ in complete], [p for _, p in complete])
    result['n_tasks_incomplete'] = len(totals) - len(complete)

    return result
