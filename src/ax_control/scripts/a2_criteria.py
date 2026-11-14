#!/usr/bin/env python3
"""A2 정지·바퀴 odom 결과를 기준 파일(stop_odom_criteria_dev.yaml)로 판정한다(1.5단계).

ROS·Gazebo 없이 동작하는 순수 판정 함수, 과거 결과 재판정, 여러 회차 묶음 판정을 둔다.
check_stop_feedback.py·check_wheel_calibration.py가 --criteria·--rejudge에서 이 모듈을 쓴다.

항목 판정 값
  PASS·FAIL: 기준과 비교한 결과. 비교는 경계를 포함한다(≤, ≥).
  not_judged: 판정해야 하지만 값·조건·지표가 없어 못 한 항목(사유 필수). 값이 비유한이어도 여기다.
  not_applicable: 이 회차 조건에서 대상이 아닌 항목(다른 정지 방식, 팔 없음, 주행 시나리오 아님 등).
  recorded: 기준 파일 record_only에 있는 항목. 판정하지 않고 값만 남긴다.

회차 전체 판정
  status가 completed가 아니면 not_judged("status incomplete").
  기록 표본 최대 간격이 max_sample_interval_s를 넘으면 not_judged("SAMPLE_GAP").
  FAIL 항목이 하나라도 있으면 FAIL. not_judged 항목이 없으면(not_applicable·recorded 제외) PASS.
  그 밖은 not_judged이며 judged_subset으로 판정한 항목만의 결과를 따로 적는다.
  rtf_min이 null이면 RTF로 판정·무효 처리하지 않는다(기록만).

묶음 판정(여러 회차)
  각 회차를 위 규칙으로 판정한 뒤, 기준 항목 ID마다 PASS·FAIL로 판정된 회차가 하나 이상 있으면 충족이다.
  같은 ID의 세부 지표 중 어느 회차에서도 판정되지 않은 것이 있으면 미충족이다(not_applicable로만 나온 세부 지표는 뺌).
  모든 회차에서 recorded뿐인 ID는 판정 대상이 아니다. not_applicable뿐인 ID는 판정된 적이 없으므로 미충족이다.
  A2-S7 반복 수는 로봇·정지 방식·하중·동시 로봇 수·시나리오 묶음별로 센다. 이 조건을 결과에서 읽을 수 없으면 not_judged.
  LD-1은 결과에 부하 단계(active_robot_count·world_sha256)가 없으면 not_judged "부하 단계 정보 없음(3단계)".
  두 묶음 항목은 coverage에서 같은 ID의 세부 지표로 합쳐 ID 하나로 나온다(미충족 목록에도 한 번).
  부하 단계 정보는 측정 도구가 conditions에 남긴다(active_robot_ids·active_robot_count·world_file·world_sha256·renderer).
  FAIL 회차·묶음 항목이 하나라도 있으면 FAIL, 미충족이 없으면 PASS, 그 밖은 not_judged다.

명령(ROS 없이)
  python3 a2_criteria.py [stop|wheel] --rejudge <result.json> --criteria <기준 파일>
    원본은 바꾸지 않고 같은 폴더에 <원본 이름(확장자 제외)>.judged.json을 쓴다(있으면 그 파일만 덮어씀).
  python3 a2_criteria.py --batch <result.json>... --criteria <기준 파일> --output <묶음.json>
    입력 원본은 바꾸지 않는다. 출력 파일이 이미 있으면 쓰지 않는다.
  종류(stop|wheel)를 생략하면 결과 형식(runs / turns·drives)으로 정한다.
"""

import argparse
import copy
import hashlib
import json
import math
import os
import re
import sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import yaml

PASS = 'PASS'
FAIL = 'FAIL'
NOT_JUDGED = 'not_judged'
NOT_APPLICABLE = 'not_applicable'
RECORDED = 'recorded'
JUDGED = (PASS, FAIL)

# 명령 값 비교 허용 오차. 도구가 같은 상수로 만든 명령인지 보는 값이다.
COMMAND_TOLERANCE = 1e-9

# 지표가 아직 없는 항목의 공통 사유
NO_METRIC_STAGE2 = '지표 없음(2단계)'
BATCH_ONLY = '묶음 판정 항목(--batch)'
NO_LOAD_STAGE = '부하 단계 정보 없음(3단계)'

# 판정 대상 상태. check_stop_feedback·check_wheel_calibration 모두 정상 종료면 completed다.
COMPLETED = 'completed'

# 묶음 판정에서 반복 수를 셀 조건 키(결과 conditions 기준). 동시 로봇 수는 active_robot_count로 기록돼야 한다.
REPEAT_GROUP_FIELDS = ('robot_id', 'stop_mode', 'payload_kg', 'active_robot_count')


# 공통 주행 시나리오 목록(이름, 선속도, 각속도, 명령 시간 s). 저속 비교 조건은 유지하고 상한은 현재 공통 설정을 쓴다.
# check_stop_feedback.py가 실행에 쓰고, 재판정은 기록된 상한으로 같은 이름·명령을 다시 만든다.
def drive_scenarios(maximum_linear, maximum_angular):
    return [
        ('직진 0.3 m/s', 0.3, 0.0, 3.0),
        (f'직진 상한 {maximum_linear:g} m/s', maximum_linear, 0.0, 3.0),
        (f'제자리 회전 상한 {maximum_angular:g} rad/s', 0.0, maximum_angular, 3.0),
        ('호 주행', 0.3, 0.3, 3.0),
    ]


# 파일 내용의 sha256을 구한다
def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# 기준 파일을 읽어 (기준 사전, 기록용 출처 사전)을 돌려준다. 판정에 쓰는 절이 없으면 ValueError다.
def load_criteria(path):
    path = Path(path).resolve()
    criteria = yaml.safe_load(path.read_text(encoding='utf-8'))

    if not isinstance(criteria, dict):
        raise ValueError(f'{path}: 기준 파일이 사전 형식이 아님')

    missing = [key for key in ('theoretical_deceleration', 'stop', 'odom', 'load') if key not in criteria]

    if missing:
        raise ValueError(f'{path}: 기준 절 없음 {missing}')

    record_only = criteria.get('record_only') or []

    if not isinstance(record_only, list) or not all(isinstance(item, str) for item in record_only):
        raise ValueError(f'{path}: record_only는 항목 ID 문자열 목록이어야 함')

    return criteria, {'path': str(path), 'sha256': file_sha256(path)}


# 유한한 실수인지 확인한다(bool 제외)
def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


# 판정 항목 하나를 만든다
def make_item(item_id, metric, value, limit, result, reason=''):
    return {
        'id': item_id,
        'metric': metric,
        'value': value,
        'limit': limit,
        'result': result,
        'reason': reason,
    }


# 값 하나를 상한(≤)과 비교한 항목을 만든다. 값이 없거나 유한하지 않으면 not_judged다.
def upper_item(item_id, metric, value, limit, missing_reason='값 없음'):
    if not finite(value):
        return make_item(item_id, metric, value, limit, NOT_JUDGED, missing_reason)

    if not finite(limit):
        return make_item(item_id, metric, value, limit, NOT_JUDGED, '기준 값 없음')

    passed = value <= limit

    return make_item(
        item_id, metric, value, limit,
        PASS if passed else FAIL,
        '' if passed else f'{value:.6g} > {limit:.6g}',
    )


# 값 목록이 모두 [하한, 상한] 안인지 본 항목을 만든다. 값이 비었거나 유한하지 않은 값이 있으면 not_judged다.
def range_item(item_id, metric, values, minimum, maximum):
    limit = [minimum, maximum]

    if not values:
        return make_item(item_id, metric, None, limit, NOT_JUDGED, '값 없음')

    invalid = [index for index, value in enumerate(values) if not finite(value)]

    if invalid:
        return make_item(
            item_id, metric, values, limit, NOT_JUDGED,
            f'유효하지 않은 값 {len(invalid)}개(회차 순번 {invalid})',
        )

    value = {'min': min(values), 'max': max(values), 'count': len(values)}
    outside = [item for item in values if not minimum <= item <= maximum]

    if outside:
        return make_item(
            item_id, metric, value, limit, FAIL,
            f'범위 밖 {len(outside)}개: {", ".join(f"{item:.6f}" for item in outside)}',
        )

    return make_item(item_id, metric, value, limit, PASS)


# record_only 항목을 recorded로 바꾼다. 값은 남기고, 원래 기준 값은 참고로 limit에 둔다.
# not_applicable 항목은 그대로 둔다(대상이 아닌 항목은 기록 대상도 아니다).
def apply_record_only(items, record_only):
    for item in items:
        if item['id'] not in record_only or item['result'] == NOT_APPLICABLE:
            continue

        note = '판정 안 함(기준 파일 record_only, limit은 참고 값)'
        item['result'] = RECORDED
        item['reason'] = note + ('' if finite(item['value']) else '; 값 없음')

    return items


# 항목 목록과 회차 상태로 전체 판정을 만든다. 반환 사전은 결과 기록에 그대로 더한다.
def overall(items, status, sample_gap=None, gap_limit=None):
    result = {}
    failed = [item['id'] for item in items if item['result'] == FAIL]
    skipped = [item['id'] for item in items if item['result'] == NOT_JUDGED]

    if status != COMPLETED:
        result['judgment'] = NOT_JUDGED
        result['judgment_reason'] = f'status incomplete (status={status})'

        return result

    if finite(sample_gap) and finite(gap_limit) and sample_gap > gap_limit:
        result['judgment'] = NOT_JUDGED
        result['judgment_reason'] = f'SAMPLE_GAP: 기록 표본 최대 간격 {sample_gap:.4f} s > {gap_limit} s'

        return result

    if failed:
        result['judgment'] = FAIL
        result['judgment_reason'] = f'FAIL 항목: {", ".join(dict.fromkeys(failed))}'

        return result

    if skipped:
        result['judgment'] = NOT_JUDGED
        result['judged_subset'] = PASS
        result['judgment_reason'] = f'not_judged 항목: {", ".join(dict.fromkeys(skipped))}'

        return result

    result['judgment'] = PASS
    result['judgment_reason'] = '판정 항목 모두 PASS(not_applicable·recorded 제외)'

    return result


# 명령(선·각속도)의 이론 감속 시간 T(s) = 성분별 명령 속도 / 감속 제한 중 긴 쪽
def theoretical_stop_time(linear, angular, deceleration):
    linear_limit = deceleration['linear_deceleration_limit_m_s2']
    angular_limit = deceleration['angular_deceleration_limit_rad_s2']

    return max(abs(linear) / linear_limit, abs(angular) / angular_limit)


# 명령 값과 같은 기준 시나리오 이름을 찾는다. 없으면 None
def match_scenario(linear, angular, scenarios):
    for name, values in scenarios.items():
        linear_match = abs(values['linear_m_s'] - abs(linear)) <= COMMAND_TOLERANCE
        angular_match = abs(values['angular_rad_s'] - abs(angular)) <= COMMAND_TOLERANCE

        if linear_match and angular_match:
            return name

    return None


# 결과 기록의 상한 설정으로 도구 시나리오 이름 → (선속도, 각속도)를 다시 만든다. 상한 기록이 없으면 빈 사전
def recorded_commands(record):
    limits = (record.get('conditions') or {}).get('scenario_limits_from_config') or {}
    linear = limits.get('linear.x.max_velocity')
    angular = limits.get('angular.z.max_velocity')

    if not finite(linear) or not finite(angular):
        return {}

    return {name: (lin, ang) for name, lin, ang, _ in drive_scenarios(linear, angular)}


# 결과 기록의 감속 제한이 기준 파일 값과 같은지 본다. 다르면 사유 문자열, 같거나 기록이 없으면 None
def deceleration_mismatch(record, deceleration):
    limits = (record.get('conditions') or {}).get('scenario_limits_from_config') or {}
    pairs = (
        ('linear.x.max_deceleration', deceleration['linear_deceleration_limit_m_s2']),
        ('angular.z.max_deceleration', deceleration['angular_deceleration_limit_rad_s2']),
    )

    for key, expected in pairs:
        value = limits.get(key)

        if finite(value) and abs(abs(value) - expected) > COMMAND_TOLERANCE:
            return f'조건 불일치: 기록 {key} {value} ≠ 기준 {expected}'

    return None


# 한 시나리오의 정지 회차에서 key 값을 모은다(정지 미확인 회차 None은 뺀다)
def run_values(runs, key, transform=None):
    values = [run.get(key) for run in runs if run is not None]

    return [transform(value) if transform and finite(value) else value for value in values]


# 값 목록의 최댓값. 유한하지 않은 값이 있거나 비었으면 None
def worst(values):
    if not values or not all(finite(value) for value in values):
        return None

    return max(values)


# 결과 기록의 정지 방식. 기능 도입 전 기록(필드 없음)은 None이다
def record_stop_mode(record):
    return record.get('stop_mode') or (record.get('conditions') or {}).get('stop_mode')


# 시나리오가 판정 대상인지 정한다. (결과, 사유, 기준 시나리오 이름)을 돌려주며 대상이면 결과가 None이다.
# 팔 시나리오처럼 주행 명령이 아닌 것은 not_applicable, 명령·조건을 확인할 수 없으면 not_judged다.
def scenario_scope(name, runs, commands, criteria, mismatch):
    scenarios = criteria['theoretical_deceleration'].get('scenarios') or {}

    if not commands:
        return NOT_JUDGED, '명령 값 기록 없음(conditions.scenario_limits_from_config)', None

    if name not in commands:
        return NOT_APPLICABLE, '주행 시나리오 아님(팔 시나리오 등)', None

    reference = match_scenario(*commands[name], scenarios)

    if reference is None:
        return NOT_JUDGED, f'기준 시나리오 매핑 없음(명령 {commands[name]})', None

    if mismatch:
        return NOT_JUDGED, mismatch, reference

    if not any(run is not None for run in runs):
        return NOT_JUDGED, '정지 확인 회차 없음(A2-S7 참조)', reference

    return None, '', reference


# 정지 방식 전용 항목(S1·S2)의 대상 여부. 대상이면 None, 아니면 (결과, 사유)
def mode_scope(item_id, wanted, stop_mode):
    if stop_mode is None:
        return NOT_JUDGED, 'stop_mode 기록 없음(정지 방식 기능 도입 전 기록)'

    if stop_mode != wanted:
        return NOT_APPLICABLE, f'stop_mode {stop_mode}: {item_id}는 {wanted} 전용'

    return None


# 시나리오 하나의 A2-S1·S2·S4·S5 항목을 만든다
def stop_scenario_items(name, runs, commands, stop_mode, criteria, record, mismatch):
    stop = criteria['stop']
    deceleration = criteria['theoretical_deceleration']
    scope, scope_reason, reference = scenario_scope(name, runs, commands, criteria, mismatch)
    label = f'{name}' + (f' → {reference}' if reference else '')
    timeout = stop['command_loss_cmd_vel_timeout_s']
    truth_stop = worst(run_values(runs, 'truth_stop_s'))
    items = []

    if scope is None:
        t_value = theoretical_stop_time(*commands[name], deceleration)
        listed = deceleration['scenarios'][reference].get('t_s')
        t_text = f'T {t_value:.4f} s(기준 파일 표기 {listed})'

    # A2-S1 zero_command: 실제 기준 D-06 진입 ≤ T + 여유
    metric = f'{label}: 실제 정지 truth_stop_s 최대(zero_command)'
    mode = mode_scope('A2-S1', 'zero_command', stop_mode)

    if scope is not None:
        items.append(make_item('A2-S1', metric, None, None, scope, scope_reason))
    elif mode is not None:
        items.append(make_item('A2-S1', metric, None, None, *mode))
    else:
        item = upper_item('A2-S1', metric, truth_stop, t_value + stop['zero_command_margin_s'])
        item['reason'] = '; '.join(filter(None, [item['reason'], t_text]))
        items.append(item)

    # A2-S2 command_loss: 실제 기준 D-06 진입 ≤ cmd_vel_timeout + T + 여유
    metric = f'{label}: 실제 정지 truth_stop_s 최대(command_loss)'
    mode = mode_scope('A2-S2', 'command_loss', stop_mode)
    recorded_timeout = (record.get('conditions') or {}).get('cmd_vel_timeout_s')

    if scope is not None:
        items.append(make_item('A2-S2', metric, None, None, scope, scope_reason))
    elif mode is not None:
        items.append(make_item('A2-S2', metric, None, None, *mode))
    elif not finite(recorded_timeout):
        items.append(make_item('A2-S2', metric, None, None, NOT_JUDGED, '기록에 cmd_vel_timeout_s 없음'))
    elif abs(recorded_timeout - timeout) > COMMAND_TOLERANCE:
        items.append(make_item('A2-S2', metric, None, None, NOT_JUDGED,
                               f'조건 불일치: 기록 cmd_vel_timeout {recorded_timeout} ≠ 기준 {timeout}'))
    else:
        item = upper_item('A2-S2', metric, truth_stop, timeout + t_value + stop['command_loss_margin_s'])
        item['reason'] = '; '.join(filter(None, [item['reason'], t_text, f'cmd_vel_timeout {timeout} s']))
        items.append(item)

    # A2-S4 |odom 정지 − 실제 정지| 최대
    metric = f'{label}: |stop_time_difference_s| 최대'

    if scope is not None:
        items.append(make_item('A2-S4', metric, None, None, scope, scope_reason))
    else:
        items.append(upper_item('A2-S4', metric, worst(run_values(runs, 'stop_time_difference_s', abs)),
                                stop['max_odom_truth_stop_time_difference_s']))

    # A2-S5 odom 정지 뒤 차체 실제 추가 이동(odom 정지 시점 위치 → 기록 끝 위치의 수평 거리).
    # record_only면 시나리오 종류와 관계없이 값을 남긴다.
    metric = f'{label}: slide_after_odom_stop_m 최대'
    slide = worst(run_values(runs, 'slide_after_odom_stop_m'))
    limit = stop['max_post_stop_body_motion_m']

    if 'A2-S5' in (criteria.get('record_only') or []):
        items.append(make_item('A2-S5', metric, slide, limit, RECORDED, ''))
    elif scope is not None:
        items.append(make_item('A2-S5', metric, None, None, scope, scope_reason))
    else:
        items.append(upper_item('A2-S5', metric, slide, limit))

    return items


# 정지 피드백 결과(check_stop_feedback.py result.json 사전)를 판정한다. 입력은 바꾸지 않는다.
# 반환: criteria·criteria_items·judgment·judgment_reason(·judged_subset)·rtf 기록·sample_gap 사전
def judge_stop(record, criteria, source):
    stop = criteria['stop']
    load = criteria['load']
    runs_by_scenario = record.get('runs') or {}
    stop_mode = record_stop_mode(record)
    commands = recorded_commands(record)
    mismatch = deceleration_mismatch(record, criteria['theoretical_deceleration'])
    items = []

    # 시나리오별 A2-S1·S2·S4·S5
    for name, runs in runs_by_scenario.items():
        items.extend(stop_scenario_items(name, runs, commands, stop_mode, criteria, record, mismatch))

    all_runs = [run for runs in runs_by_scenario.values() for run in runs]
    total = len(all_runs)

    # A2-S3 D-06 유지 확인(정지 회차의 truth_hold_confirmed_s 존재) — 정지 미확인 회차는 유지 미확인으로 센다
    confirmed = sum(1 for run in all_runs if run is not None and finite(run.get('truth_hold_confirmed_s')))
    metric = f'실제 기준 D-06 {stop["hold_s"]} s 유지 확인 회차 수 / 전체 회차'

    if total == 0:
        items.append(make_item('A2-S3', metric, None, total, NOT_JUDGED, '회차 없음'))
    else:
        items.append(make_item(
            'A2-S3', metric, confirmed, total,
            PASS if confirmed == total else FAIL,
            '' if confirmed == total else f'유지 미확인 {total - confirmed}회',
        ))

    items.append(make_item(
        'A2-S3', '정지 뒤 재가속 횟수', None, stop['max_reacceleration_count'], NOT_JUDGED, NO_METRIC_STAGE2,
    ))

    # A2-S6 팔 취소 정지
    has_arm = (record.get('criterion') or {}).get('arm_applies')
    has_arm = (record.get('conditions') or {}).get('has_arm', has_arm)
    metric = '팔 취소부터 Gazebo 기준 정지까지 s'

    if has_arm is False:
        items.append(make_item('A2-S6', metric, None, stop['max_arm_cancel_stop_s'], NOT_APPLICABLE, '팔 없음'))
    else:
        items.append(make_item(
            'A2-S6', metric, None, stop['max_arm_cancel_stop_s'], NOT_JUDGED,
            f'{NO_METRIC_STAGE2}: 팔 취소 시험 없음(팔 이동·복귀는 Action 완료 기준)',
        ))

    # A2-S7 실패 회차(회차 단독). 시나리오·정지 방식별 반복 수는 묶음 판정에서 센다.
    repeats = record.get('repeats')
    failed = sum(1 for run in all_runs if run is None)
    short = {name: len(runs) for name, runs in runs_by_scenario.items() if finite(repeats) and len(runs) < repeats}
    value = {'repeats': repeats, 'failed_runs': failed, 'scenarios': len(runs_by_scenario)}
    limit = {'max_failed_runs': stop['max_failed_runs']}
    metric = '정지 미확인(실패) 회차 수·반복 미완료 시나리오(반복 수는 묶음 판정)'

    if total == 0:
        items.append(make_item('A2-S7', metric, value, limit, NOT_JUDGED, '회차 없음'))
    elif failed > stop['max_failed_runs']:
        items.append(make_item('A2-S7', metric, value, limit, FAIL, f'실패 회차 {failed}개'))
    elif short:
        items.append(make_item('A2-S7', metric, value, limit, FAIL, f'반복 미완료 시나리오 {short}'))
    else:
        items.append(make_item('A2-S7', metric, value, limit, PASS))

    # 부하 항목. LD-1은 L0 회차와 비교해야 하므로 회차 단독으로는 대상이 아니다.
    items.append(make_item(
        'LD-1', 'L0 대비 같은 시나리오 정지 시간 차이', None,
        load['max_stop_time_difference_from_l0_s'], NOT_APPLICABLE, BATCH_ONLY,
    ))

    feedback = record.get('feedback') or {}
    gaps = [feedback.get('joint_states_max_gap_s'), feedback.get('odom_max_gap_s')]
    gap = max(gaps) if all(finite(value) for value in gaps) else None
    items.append(upper_item(
        'A2-L1', 'joint_states·odom 수신 최대 간격(feedback: 시험 전 대기 구간, sim s)',
        gap, load['max_control_input_gap_s'], '지표 없음(feedback 간격 기록 없음)',
    ))
    items.append(make_item(
        'A2-L1', '예상 밖 cmd_vel_timeout 횟수', None,
        load['max_unexpected_cmd_vel_timeout_count'], NOT_JUDGED, NO_METRIC_STAGE2,
    ))
    apply_record_only(items, criteria.get('record_only') or [])

    # 기록 표본 최대 간격(회차 유효 조건). 정지 확인 회차의 max_sample_gap_s 최댓값이다.
    sample_gap = worst(run_values(all_runs, 'max_sample_gap_s')) if any(all_runs) else None
    gap_limit = load['max_sample_interval_s']
    result = {
        'criteria': source,
        'criteria_items': items,
        'sample_gap': {'max_sample_gap_s': sample_gap, 'limit_s': gap_limit,
                       'source': 'runs[*].max_sample_gap_s(정지 확인 회차, 기준 시각 뒤)'},
        'rtf': rtf_record(record, load),
    }
    result.update(overall(items, record.get('status'), sample_gap, gap_limit))

    return result


# RTF 기록. rtf_min이 null이면 판정·무효 처리 없이 기록만 한다
def rtf_record(record, load):
    return {
        'rtf_min': load.get('rtf_min'),
        'value': record.get('rtf'),
        'use': '기록만(판정·무효 처리 안 함)' if load.get('rtf_min') is None else '하한 판정 미구현',
    }


# 바퀴 결과의 A2-O3 조건(속도·거리·방향별 반복 수)을 기준과 비교한다. 다르면 사유 목록
# 반복 수는 기준 이상이면 맞는 것으로 본다(A2-S7과 같은 방식).
def wheel_condition_problems(record, conditions):
    problems = []
    drives = record.get('drives') or []
    turns = record.get('turns') or []
    speed = conditions['straight_speed_m_s']
    distance = conditions['straight_distance_m']

    for item in drives:
        command = item.get('command') or {}
        linear, seconds = command.get('linear_m_s'), command.get('seconds')

        if not finite(linear) or not finite(seconds):
            problems.append(f'{item.get("label")}: 명령 기록 없음')
        elif abs(abs(linear) - speed) > COMMAND_TOLERANCE:
            problems.append(f'{item.get("label")}: 속도 {abs(linear)} ≠ {speed}')
        elif abs(abs(linear) * seconds - distance) > 1e-6:
            problems.append(f'{item.get("label")}: 명령 거리 {abs(linear) * seconds:.4f} ≠ {distance}')

    counts = {
        '전진': sum(1 for item in drives if ((item.get('command') or {}).get('linear_m_s') or 0) > 0),
        '후진': sum(1 for item in drives if ((item.get('command') or {}).get('linear_m_s') or 0) < 0),
        '회전 왼쪽': sum(1 for item in turns if ((item.get('command') or {}).get('angular_rad_s') or 0) > 0),
        '회전 오른쪽': sum(1 for item in turns if ((item.get('command') or {}).get('angular_rad_s') or 0) < 0),
    }
    required = {
        '전진': conditions['straight_repeats_each_direction'],
        '후진': conditions['straight_repeats_each_direction'],
        '회전 왼쪽': conditions['rotation_repeats_each_direction'],
        '회전 오른쪽': conditions['rotation_repeats_each_direction'],
    }

    for key, count in counts.items():
        if count < required[key]:
            problems.append(f'{key} 반복 {count} < {required[key]}')

    return problems, counts


# 바퀴 보정 결과(check_wheel_calibration.py result.json 사전)를 판정한다. 입력은 바꾸지 않는다.
def judge_wheel(record, criteria, source):
    odom = criteria['odom']
    load = criteria['load']
    conditions = odom['conditions']
    problems, counts = wheel_condition_problems(record, conditions)
    items = []

    # A2-O3 조건
    metric = '조건: 직진 속도·명령 거리·방향별 반복 수(이상)'
    value = {'repeats': counts}
    limit = dict(conditions)

    if problems:
        items.append(make_item('A2-O3', metric, value, limit, NOT_JUDGED, '조건 불일치: ' + '; '.join(problems)))
    else:
        items.append(make_item('A2-O3', metric, value, limit, PASS))

    # A2-O1 직진 비율, A2-O2 회전 비율. 조건이 다르면 판정하지 않는다
    ratio_sets = (
        ('A2-O1', '직진 거리 비율(odom/실제) 회차별', record.get('drives') or [],
         odom['straight_ratio_min'], odom['straight_ratio_max']),
        ('A2-O2', '회전 비율(odom/실제) 회차별', record.get('turns') or [],
         odom['rotation_ratio_min'], odom['rotation_ratio_max']),
    )

    for item_id, metric, trials, minimum, maximum in ratio_sets:
        if problems:
            items.append(make_item(item_id, metric, None, [minimum, maximum], NOT_JUDGED, '조건 불일치(A2-O3)'))
            continue

        values = [trial.get('ratio_odom_over_truth') for trial in trials]
        items.append(range_item(item_id, metric, values, minimum, maximum))

    apply_record_only(items, criteria.get('record_only') or [])

    # 기록 표본 최대 간격: 회차 끝 정지 확인 구간(end_speed.max_gap_s)만 기록돼 있다
    gaps = [
        (trial.get('end_speed') or {}).get('max_gap_s')
        for trial in (record.get('turns') or []) + (record.get('drives') or [])
    ]
    sample_gap = worst([value for value in gaps if value is not None])
    gap_limit = load['max_sample_interval_s']
    result = {
        'criteria': source,
        'criteria_items': items,
        'sample_gap': {'max_sample_gap_s': sample_gap, 'limit_s': gap_limit,
                       'source': 'turns·drives[*].end_speed.max_gap_s(회차 끝 정지 확인 구간만)'},
        'rtf': rtf_record(record, load),
    }
    result.update(overall(items, record.get('status'), sample_gap, gap_limit))

    return result


JUDGES = {'stop': judge_stop, 'wheel': judge_wheel}

# 활성 로봇 판별: joint_state_broadcaster가 내는 /ax/<id>/joint_states에 발행자가 있으면 활성으로 본다.
ROBOT_JOINT_STATES = re.compile(r'^/ax/([^/]+)/joint_states$')
ACTIVE_ROBOT_SOURCE = (
    'ROS 그래프: /ax/<id>/joint_states 발행자 수 > 0'
    '(측정 시작 시 get_topic_names_and_types·count_publishers, 같은 ROS domain)'
)
RENDERER_KEYS = ('LIBGL_ALWAYS_SOFTWARE', 'GALLIUM_DRIVER', 'MESA_D3D12_DEFAULT_ADAPTER_NAME')
WORLD_SUFFIXES = ('.world', '.sdf')


# 토픽별 발행자 수에서 활성 로봇 ID 목록(정렬)을 만든다. 발행자가 없는 토픽(구독만 있는 것)은 뺀다
def active_robots(publisher_counts):
    robots = set()

    for topic, count in publisher_counts.items():
        match = ROBOT_JOINT_STATES.match(topic)

        if match and count > 0:
            robots.add(match.group(1))

    return sorted(robots)


# 노드의 ROS 그래프에서 joint_states 토픽별 발행자 수를 읽는다(rclpy Node와 같은 메서드를 가진 객체)
def graph_publisher_counts(node):
    return {
        name: node.count_publishers(name)
        for name, _ in node.get_topic_names_and_types()
        if ROBOT_JOINT_STATES.match(name)
    }


# /proc에서 같은 GZ_PARTITION의 gz sim 프로세스 명령줄에 있는 월드 파일 경로를 모은다.
# gz(ruby)는 프로세스 제목을 공백으로 이은 한 문자열로 바꿔 쓰므로 NUL과 공백 모두로 나눈다(공백 든 경로는 못 찾음).
# 상대 경로는 그 프로세스의 작업 폴더 기준이다. 읽을 수 없는 프로세스는 건너뛴다. 결과는 존재하는 파일의 절대 경로 목록이다
def server_world_files(partition, proc_root='/proc'):
    worlds = []

    for folder in sorted(Path(proc_root).glob('[0-9]*')):
        try:
            argv = (folder / 'cmdline').read_bytes().split(b'\0')
            environ = (folder / 'environ').read_bytes().split(b'\0')
        except OSError:
            continue

        words = [part for word in argv for part in word.decode('utf-8', 'replace').split()]
        is_gz_sim = any(
            Path(word).name == 'gz' and index + 1 < len(words) and words[index + 1] == 'sim'
            for index, word in enumerate(words)
        )

        if not is_gz_sim:
            continue

        values = dict(
            item.decode('utf-8', 'replace').split('=', 1) for item in environ if b'=' in item
        )

        if values.get('GZ_PARTITION') != partition:
            continue

        try:
            cwd = (folder / 'cwd').resolve()
        except OSError:
            cwd = None

        for word in words:
            path = Path(word)

            if path.suffix not in WORLD_SUFFIXES:
                continue

            if not path.is_absolute():
                if cwd is None:
                    continue

                path = cwd / path

            if path.is_file():
                worlds.append(str(path.resolve()))

    return list(dict.fromkeys(worlds))


# --world-file 인자로 받은 월드의 경로·sha256과, gz 서버가 연 월드와 같은지 확인한 결과를 만든다
def world_conditions(world_file, partition, proc_root='/proc'):
    if world_file is None:
        return {
            'world_file': None,
            'world_sha256': None,
            'world_reason': '--world-file 없음',
            'world_check': None,
        }

    path = Path(world_file).resolve()
    digest = file_sha256(path)
    servers = server_world_files(partition, proc_root)

    if servers:
        matched = [server for server in servers if file_sha256(server) == digest]
        check = {
            'method': 'gz sim 서버 명령줄(/proc, 같은 GZ_PARTITION)의 월드 파일 sha256 비교',
            'partition': partition,
            'server_worlds': servers,
            'match': bool(matched),
        }
    else:
        check = {
            'method': '인자 기준',
            'partition': partition,
            'reason': '같은 GZ_PARTITION의 gz sim 서버 명령줄에서 월드 파일을 찾지 못함',
            'match': None,
        }

    return {'world_file': str(path), 'world_sha256': digest, 'world_reason': None, 'world_check': check}


# 렌더러 관련 환경값(측정 도구 프로세스 기준)
def renderer_conditions(environ):
    values = {key: environ.get(key) for key in RENDERER_KEYS}
    values['source'] = '측정 도구 프로세스 환경값(시뮬레이터 프로세스 값과 다를 수 있음)'

    return values


# 측정 결과 conditions에 더할 부하 단계 정보(활성 로봇·월드·렌더러)를 만든다.
# node는 get_topic_names_and_types·count_publishers를 가진 객체(rclpy Node)다.
def measurement_load_conditions(node, world_file, environ=None, proc_root='/proc'):
    environ = os.environ if environ is None else environ
    robots = active_robots(graph_publisher_counts(node))
    conditions = {
        'active_robot_ids': robots,
        'active_robot_count': len(robots),
        'active_robot_source': ACTIVE_ROBOT_SOURCE,
    }
    conditions.update(world_conditions(world_file, environ.get('GZ_PARTITION'), proc_root))
    conditions['renderer'] = renderer_conditions(environ)

    return conditions


# 결과 형식으로 종류를 정한다. 정지 결과는 runs, 바퀴 결과는 turns·drives가 있다. 알 수 없으면 ValueError
def detect_kind(record):
    if 'runs' in record:
        return 'stop'

    if 'turns' in record or 'drives' in record:
        return 'wheel'

    raise ValueError('결과 종류를 알 수 없음(runs / turns·drives 없음)')


# 회차 결과에서 묶음 판정에 쓰는 조건을 요약한다. 기록에 없는 값은 None이다
def run_conditions(kind, record):
    conditions = record.get('conditions') or {}
    summary = {
        'kind': kind,
        'robot_id': record.get('robot_id', conditions.get('robot_id')),
        'status': record.get('status'),
    }

    summary.update({
        'active_robot_count': conditions.get('active_robot_count'),
        'active_robot_ids': conditions.get('active_robot_ids'),
        'world_sha256': conditions.get('world_sha256'),
    })

    if kind == 'stop':
        summary.update({
            'stop_mode': record_stop_mode(record),
            'payload_kg': conditions.get('payload_kg'),
            'repeats': record.get('repeats'),
        })

    return summary


# 회차 판정이 묶음 근거로 유효한지 본다(status incomplete·SAMPLE_GAP 회차는 근거로 쓰지 않음)
def run_valid(judged):
    reason = judged.get('judgment_reason') or ''

    return not reason.startswith(('status incomplete', 'SAMPLE_GAP'))


# A2-S7 반복 수를 로봇·정지 방식·하중·동시 로봇 수·시나리오 묶음별로 센다(유효 회차만)
def repeat_coverage(entries, criteria):
    required = criteria['stop']['repeats']
    stop_entries = [entry for entry in entries if entry['kind'] == 'stop' and entry['valid']]
    metric = '로봇·정지 방식·하중·동시 로봇 수·시나리오별 회차 수'

    if not stop_entries:
        return make_item('A2-S7', metric, None, {'repeats_min': required}, NOT_APPLICABLE, '정지 결과 없음')

    groups = {}
    missing = set()

    for entry in stop_entries:
        conditions = entry['conditions']
        missing.update(field for field in REPEAT_GROUP_FIELDS if conditions.get(field) is None)
        key = tuple(conditions.get(field) for field in REPEAT_GROUP_FIELDS)

        for scenario, runs in (entry['record'].get('runs') or {}).items():
            groups[key + (scenario,)] = groups.get(key + (scenario,), 0) + len(runs)

    value = [
        dict(zip(REPEAT_GROUP_FIELDS + ('scenario',), key), runs=count)
        for key, count in groups.items()
    ]
    limit = {'repeats_min': required}

    if missing:
        return make_item('A2-S7', metric, value, limit, NOT_JUDGED,
                         f'결과에서 읽을 수 없는 조건: {", ".join(sorted(missing))}')

    short = [item for item in value if item['runs'] < required]

    if short:
        return make_item('A2-S7', metric, value, limit, NOT_JUDGED, f'반복 수 부족 묶음 {len(short)}개')

    return make_item('A2-S7', metric, value, limit, PASS)


# LD-1 부하 불변성: 같은 월드·로봇·정지 방식·하중·시나리오에서 L0(동시 로봇 1대) 대비 정지 시간 평균 차이
def load_invariance(entries, criteria):
    limit = criteria['load']['max_stop_time_difference_from_l0_s']
    stop_entries = [entry for entry in entries if entry['kind'] == 'stop' and entry['valid']]
    metric = 'L0(동시 로봇 1대) 대비 같은 조건·시나리오 truth_stop_s 평균 차이 최대'

    if not stop_entries:
        return make_item('LD-1', metric, None, limit, NOT_APPLICABLE, '정지 결과 없음')

    if any(entry['conditions'].get(field) is None
           for entry in stop_entries for field in ('active_robot_count', 'world_sha256')):
        return make_item('LD-1', metric, None, limit, NOT_JUDGED, NO_LOAD_STAGE)

    # (월드, 로봇, 정지 방식, 하중, 시나리오) → 부하 단계(동시 로봇 수)별 정지 시간 목록
    table = {}

    for entry in stop_entries:
        conditions = entry['conditions']
        base = tuple(conditions[field] for field in ('world_sha256', 'robot_id', 'stop_mode', 'payload_kg'))

        for scenario, runs in (entry['record'].get('runs') or {}).items():
            values = [value for value in run_values(runs, 'truth_stop_s') if finite(value)]
            stages = table.setdefault(base + (scenario,), {})
            stages.setdefault(conditions['active_robot_count'], []).extend(values)

    differences = []

    for stages in table.values():
        baseline = stages.get(1)

        if not baseline:
            continue

        for count, values in stages.items():
            if count != 1 and values:
                differences.append(abs(sum(values) / len(values) - sum(baseline) / len(baseline)))

    if not differences:
        return make_item('LD-1', metric, None, limit, NOT_JUDGED, 'L0와 비교할 같은 조건의 부하 회차 없음')

    return upper_item('LD-1', metric, max(differences), limit)


# 항목 ID별 충족 여부를 정한다. 같은 ID의 세부 지표(metric)마다 유효 회차의 PASS·FAIL 근거를 찾는다.
#   met: 근거가 있고, not_applicable로만 나온 세부 지표를 뺀 모든 세부 지표에 근거가 있다.
#   recorded: 모든 회차에서 recorded뿐이다(판정 대상 아님).
#   unmet: 그 밖. 모든 회차에서 not_applicable뿐인 ID도 판정된 적이 없으므로 unmet이다.
# 묶음 항목(A2-S7 반복 수, LD-1)은 같은 ID의 세부 지표 "<metric>(묶음)"으로 합쳐 ID 하나로 나온다.
# 묶음 항목의 PASS·FAIL 근거는 evidence에 'batch'로 남는다. 회차 단독 LD-1(묶음 판정 항목)은 세지 않는다.
def item_coverage(entries, batch_items=()):
    table = {}

    for entry in entries:
        for item in entry['judged']['criteria_items']:
            # 회차 단독으로는 대상이 아닌 묶음 판정 항목(LD-1)은 묶음 항목으로만 센다.
            if item['result'] == NOT_APPLICABLE and item['reason'] == BATCH_ONLY:
                continue

            metrics = table.setdefault(item['id'], {})
            seen = metrics.setdefault(item['metric'], {'results': set(), 'evidence': []})
            seen['results'].add(item['result'])

            if entry['valid'] and item['result'] in JUDGED and entry['path'] not in seen['evidence']:
                seen['evidence'].append(entry['path'])

    for item in batch_items:
        if item['result'] == NOT_APPLICABLE:
            continue

        metrics = table.setdefault(item['id'], {})
        seen = metrics.setdefault(f'{item["metric"]}(묶음)', {'results': set(), 'evidence': []})
        seen['results'].add(item['result'])

        if item['result'] in JUDGED:
            seen['evidence'].append('batch')

    coverage = []

    for item_id, metrics in table.items():
        results = set().union(*(seen['results'] for seen in metrics.values()))
        evidence = list(dict.fromkeys(path for seen in metrics.values() for path in seen['evidence']))
        missing = [
            metric for metric, seen in metrics.items()
            if not seen['evidence'] and not seen['results'] <= {NOT_APPLICABLE, RECORDED}
        ]

        if results <= {RECORDED}:
            status = RECORDED
        elif evidence and not missing:
            status = 'met'
        else:
            status = 'unmet'

        coverage.append({
            'id': item_id,
            'status': status,
            'evidence': evidence,
            'missing_metrics': missing,
            'results_seen': sorted(results),
        })

    return coverage


# 묶음 출력의 회차 요약(경로·해시·종류·유효 여부·조건·회차 판정)
def run_summary(entry):
    summary = {key: entry[key] for key in ('path', 'sha256', 'kind', 'valid', 'conditions')}
    summary['judgment'] = entry['judged']['judgment']
    summary['judgment_reason'] = entry['judged']['judgment_reason']

    return summary


# 여러 회차 결과를 묶어 판정한다. 입력은 (경로, 원본 바이트, 종류 또는 None) 목록이며 원본은 바꾸지 않는다.
def judge_batch(inputs, criteria, source):
    entries = []

    for path, original, kind in inputs:
        record = json.loads(original.decode('utf-8'))
        kind = kind or detect_kind(record)
        judged = JUDGES[kind](copy.deepcopy(record), criteria, source)
        entries.append({
            'path': str(path),
            'sha256': hashlib.sha256(original).hexdigest(),
            'kind': kind,
            'record': record,
            'judged': judged,
            'valid': run_valid(judged),
            'conditions': run_conditions(kind, record),
        })

    # 묶음 항목(반복 수·부하 불변성)을 먼저 판정하고, 항목 ID별 충족에 합친다.
    batch_items = [repeat_coverage(entries, criteria), load_invariance(entries, criteria)]
    coverage = item_coverage(entries, batch_items)

    # 묶음 전체 판정
    failures = [
        {'path': entry['path'],
         'items': sorted({item['id'] for item in entry['judged']['criteria_items'] if item['result'] == FAIL})}
        for entry in entries if entry['judged']['judgment'] == FAIL
    ]
    failures += [{'path': 'batch', 'items': [item['id']]} for item in batch_items if item['result'] == FAIL]
    unmet = [item['id'] for item in coverage if item['status'] == 'unmet']
    result = {
        'criteria': source,
        'runs': [run_summary(entry) for entry in entries],
        'coverage': coverage,
        'batch_items': batch_items,
        'failures': failures,
        'unmet': unmet,
    }

    if failures:
        result['judgment'] = FAIL
        result['judgment_reason'] = 'FAIL: ' + '; '.join(f'{item["path"]} {item["items"]}' for item in failures)
    elif not unmet:
        result['judgment'] = PASS
        result['judgment_reason'] = '미충족 항목 없음'
    else:
        result['judgment'] = NOT_JUDGED
        result['judged_subset'] = PASS
        result['judgment_reason'] = f'미충족 항목: {", ".join(unmet)}'

    return result


# 판정 결과를 한 줄씩 출력한다
def print_judgment(judged, stream=None):
    stream = stream or sys.stdout

    for item in judged['criteria_items']:
        value = item['value']
        text = f'{value:.6g}' if finite(value) else json.dumps(value, ensure_ascii=False)
        reason = f' — {item["reason"]}' if item['reason'] else ''
        print(f'[{item["result"]}] {item["id"]} {item["metric"]}: {text} (기준 {item["limit"]}){reason}',
              file=stream, flush=True)

    subset = f', judged_subset {judged["judged_subset"]}' if 'judged_subset' in judged else ''
    print(f'[판정] {judged["judgment"]}{subset}: {judged["judgment_reason"]}', file=stream, flush=True)


# JSON을 임시 파일에 쓴 뒤 바꿔 넣는다
def write_json(target, data):
    temporary = target.with_name(target.name + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, default=float) + '\n', encoding='utf-8')
    os.replace(temporary, target)


# 과거 결과를 재판정해 <원본 이름>.judged.json으로 쓴다. 원본은 읽기만 한다. 쓴 경로와 판정 사전을 돌려준다.
def rejudge(kind, result_path, criteria_path):
    result_path = Path(result_path).resolve()
    original_bytes = result_path.read_bytes()
    record = json.loads(original_bytes.decode('utf-8'))
    criteria, source = load_criteria(criteria_path)
    kind = kind or detect_kind(record)
    judged = JUDGES[kind](copy.deepcopy(record), criteria, source)

    output = copy.deepcopy(record)
    output.update(judged)
    output['rejudged_from'] = {
        'path': str(result_path),
        'sha256': hashlib.sha256(original_bytes).hexdigest(),
        'at': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(timespec='seconds'),
        'tool': f'a2_criteria.py {kind}',
    }

    target = result_path.with_name(result_path.stem + '.judged.json')

    if target == result_path:
        raise ValueError('출력 경로가 원본과 같음')

    write_json(target, output)

    return target, judged


# 묶음 판정 결과를 output에 쓴다. output이 이미 있거나 입력과 같으면 쓰지 않는다.
def batch(kind, result_paths, criteria_path, output):
    output = Path(output).resolve()
    paths = [Path(path).resolve() for path in result_paths]

    if output.exists() or output in paths:
        raise ValueError(f'출력 파일이 이미 있거나 입력과 같음: {output}')

    criteria, source = load_criteria(criteria_path)
    inputs = [(path, path.read_bytes(), kind) for path in paths]
    result = judge_batch(inputs, criteria, source)
    result['at'] = datetime.now(ZoneInfo('Asia/Seoul')).isoformat(timespec='seconds')
    output.parent.mkdir(parents=True, exist_ok=True)
    write_json(output, result)

    return output, result


# --rejudge 명령을 실행하고 종료 코드를 돌려준다(판정 결과와 관계없이 쓰기에 성공하면 0)
def run_rejudge(kind, result_path, criteria_path):
    try:
        target, judged = rejudge(kind, result_path, criteria_path)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f'[ERROR] 재판정 실패: {type(error).__name__}: {error}', file=sys.stderr, flush=True)

        return 2

    print_judgment(judged)
    print(f'재판정 결과: {target}', flush=True)

    return 0


# --batch 명령을 실행하고 종료 코드를 돌려준다(판정 결과와 관계없이 쓰기에 성공하면 0)
def run_batch(kind, result_paths, criteria_path, output):
    try:
        target, result = batch(kind, result_paths, criteria_path, output)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f'[ERROR] 묶음 판정 실패: {type(error).__name__}: {error}', file=sys.stderr, flush=True)

        return 2

    for run in result['runs']:
        print(f'[회차] {run["judgment"]} {run["path"]}', flush=True)

    for item in result['coverage']:
        print(f'[항목] {item["id"]} {item["status"]} (근거 {len(item["evidence"])}회차)', flush=True)

    for item in result['batch_items']:
        print(f'[묶음] {item["id"]} {item["result"]} {item["reason"]}', flush=True)

    subset = f', judged_subset {result["judged_subset"]}' if 'judged_subset' in result else ''
    print(f'[판정] {result["judgment"]}{subset}: {result["judgment_reason"]}', flush=True)
    print(f'묶음 판정 결과: {target}', flush=True)

    return 0


# 명령행 인자 해석기를 만든다
def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        'kind', nargs='?', choices=sorted(JUDGES), default=None,
        help='stop: check_stop_feedback 결과, wheel: check_wheel_calibration 결과(생략하면 형식으로 정함)',
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--rejudge', type=Path, help='재판정할 result.json')
    mode.add_argument('--batch', type=Path, nargs='+', help='묶어 판정할 result.json 목록')
    parser.add_argument('--criteria', type=Path, required=True, help='기준 파일(stop_odom_criteria_dev.yaml)')
    parser.add_argument('--output', type=Path, default=None, help='--batch 출력 파일(이미 있으면 쓰지 않음)')

    return parser


# 명령행 진입점
def main(argv=None):
    parser = build_parser()
    arguments = parser.parse_args(argv)

    if arguments.batch is not None:
        if arguments.output is None:
            parser.error('--batch에는 --output이 필요')

        return run_batch(arguments.kind, arguments.batch, arguments.criteria, arguments.output)

    return run_rejudge(arguments.kind, arguments.rejudge, arguments.criteria)


if __name__ == '__main__':
    sys.exit(main())
