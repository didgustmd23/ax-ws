# B2-L4~L7(과신·도약 감지·재위치 정확도·재위치 거절) 판정 지표를 합성 원시 기록으로 ROS 없이 검사한다.
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'src/ax_localization/scripts'
CRITERIA_PATH = ROOT / 'src/ax_localization/config/localization_criteria_dev.yaml'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(ROOT / 'tools/tests'))

spec = importlib.util.spec_from_file_location('check_localization', SCRIPTS / 'check_localization.py')
CHECK = importlib.util.module_from_spec(spec)
spec.loader.exec_module(CHECK)
import b2_event_fixture as F  # noqa: E402


# 기준 파일을 읽는다(시험마다 새로 읽는다).
def criteria():
    return yaml.safe_load(CRITERIA_PATH.read_text(encoding='utf-8'))


# (id, metric) 항목 하나를 찾는다.
def find(items, item_id, metric):
    return next(item for item in items if item['id'] == item_id and item['metric'] == metric)


# 결과만 (metric → result) dict로 꺼낸다.
def results(items, item_id):
    return {item['metric'].split('.')[1]: item['result'] for item in items if item['id'] == item_id}


# truth를 시각 구간 [start, end]에서 x 방향으로 offset만큼 옮긴다(평가 오차 = offset).
def shift_truth(start, end, offset):
    def change(data):
        for row in data['truth']:
            if start <= row['stamp'] <= end:
                row['x'] += offset

    return change


# health 상태를 시각 함수로 다시 만든다(감독 tick도 같은 상태로 맞춘다).
def set_health(state_of):
    def change(data):
        data['health'] = [F.health_row(t, *state_of(t)) for t in F.times(0.0, 40.0, 0.2)]

    return change


# 감독 진단 줄 하나를 바꾸거나 지운다(kind·시각 at으로 찾는다).
def edit_diag(kind, at, **updates):
    def change(data):
        for row in list(data['supervisor']):
            if row['kind'] == kind and abs(row['sim'] - at) < 1e-9:
                if updates.get('remove'):
                    data['supervisor'].remove(row)
                else:
                    row.update(updates)

    return change


# 여러 변경을 차례로 적용한다.
def chain(*changes):
    def change(data):
        for item in changes:
            item(data)

    return change


L4 = 'overconfidence.max_ok_samples_above_threshold'
DELAY = 'discontinuity.max_ok_exit_delay_s'
EARLY = 'discontinuity.max_ok_before_relocalization'
COMPLETION = 'relocalization.max_completion_s'
ACCURACY = 'relocalization.max_position_error_after_ok_m'
MOVING = 'relocalization.moving_request_result'
OK_REQUEST = 'relocalization.ok_request_result'
WRONG = 'relocalization.max_wrong_accept_count'


# B2-L4 과신
class Overconfidence(unittest.TestCase):

    # 오차 0.15 m(= 기준)는 초과가 아니고, 0.15 m를 조금이라도 넘으면 OK 표본이 과신으로 세져 FAIL이다.
    def test_boundary(self):
        exact = find(F.items(CHECK, criteria(), shift_truth(1.0, 2.0, 0.15)), 'B2-L4', L4)
        over = find(F.items(CHECK, criteria(), shift_truth(1.0, 2.0, 0.15 + 1e-6)), 'B2-L4', L4)

        self.assertEqual((exact['result'], exact['value'], exact['overconfidence']['exceeded']), ('PASS', 0, 0))
        self.assertEqual(over['result'], 'FAIL')
        self.assertEqual(over['value'], over['overconfidence']['exceeded'])
        self.assertGreater(over['value'], 0)

    # 반올림하면 0.1500이 되는 0.15004 m도 원시 정밀도에서 초과로 센다(보고 숫자로 경계를 재판정하지 않는다).
    def test_rounding(self):
        item = find(F.items(CHECK, criteria(), shift_truth(1.0, 1.0, 0.15004)), 'B2-L4', L4)
        sample = item['overconfidence']['exceeded_samples'][0]

        self.assertEqual((item['result'], item['value']), ('FAIL', 1))
        self.assertEqual(round(sample['position_error_m'], 4), 0.15)
        self.assertGreater(sample['position_error_m'], 0.15)
        self.assertEqual(sample['health']['state'], 'OK')

    # 초과 표본이 0개면 PASS이며 위험 조건 미노출을 사유에 쓴다. 초과 표본의 health가 DEGRADED면 과신 아님(PASS).
    def test_zero_and_safe(self):
        zero = find(F.items(CHECK, criteria()), 'B2-L4', L4)
        safe = find(F.items(CHECK, criteria(), chain(shift_truth(6.8, 7.3, 0.2))), 'B2-L4', L4)

        self.assertEqual(zero['result'], 'PASS')
        self.assertIn('위험 조건 미노출', zero['reason'])
        self.assertEqual((safe['result'], safe['value']), ('PASS', 0))
        self.assertGreater(safe['overconfidence']['exceeded'], 0)
        self.assertIn('위험 조건 노출', safe['reason'])

    # state OK·valid=false는 과신으로 세지 않고 따로 남긴다.
    def test_invalid_ok(self):
        change = chain(shift_truth(1.0, 2.0, 0.2), set_health(lambda t: ('OK', not 0.9 <= t <= 2.1)))
        item = find(F.items(CHECK, criteria(), change), 'B2-L4', L4)

        self.assertEqual((item['result'], item['value']), ('PASS', 0))
        self.assertEqual(item['overconfidence']['invalid_ok'], item['overconfidence']['exceeded'])

    # 초과 표본 시각에 health가 없거나(만료 1.0 s) 기록 자체가 없으면 not_judged다.
    def test_missing_health(self):
        def drop(data):
            data['health'] = [row for row in data['health'] if not 0.5 <= row['stamp'] <= 2.5]

        gap = find(F.items(CHECK, criteria(), chain(shift_truth(1.6, 2.0, 0.2), drop)), 'B2-L4', L4)
        none = find(F.items(CHECK, criteria(), lambda data: data.update(health=[])), 'B2-L4', L4)
        no_map = find(F.items(CHECK, criteria(), world_map=None), 'B2-L4', L4)

        self.assertEqual(gap['result'], 'not_judged')
        self.assertIn('확인 불가', gap['reason'])
        self.assertEqual(none['result'], 'not_judged')
        self.assertEqual(no_map['result'], 'not_judged')

    # 확인된 과신은 다른 표본의 health 결측이 있어도 FAIL로 남는다.
    def test_confirmed_fail_with_missing(self):
        def drop(data):
            data['health'] = [row for row in data['health'] if not 2.5 <= row['stamp'] <= 4.5]

        item = find(F.items(CHECK, criteria(), chain(shift_truth(1.0, 4.0, 0.2), drop)), 'B2-L4', L4)

        self.assertEqual(item['result'], 'FAIL')
        self.assertGreater(item['overconfidence']['unknown_health'], 0)

    # 같은 stamp 재발행·다른 로봇 health 줄은 표본 수를 늘리거나 판정에 섞이지 않는다.
    def test_republish_and_other_robot(self):
        def noise(data):
            data['health'] += [dict(row) for row in data['health']]
            data['health'].sort(key=lambda row: row['stamp'])
            data['health'] += [F.health_row(1.5, 'OK', robot_id='robot_1')]

        base = find(F.items(CHECK, criteria(), shift_truth(1.0, 2.0, 0.2)), 'B2-L4', L4)
        noisy = find(F.items(CHECK, criteria(), chain(shift_truth(1.0, 2.0, 0.2), noise)), 'B2-L4', L4)

        self.assertEqual(noisy['value'], base['value'])
        self.assertEqual(noisy['overconfidence']['exceeded'], base['overconfidence']['exceeded'])


# B2-L5 AMCL 재시작·도약 감지
class Discontinuity(unittest.TestCase):

    # OK 이탈까지 1.0 s(= 기준)는 PASS, 1.2 s는 FAIL. 사건 시작은 lifecycle 수신 시각이다(health 저하 시각 아님).
    def test_delay_boundary(self):
        for lost_from, expected in ((16.0, 'PASS'), (16.2, 'FAIL')):
            with self.subTest(lost_from=lost_from):
                state = set_health(lambda t: ('LOST', True) if lost_from <= t < 20.0 else ('OK', True))
                item = find(F.items(CHECK, criteria(), state), 'B2-L5', DELAY)

                self.assertEqual(item['result'], expected)
                self.assertAlmostEqual(item['value'], lost_from - 15.0)
                self.assertEqual(item['discontinuity']['incidents'][0]['start'], 15.0)

    # 재위치 성공 전 OK 복귀 1개면 FAIL, 0개면 PASS다.
    def test_early_ok(self):
        state = set_health(lambda t: ('OK', True) if t < 15.4 or t >= 20.0 or abs(t - 18.0) < 1e-9
                           else ('LOST', True))
        item = find(F.items(CHECK, criteria(), state), 'B2-L5', EARLY)

        self.assertEqual((item['result'], item['value']), ('FAIL', 1))
        self.assertEqual(find(F.items(CHECK, criteria()), 'B2-L5', EARLY)['result'], 'PASS')

    # 재위치 성공 tick에 발행한 OK(stamp ns 절삭으로 진단 sim보다 1 ns 이하 작음)는 성공 전 복귀가 아니다.
    # 1 ms 앞선 OK는 성공 전 복귀(FAIL)다.
    def test_same_tick_success(self):
        for ok_at, expected in ((20.0 - 1e-9, 'PASS'), (19.999, 'FAIL')):
            with self.subTest(ok_at=ok_at):
                def change(data, ok_at=ok_at):
                    edit_diag('reinit_finished', 19.9, sim=20.0)(data)
                    data['health'].append(F.health_row(ok_at, 'OK'))
                    data['health'].sort(key=lambda row: row['stamp'])

                self.assertEqual(find(F.items(CHECK, criteria(), change), 'B2-L5', EARLY)['result'], expected)

    # 사건이 없으면 not_applicable(도약 감지 시험 아님). 사건 출처가 끊겼으면 not_judged다.
    def test_no_event(self):
        quiet = F.items(CHECK, criteria(), lambda data: data.update(amcl_lifecycle=[]))
        broken = F.items(CHECK, criteria(), lambda data: data.update(amcl_lifecycle=[]),
                         source_problems=['감독 진단 tick 공백 1곳'])

        self.assertEqual(results(quiet, 'B2-L5'), {'max_ok_exit_delay_s': 'not_applicable',
                                                   'max_ok_before_relocalization': 'not_applicable'})
        self.assertIn('도약 감지 시험 아님', find(quiet, 'B2-L5', DELAY)['reason'])
        self.assertEqual(set(results(broken, 'B2-L5').values()), {'not_judged'})

    # 사건 시각이 없으면 not_judged(그 사건을 0 지연으로 메우지 않는다).
    def test_missing_event_time(self):
        def change(data):
            data['amcl_lifecycle'][0]['recv_sim'] = None

        items = F.items(CHECK, criteria(), change)

        self.assertEqual(set(results(items, 'B2-L5').values()), {'not_judged'})
        self.assertIn('사건 시각 누락', find(items, 'B2-L5', DELAY)['reason'])

    # 재위치 성공 전 기록이 끝나면 조기 복귀 감시가 잘려 not_judged, 지연은 따로 판정한다.
    def test_truncated(self):
        change = chain(edit_diag('reinit_finished', 19.9, remove=True),
                       set_health(lambda t: ('LOST', True) if t >= 15.4 else ('OK', True)))
        items = F.items(CHECK, criteria(), change)

        self.assertEqual(find(items, 'B2-L5', DELAY)['result'], 'PASS')
        self.assertEqual(find(items, 'B2-L5', EARLY)['result'], 'not_judged')
        self.assertIn('잘림', find(items, 'B2-L5', EARLY)['reason'])

    # OK·valid=false로의 이탈은 OK_INVALID로 따로 표시한다(안전한 DEGRADED와 섞지 않는다).
    def test_invalid_ok_exit(self):
        state = set_health(lambda t: ('OK', not 15.4 <= t < 20.0))
        incident = find(F.items(CHECK, criteria(), state), 'B2-L5', DELAY)['discontinuity']['incidents'][0]

        self.assertEqual(incident['exit_state'], 'OK_INVALID')
        self.assertGreater(incident['invalid_ok'], 0)
        self.assertEqual(incident['early_ok'], 0)

    # 다른 세션의 재위치 성공으로 사건을 닫지 않는다. 그 뒤 20.0 s OK 복귀는 같은 세션 성공 전 조기 복귀(FAIL)다.
    def test_other_session_success(self):
        items = F.items(CHECK, criteria(), edit_diag('reinit_finished', 19.9, session='s2'))
        incident = find(items, 'B2-L5', EARLY)['discontinuity']['incidents'][0]

        self.assertIsNone(incident['success'])
        self.assertEqual(incident['ignored_other_session_successes'], 1)
        self.assertEqual(incident['early_ok_first'], 20.0)
        self.assertEqual(find(items, 'B2-L5', EARLY)['result'], 'FAIL')

    # 사건 시각에 이미 OK가 아니면 지연 0의 의미를 따로 적는다.
    def test_already_not_ok(self):
        def change(data):
            data['amcl_lifecycle'][0]['recv_sim'] = 15.6

        incident = find(F.items(CHECK, criteria(), change), 'B2-L5', DELAY)['discontinuity']['incidents'][0]

        self.assertEqual(incident['delay_s'], 0.0)
        self.assertIn('감지 지연이 아님', incident['already_not_ok'])

    # 원시 TF 도약: map→odom 병진은 0이어도 같은 odom 자세에서 합성한 추정이 0.3 m를 넘으면 사건이다.
    # 감독 INTENDED와 같은 valid_time이면 의도된 갱신이라 사건이 아니다. health가 계속 OK면 지연 FAIL(하한)이다.
    def test_raw_estimate_jump(self):
        def rotate(data):
            data['amcl_lifecycle'] = []

            for row in data['tf_map_odom']:
                if row['valid_time'] >= 25.0:
                    row['yaw'] = 0.25

        jump = F.items(CHECK, criteria(), rotate)
        incident = find(jump, 'B2-L5', DELAY)['discontinuity']['incidents'][0]

        self.assertEqual(incident['source'], 'tf_estimate_jump(원시 TF·odom 합성)')
        self.assertEqual(incident['start'], 25.0)
        self.assertEqual(find(jump, 'B2-L5', DELAY)['result'], 'FAIL')
        self.assertEqual(incident['delay_level'], 'lower_bound')

        def intended(data):
            rotate(data)
            data['supervisor'].append(F.diag('input', 25.01, estimate_events=[
                {'event': 'INTENDED', 'source': 'REINIT', 'valid_time': 25.0}]))

        self.assertEqual(find(F.items(CHECK, criteria(), intended), 'B2-L5', DELAY)['result'], 'not_applicable')

    # 같은 사건이 열려 있는 동안 들어온 다른 출처(감독 LATCHED)는 같은 사건에 묶는다.
    def test_related_sources(self):
        def latch(data):
            data['supervisor'].append(F.diag('input', 15.5, 'LOST', estimate_events=[
                {'event': 'LATCHED', 'source': 'amcl_lifecycle', 'stamp': 15.5}]))

        incidents = find(F.items(CHECK, criteria(), latch), 'B2-L5', DELAY)['discontinuity']['incidents']

        self.assertEqual(len(incidents), 1)
        self.assertEqual(incidents[0]['related'][0]['start'], 15.5)


# B2-L6 재위치 정확도·완료 시간
class Relocalization(unittest.TestCase):

    # 완료 10.0 s(= 기준)는 PASS, 10.1 s는 FAIL이다.
    def test_completion_boundary(self):
        for finish, expected in ((27.0, 'PASS'), (27.1, 'FAIL')):
            with self.subTest(finish=finish):
                change = chain(edit_diag('reinit_finished', 19.9, sim=finish),
                               set_health(lambda t: ('LOST', True) if 15.4 <= t < 27.2 else ('OK', True)))
                item = find(F.items(CHECK, criteria(), change), 'B2-L6', COMPLETION)

                self.assertEqual(item['result'], expected)
                self.assertAlmostEqual(item['value'], finish - 17.0)

    # 성공 뒤 첫 유효 OK 시점 오차: 기준과 같으면 PASS(경계 포함), 넘으면 FAIL이다.
    def test_accuracy_boundary(self):
        exact = criteria()
        exact['relocalization']['max_position_error_after_ok_m'] = 0.0625
        boundary = find(F.items(CHECK, exact, shift_truth(19.95, 20.5, 0.0625)), 'B2-L6', ACCURACY)
        over = find(F.items(CHECK, criteria(), shift_truth(19.95, 20.5, 0.06)), 'B2-L6', ACCURACY)
        attempt = over['relocalization']['attempts'][0]

        self.assertEqual((boundary['result'], boundary['value']), ('PASS', 0.0625))
        self.assertEqual(over['result'], 'FAIL')
        self.assertEqual(attempt['accuracy']['ok_sim'], 20.0)

    # 접수만 있고 종료가 없으면: 기록 끝까지 기준을 넘었으면 FAIL, 아니면 절단으로 not_judged다.
    def test_accepted_only(self):
        drop = edit_diag('reinit_finished', 19.9, remove=True)
        long_items = F.items(CHECK, criteria(), drop)
        short_items = F.items(CHECK, criteria(), drop, record_end=20.0)

        self.assertEqual(find(long_items, 'B2-L6', COMPLETION)['result'], 'FAIL')
        self.assertEqual(find(short_items, 'B2-L6', COMPLETION)['result'], 'not_judged')
        self.assertEqual(find(short_items, 'B2-L6', ACCURACY)['result'], 'not_judged')

    # 시간초과·회복 실패는 정상 완료 못함(FAIL), A4 정지 상실 중단은 not_judged다. 정확도는 성공이 없어 not_judged다.
    def test_failure_kinds(self):
        timeout = F.items(CHECK, criteria(), edit_diag('reinit_finished', 19.9, result='REINIT_FAILED_TIMEOUT'))
        aborted = F.items(CHECK, criteria(), edit_diag('reinit_finished', 19.9, result='REINIT_FAILED_NOT_STOPPED'))

        self.assertEqual(find(timeout, 'B2-L6', COMPLETION)['result'], 'FAIL')
        self.assertIn('REINIT_FAILED_TIMEOUT', find(timeout, 'B2-L6', COMPLETION)['reason'])
        self.assertEqual(find(timeout, 'B2-L6', ACCURACY)['result'], 'not_judged')
        self.assertEqual(find(aborted, 'B2-L6', COMPLETION)['result'], 'not_judged')

    # 성공 뒤 OK 시점 truth가 없으면 정확도는 추측하지 않고 not_judged, 완료 시간은 따로 PASS다.
    def test_truth_missing(self):
        def drop(data):
            data['truth'] = [row for row in data['truth'] if not 19.5 <= row['stamp'] <= 21.0]

        items = F.items(CHECK, criteria(), drop)

        self.assertEqual(find(items, 'B2-L6', COMPLETION)['result'], 'PASS')
        self.assertEqual(find(items, 'B2-L6', ACCURACY)['result'], 'not_judged')

    # 다른 세션의 종료는 이 접수와 대응하지 않는다. 같은 세션 두 시도는 attempt 번호로 각각 묶는다.
    def test_sessions_and_attempts(self):
        other = F.items(CHECK, criteria(), edit_diag('reinit_finished', 19.9, session='s2'), record_end=20.0)

        def second(data):
            data['supervisor'] += [F.diag('reinit_request', 30.0, 'LOST', accepted=True, code='REINIT_ACCEPTED', attempt=2),
                                   F.diag('reinit_finished', 31.0, 'LOST', result='SUCCEEDED', attempt=2),
                                   F.diag('reinit_finished', 32.0, 'LOST', result='SUCCEEDED', attempt=9)]
            data['supervisor'].sort(key=lambda row: row['sim'])

        attempts = find(F.items(CHECK, criteria(), second), 'B2-L6', COMPLETION)['relocalization']['attempts']

        self.assertEqual(find(other, 'B2-L6', COMPLETION)['result'], 'not_judged')
        self.assertEqual([(entry['attempt'], entry['finish']['sim']) for entry in attempts], [(1, 19.9), (2, 31.0)])
        self.assertEqual({entry['finish']['basis'] for entry in attempts}, {'attempt 번호'})

    # 접수 시도가 없으면 not_applicable, 감독 진단이 없으면 not_judged다.
    def test_absent(self):
        none = F.items(CHECK, criteria(), edit_diag('reinit_request', 17.0, remove=True))
        missing = F.items(CHECK, criteria(), lambda data: data.update(supervisor=None))

        self.assertEqual(set(results(none, 'B2-L6').values()), {'not_applicable'})
        self.assertEqual(set(results(missing, 'B2-L6').values()), {'not_judged'})

    # 구동 상태 출처: stub·실제 A4(STOPPED 제공 여부)·없음을 구분해 남긴다.
    def test_a4_path(self):
        stub = CHECK.actuation_path([{'source_id': 'stub_actuation_state', 'state': 4, 'valid': True}])
        real = CHECK.actuation_path([{'source_id': 'a4_safety_gate', 'state': 0, 'valid': True}])
        stopped = CHECK.actuation_path([{'source_id': 'a4_safety_gate', 'state': 4, 'valid': True}])

        self.assertEqual(stub['basis'], 'stub')
        self.assertIn('stub', stub['real_a4_path'])
        self.assertIn('검증 불가', real['real_a4_path'])
        self.assertEqual(stopped['real_a4_path'], '실제 A4 경로')
        self.assertEqual(CHECK.actuation_path(None)['basis'], 'none')


# B2-L7 재위치 거절
class Rejection(unittest.TestCase):

    # 정상 거절 두 종류(이동 중 NOT_STOPPED, OK 중 NOT_NEEDED)와 오판 접수 0 → PASS. 이름 대응표는 명시적이다.
    def test_normal(self):
        items = F.items(CHECK, criteria())

        self.assertEqual(set(results(items, 'B2-L7').values()), {'PASS'})
        self.assertEqual(CHECK.REINIT_RESULT_CODES, {'REJECTED_NOT_STOPPED': 'REINIT_REJECTED_NOT_STOPPED',
                                                     'NOT_NEEDED': 'REINIT_REJECTED_NOT_NEEDED'})

    # 기준 이름이 대응표에 없으면(부분 문자열 'NOT_STOPPED' 등) 같은 것으로 보지 않고 not_judged다.
    def test_unknown_name(self):
        changed = criteria()
        changed['relocalization']['moving_request_result'] = 'NOT_STOPPED'

        self.assertEqual(find(F.items(CHECK, changed), 'B2-L7', MOVING)['result'], 'not_judged')

    # 이동 중 요청을 접수하면 오판 접수(FAIL), 다른 사유로 거절하면 잘못된 거절 사유(FAIL)로 따로 센다.
    def test_wrong_accept_and_reason(self):
        accept = F.items(CHECK, criteria(), edit_diag('reinit_request', 7.0, accepted=True, code='REINIT_ACCEPTED'))
        reason = F.items(CHECK, criteria(), edit_diag('reinit_request', 7.0, code='REINIT_REJECTED_MOTION_TOO_LARGE'))

        self.assertEqual(find(accept, 'B2-L7', MOVING)['value']['wrong_accept'], 1)
        self.assertEqual(find(accept, 'B2-L7', WRONG)['result'], 'FAIL')
        self.assertEqual(find(reason, 'B2-L7', MOVING)['result'], 'FAIL')
        self.assertEqual(find(reason, 'B2-L7', MOVING)['value']['wrong_reason'], 1)
        self.assertEqual(find(reason, 'B2-L7', WRONG)['result'], 'PASS')

    # A4 상태 미수신·만료로 거절된 것은 이동 거절 성공이 아니다(전제 불충족, not_judged).
    def test_a4_stale(self):
        items = F.items(CHECK, criteria(), edit_diag('reinit_request', 7.0, code='REINIT_REJECTED_ACTUATION_STALE'))
        item = find(items, 'B2-L7', MOVING)

        self.assertEqual(item['result'], 'not_judged')
        self.assertEqual(item['value']['premise_unmet'], 1)
        self.assertEqual(item['value']['correct'], 0)

    # 요청 시각 odom이 없으면(0.5 s 넘게 공백) 조건을 정하지 못해 분류하지 않는다.
    def test_state_time_missing(self):
        def drop(data):
            data['odom'] = [row for row in data['odom'] if not 6.0 <= row['stamp'] <= 7.0]

        item = find(F.items(CHECK, criteria(), drop), 'B2-L7', MOVING)

        self.assertEqual(item['result'], 'not_applicable')
        self.assertEqual(item['rejection']['requests'][1]['category'], 'unknown')

    # OK이면서 이동 중인 요청은 서버 우선순위대로 NOT_NEEDED를 기대하고, 이동·OK 조건 충족에는 세지 않는다.
    def test_ok_and_moving(self):
        items = F.items(CHECK, criteria(), edit_diag('reinit_request', 7.0, state='OK',
                                                     code='REINIT_REJECTED_NOT_NEEDED'))
        request = find(items, 'B2-L7', MOVING)['rejection']['requests'][1]

        self.assertEqual((request['category'], request['outcome']), ('ok_moving', 'correct'))
        self.assertEqual(find(items, 'B2-L7', MOVING)['result'], 'not_applicable')
        self.assertEqual(find(items, 'B2-L7', OK_REQUEST)['value']['correct'], 1)

    # 한 조건(OK)만 시험한 회차 둘을 묶으면 이동 조건 근거가 없어 B2-L7은 묶음에서 미충족이다.
    def test_batch_one_condition(self):
        events = F.evidence(CHECK, criteria(), edit_diag('reinit_request', 7.0, remove=True))
        items = CHECK.event_items('robot_2', events, criteria())
        evidence = {'source': {'how': '시험'},
                    'robots': {'robot_2': {'health_ok': {'status': 'not_judged', 'reason': '시험'}, 'events': events}},
                    'wall_clock': CHECK.wall_clock_summary(
                        CHECK.collector_wall_status({'recording': {'duration_met': True}}), {})}
        runs = []

        for index in range(2):
            result = {'status': 'completed', 'conditions': {'robot_ids': ['robot_2']}, 'robots': {'robot_2': {}}}
            judged = CHECK.judge_localization(result, criteria(), {'path': 'c', 'sha256': '0'}, None, evidence)
            runs.append((f'run{index}', result, judged, f'{index:064d}', None))

        batch = CHECK.judge_batch(runs, criteria(), {'path': 'c', 'sha256': '0'})
        entry = next(entry for entry in batch['coverage'] if entry['id'] == 'B2-L7')

        self.assertEqual(find(items, 'B2-L7', MOVING)['result'], 'not_applicable')
        self.assertFalse(entry['satisfied'])
        self.assertIn(MOVING, entry['missing_metrics'])


# 회차 판정·원시 폴더 경로
class Wiring(unittest.TestCase):

    # 사건 시험이 노출되지 않은 회차(L5~L7 not_applicable)는 다른 항목이 모두 PASS여도 회차 전체 PASS가 아니다.
    def test_unexposed_not_overall_pass(self):
        quiet = copy.deepcopy(F.streams())
        quiet['amcl_lifecycle'] = []
        quiet['supervisor'] = [row for row in quiet['supervisor'] if row['kind'] == 'tick']
        events = CHECK.event_evidence(quiet, F.identity_world_map(CHECK), criteria(), F.JUMP_LIMITS, [], 40.0,
                                      'robot_2')
        items = CHECK.event_items('robot_2', events, criteria())

        self.assertEqual({item['result'] for item in items if item['id'] in CHECK.EVENT_TEST_IDS}, {'not_applicable'})
        self.assertEqual(find(items, 'B2-L4', L4)['result'], 'PASS')

    # 원시 폴더 경로(collect_raw_evidence)가 순수 계산과 같은 근거를 만들고 원본 파일을 바꾸지 않는다.
    def test_raw_dir_path(self):
        data = F.streams()

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'raw/robot_2').mkdir(parents=True)
            (root / 'supervisor').mkdir()

            for name in ('health', 'odom', 'tf_map_odom', 'truth', 'amcl_lifecycle', 'actuation'):
                (root / f'raw/robot_2/{name}.jsonl').write_text(
                    ''.join(json.dumps(row) + '\n' for row in data[name]), encoding='utf-8')

            (root / 'supervisor/robot_2_supervisor.jsonl').write_text(
                ''.join(json.dumps(row) + '\n' for row in data['supervisor']), encoding='utf-8')
            (root / 'conditions.json').write_text(json.dumps({'time_reference': {'monotonic_s': 1000.0},
                                                              'recorded_sim_s': [0.0, 40.0]}), encoding='utf-8')
            result = {'status': 'completed', 'conditions': {'robot_ids': ['robot_2'],
                                                            'world_map': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}},
                      'recording': {'termination': {'wall_limited': False, 'mono_end': 1040.0}}}
            before = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in root.rglob('*.jsonl')}
            evidence = CHECK.collect_raw_evidence(root, ['robot_2'], result, {'how': '시험'}, criteria())
            after = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in root.rglob('*.jsonl')}

        raw_items = CHECK.event_items('robot_2', evidence['robots']['robot_2']['events'], criteria())
        pure_items = F.items(CHECK, criteria())

        self.assertEqual(before, after)
        self.assertEqual([(item['metric'], item['result'], item['value']) for item in raw_items],
                         [(item['metric'], item['result'], item['value']) for item in pure_items])
        self.assertEqual(set(results(raw_items, 'B2-L7').values()), {'PASS'})
        self.assertIn('jump_limits_source', evidence['source'])

    # 기준이 없으면(원시 근거 없음) 사건 항목은 모두 not_judged이고 사유가 있다.
    def test_no_evidence(self):
        items = CHECK.event_items('robot_2', None, criteria())

        self.assertEqual(len(items), 8)
        self.assertTrue(all(item['result'] == 'not_judged' and item['reason'] for item in items))


if __name__ == '__main__':
    unittest.main()
