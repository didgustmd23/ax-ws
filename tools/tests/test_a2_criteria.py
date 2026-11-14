# A2 정지·바퀴 odom 기준 판정(a2_criteria.py)을 가짜 결과로 검사한다. ROS·Gazebo 없이 실행한다.
import copy
import hashlib
import importlib.util
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'src/ax_control/scripts'
CRITERIA = ROOT / 'src/ax_control/config/stop_odom_criteria_dev.yaml'

sys.path.insert(0, str(SCRIPTS))

import a2_criteria  # noqa: E402

LIMITS = {
    'cmd_vel_timeout': 0.5,
    'linear.x.max_velocity': 0.67,
    'linear.x.max_deceleration': -0.67,
    'angular.z.max_velocity': 0.5,
    'angular.z.max_deceleration': -0.2,
}


# 정지 확인 회차 하나를 만든다
def stop_run(truth_stop, difference=0.0, slide=0.0, gap=0.02):
    return {
        'odom_stop_s': truth_stop - difference,
        'truth_stop_s': truth_stop,
        'stop_time_difference_s': difference,
        'odom_hold_confirmed_s': truth_stop - difference + 0.5,
        'truth_hold_confirmed_s': truth_stop + 0.5,
        'slide_after_odom_stop_m': slide,
        'max_sample_gap_s': gap,
    }


# 모든 항목이 기준 안인 정지 피드백 결과(robot_2, 3회)를 만든다
def stop_record(stop_mode='zero_command', repeats=3, status='completed'):
    truth = {'직진 0.3 m/s': 0.40, '직진 상한 0.67 m/s': 1.0, '제자리 회전 상한 0.5 rad/s': 2.4, '호 주행': 1.4}
    extra = 0.5 if stop_mode == 'command_loss' else 0.0

    return {
        'robot_id': 'robot_2',
        'repeats': repeats,
        'stop_mode': stop_mode,
        'status': status,
        'criterion': {'arm_applies': False},
        'conditions': {'has_arm': False, 'stop_mode': stop_mode, 'cmd_vel_timeout_s': 0.5,
                       'scenario_limits_from_config': dict(LIMITS)},
        'feedback': {'joint_states_max_gap_s': 0.02, 'odom_max_gap_s': 0.04},
        'runs': {name: [stop_run(value + extra) for _ in range(repeats)] for name, value in truth.items()},
    }


# 바퀴 보정 회차 하나를 만든다
def wheel_trial(label, linear, angular, seconds, ratio):
    return {
        'label': label,
        'command': {'linear_m_s': linear, 'angular_rad_s': angular, 'seconds': seconds},
        'ratio_odom_over_truth': ratio,
        'end_speed': {'max_gap_s': 0.005},
    }


# 기준 조건과 같은 바퀴 보정 결과(방향별 3회)를 만든다
def wheel_record(drive_ratio=1.0, turn_ratio=1.0, repeats=3, linear=0.3, status='completed'):
    turns = [wheel_trial(f'회전 {index}', 0.0, sign * 0.5, 4.0, turn_ratio)
             for index in range(repeats) for sign in (1.0, -1.0)]
    drives = [wheel_trial(f'직진 {index}', sign * linear, 0.0, 0.9 / linear, drive_ratio)
              for index in range(repeats) for sign in (1.0, -1.0)]

    return {'robot_id': 'robot_2', 'status': status, 'turns': turns, 'drives': drives}


# 결과 항목에서 id·지표 일부가 맞는 항목을 찾는다
def find(judged, item_id, metric_part=''):
    matches = [item for item in judged['criteria_items'] if item['id'] == item_id and metric_part in item['metric']]

    if len(matches) != 1:
        raise AssertionError(f'{item_id} {metric_part}: {len(matches)}개')

    return matches[0]


class CriteriaFile(unittest.TestCase):
    def setUp(self):
        self.criteria, self.source = a2_criteria.load_criteria(CRITERIA)

    def test_source_records_path_and_sha256(self):
        self.assertEqual(self.source['path'], str(CRITERIA.resolve()))
        self.assertEqual(self.source['sha256'], hashlib.sha256(CRITERIA.read_bytes()).hexdigest())

    def test_record_only_lists_slide(self):
        self.assertEqual(self.criteria['record_only'], ['A2-S5'])

    def test_bad_record_only_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'bad.yaml'
            text = CRITERIA.read_text(encoding='utf-8').replace('record_only:\n  - A2-S5', 'record_only: A2-S5')
            path.write_text(text, encoding='utf-8')

            with self.assertRaises(ValueError):
                a2_criteria.load_criteria(path)

    def test_missing_section_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'bad.yaml'
            path.write_text('stop: {}\n', encoding='utf-8')

            with self.assertRaises(ValueError):
                a2_criteria.load_criteria(path)

    def test_listed_theoretical_times_match_limits(self):
        # 기준 파일의 t_s 표기는 반올림 값이다. 계산 T와 0.01 s 안에서 같아야 한다.
        deceleration = self.criteria['theoretical_deceleration']

        for name, values in deceleration['scenarios'].items():
            computed = a2_criteria.theoretical_stop_time(values['linear_m_s'], values['angular_rad_s'], deceleration)
            self.assertAlmostEqual(computed, values['t_s'], delta=0.01, msg=name)


class OverallJudgment(unittest.TestCase):
    def item(self, result):
        return a2_criteria.make_item('X', 'm', 1.0, 1.0, result)

    def test_all_pass_is_pass(self):
        judged = a2_criteria.overall([self.item('PASS'), self.item('PASS')], 'completed')
        self.assertEqual(judged['judgment'], 'PASS')
        self.assertNotIn('judged_subset', judged)

    def test_any_fail_is_fail(self):
        judged = a2_criteria.overall([self.item('PASS'), self.item('FAIL'), self.item('not_judged')], 'completed')
        self.assertEqual(judged['judgment'], 'FAIL')

    def test_not_applicable_and_recorded_do_not_block_pass(self):
        items = [self.item('PASS'), self.item('not_applicable'), self.item('recorded')]
        self.assertEqual(a2_criteria.overall(items, 'completed')['judgment'], 'PASS')
        self.assertEqual(a2_criteria.overall([self.item('not_applicable')], 'completed')['judgment'], 'PASS')

    def test_partial_is_not_judged_with_subset(self):
        judged = a2_criteria.overall([self.item('PASS'), self.item('not_judged')], 'completed')
        self.assertEqual((judged['judgment'], judged['judged_subset']), ('not_judged', 'PASS'))
        self.assertIn('X', judged['judgment_reason'])

    def test_incomplete_status_is_not_judged(self):
        for status in ('failed', 'interrupted', 'running', None):
            judged = a2_criteria.overall([self.item('FAIL')], status)
            self.assertEqual(judged['judgment'], 'not_judged')
            self.assertIn('status incomplete', judged['judgment_reason'])

    def test_sample_gap_overrides_fail(self):
        judged = a2_criteria.overall([self.item('FAIL')], 'completed', 0.06, 0.05)
        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertTrue(judged['judgment_reason'].startswith('SAMPLE_GAP'))

    def test_sample_gap_at_limit_is_valid(self):
        judged = a2_criteria.overall([self.item('PASS')], 'completed', 0.05, 0.05)
        self.assertEqual(judged['judgment'], 'PASS')


class UpperAndRangeItems(unittest.TestCase):
    def test_boundary_values_pass(self):
        self.assertEqual(a2_criteria.upper_item('X', 'm', 0.05, 0.05)['result'], 'PASS')
        self.assertEqual(a2_criteria.range_item('X', 'm', [0.99, 1.01], 0.99, 1.01)['result'], 'PASS')

    def test_over_limit_fails(self):
        self.assertEqual(a2_criteria.upper_item('X', 'm', 0.0500001, 0.05)['result'], 'FAIL')
        self.assertEqual(a2_criteria.range_item('X', 'm', [1.0, 1.0100001], 0.99, 1.01)['result'], 'FAIL')

    def test_non_finite_or_missing_is_not_judged(self):
        for value in (None, math.nan, math.inf, True, 'x'):
            self.assertEqual(a2_criteria.upper_item('X', 'm', value, 0.05)['result'], 'not_judged', value)

        self.assertEqual(a2_criteria.range_item('X', 'm', [1.0, None], 0.99, 1.01)['result'], 'not_judged')
        self.assertEqual(a2_criteria.range_item('X', 'm', [], 0.99, 1.01)['result'], 'not_judged')


class StopJudgment(unittest.TestCase):
    def setUp(self):
        self.criteria, self.source = a2_criteria.load_criteria(CRITERIA)

    def judge(self, record):
        return a2_criteria.judge_stop(record, self.criteria, self.source)

    def test_good_zero_command_has_no_fail_and_lists_unjudged(self):
        judged = self.judge(stop_record())

        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertEqual(judged['judged_subset'], 'PASS')
        self.assertEqual(find(judged, 'A2-S1', '직진 0.3')['result'], 'PASS')
        self.assertEqual(find(judged, 'A2-S2', '직진 0.3')['result'], 'not_applicable')
        self.assertEqual(find(judged, 'A2-S7')['result'], 'PASS')
        self.assertEqual(find(judged, 'A2-S6')['result'], 'not_applicable')
        self.assertEqual(find(judged, 'LD-1')['result'], 'not_applicable')
        self.assertEqual(find(judged, 'A2-S5', '호 주행')['result'], 'recorded')
        self.assertEqual(find(judged, 'A2-S5', '호 주행')['value'], 0.0)

        # 남은 not_judged는 지표 없는 두 항목뿐이다.
        skipped = {(item['id'], item['metric']) for item in judged['criteria_items'] if item['result'] == 'not_judged'}
        self.assertEqual({item_id for item_id, _ in skipped}, {'A2-S3', 'A2-L1'})
        self.assertEqual(find(judged, 'A2-L1', '수신')['result'], 'PASS')

        # 지표 없는 항목은 숨기지 않고 사유와 함께 남는다.
        self.assertEqual(find(judged, 'A2-S3', '재가속')['reason'], '지표 없음(2단계)')
        self.assertEqual(find(judged, 'A2-L1', 'cmd_vel_timeout')['reason'], '지표 없음(2단계)')
        self.assertEqual(judged['criteria'], self.source)
        self.assertIsNone(judged['rtf']['rtf_min'])

    def test_zero_command_boundary_and_over_limit(self):
        limit = 0.3 / 0.67 + 0.20
        record = stop_record()
        record['runs']['직진 0.3 m/s'] = [stop_run(limit)] * 3
        self.assertEqual(find(self.judge(record), 'A2-S1', '직진 0.3')['result'], 'PASS')

        record['runs']['직진 0.3 m/s'] = [stop_run(limit + 1e-6)] * 3
        judged = self.judge(record)
        self.assertEqual(find(judged, 'A2-S1', '직진 0.3')['result'], 'FAIL')
        self.assertEqual(judged['judgment'], 'FAIL')

    def test_command_loss_uses_timeout(self):
        record = stop_record('command_loss')
        judged = self.judge(record)
        item = find(judged, 'A2-S2', '호 주행')

        self.assertEqual(item['result'], 'PASS')
        self.assertAlmostEqual(item['limit'], 0.5 + 1.5 + 0.2)
        self.assertEqual(find(judged, 'A2-S1', '호 주행')['result'], 'not_applicable')

        record['conditions']['cmd_vel_timeout_s'] = 0.4
        self.assertIn('조건 불일치', find(self.judge(record), 'A2-S2', '호 주행')['reason'])

    def test_arm_scenario_is_not_applicable(self):
        record = stop_record()
        record['runs']['팔 이동'] = [stop_run(0.3, slide=0.004)] * 3
        judged = self.judge(record)

        for item_id in ('A2-S1', 'A2-S2', 'A2-S4'):
            self.assertEqual(find(judged, item_id, '팔 이동')['result'], 'not_applicable')

        # record_only 지표는 팔 시나리오에서도 값을 남긴다.
        self.assertEqual(find(judged, 'A2-S5', '팔 이동')['result'], 'recorded')
        self.assertEqual(find(judged, 'A2-S5', '팔 이동')['value'], 0.004)

    def test_missing_limits_record_is_not_judged(self):
        record = stop_record()
        del record['conditions']['scenario_limits_from_config']
        item = find(self.judge(record), 'A2-S1', '직진 0.3')
        self.assertEqual(item['result'], 'not_judged')
        self.assertIn('명령 값 기록 없음', item['reason'])

    def test_missing_stop_mode_is_not_judged(self):
        record = stop_record()
        del record['stop_mode'], record['conditions']['stop_mode']
        judged = self.judge(record)
        self.assertEqual(find(judged, 'A2-S1', '호 주행')['result'], 'not_judged')
        self.assertEqual(find(judged, 'A2-S2', '호 주행')['result'], 'not_judged')

    def test_difference_limit_boundary_from_file(self):
        limit = self.criteria['stop']['max_odom_truth_stop_time_difference_s']
        record = stop_record()
        record['runs']['호 주행'] = [stop_run(1.4, difference=-limit)] * 3
        self.assertEqual(find(self.judge(record), 'A2-S4', '호 주행')['result'], 'PASS')

        record['runs']['호 주행'] = [stop_run(1.4, difference=-(limit + 1e-6))] * 3
        self.assertEqual(find(self.judge(record), 'A2-S4', '호 주행')['result'], 'FAIL')

    def test_record_only_item_never_fails(self):
        record = stop_record()
        record['runs']['호 주행'] = [stop_run(1.4, slide=1.0)] * 3
        judged = self.judge(record)
        self.assertEqual(find(judged, 'A2-S5', '호 주행')['result'], 'recorded')
        self.assertNotEqual(judged['judgment'], 'FAIL')

    def test_slide_is_judged_when_not_record_only(self):
        criteria = copy.deepcopy(self.criteria)
        criteria['record_only'] = []
        limit = criteria['stop']['max_post_stop_body_motion_m']
        record = stop_record()
        record['runs']['호 주행'] = [stop_run(1.4, slide=limit + 1e-6)] * 3
        judged = a2_criteria.judge_stop(record, criteria, self.source)
        self.assertEqual(find(judged, 'A2-S5', '호 주행')['result'], 'FAIL')

    def test_failed_run_fails_repeat_and_hold(self):
        record = stop_record()
        record['runs']['호 주행'][1] = None
        judged = self.judge(record)

        self.assertEqual(find(judged, 'A2-S7')['result'], 'FAIL')
        self.assertEqual(find(judged, 'A2-S3', '유지')['result'], 'FAIL')
        self.assertEqual(judged['judgment'], 'FAIL')

    def test_single_repeat_run_leaves_count_to_batch(self):
        # 회차 단독 A2-S7은 실패 회차만 본다. 반복 수는 묶음 판정에서 센다.
        self.assertEqual(find(self.judge(stop_record(repeats=1)), 'A2-S7')['result'], 'PASS')

    def test_incomplete_and_sample_gap(self):
        judged = self.judge(stop_record(status='failed'))
        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertIn('status incomplete', judged['judgment_reason'])

        record = stop_record()
        record['runs']['호 주행'][0]['max_sample_gap_s'] = 0.051
        judged = self.judge(record)
        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertTrue(judged['judgment_reason'].startswith('SAMPLE_GAP'))

    def test_input_is_not_changed(self):
        record = stop_record()
        before = copy.deepcopy(record)
        self.judge(record)
        self.assertEqual(record, before)


class WheelJudgment(unittest.TestCase):
    def setUp(self):
        self.criteria, self.source = a2_criteria.load_criteria(CRITERIA)

    def judge(self, record):
        return a2_criteria.judge_wheel(record, self.criteria, self.source)

    def test_matching_conditions_pass(self):
        judged = self.judge(wheel_record())
        self.assertEqual(judged['judgment'], 'PASS')
        self.assertEqual([item['result'] for item in judged['criteria_items']], ['PASS'] * 3)

    def test_boundary_and_outside_ratio(self):
        self.assertEqual(self.judge(wheel_record(drive_ratio=0.99, turn_ratio=1.01))['judgment'], 'PASS')

        judged = self.judge(wheel_record(turn_ratio=1.0101))
        self.assertEqual(find(judged, 'A2-O2')['result'], 'FAIL')
        self.assertEqual(judged['judgment'], 'FAIL')

    def test_more_repeats_meet_condition(self):
        self.assertEqual(find(self.judge(wheel_record(repeats=4)), 'A2-O3')['result'], 'PASS')

    def test_condition_mismatch_skips_ratios(self):
        for record in (wheel_record(repeats=2), wheel_record(linear=0.5)):
            judged = self.judge(record)
            self.assertEqual(find(judged, 'A2-O3')['result'], 'not_judged')
            self.assertEqual(find(judged, 'A2-O1')['reason'], '조건 불일치(A2-O3)')
            self.assertEqual(judged['judgment'], 'not_judged')

    def test_missing_ratio_is_not_judged(self):
        record = wheel_record()
        record['drives'][0]['ratio_odom_over_truth'] = None
        self.assertEqual(find(self.judge(record), 'A2-O1')['result'], 'not_judged')

    def test_incomplete_and_sample_gap(self):
        self.assertIn('status incomplete', self.judge(wheel_record(status='interrupted'))['judgment_reason'])

        record = wheel_record()
        record['turns'][0]['end_speed']['max_gap_s'] = 0.06
        self.assertTrue(self.judge(record)['judgment_reason'].startswith('SAMPLE_GAP'))


class Rejudge(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name) / 'result.json'

    def tearDown(self):
        self.folder.cleanup()

    def test_original_unchanged_and_judged_file_written(self):
        self.path.write_text(json.dumps(stop_record(), ensure_ascii=False), encoding='utf-8')
        before = self.path.read_bytes()

        target, judged = a2_criteria.rejudge('stop', self.path, CRITERIA)

        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(target.name, 'result.judged.json')
        written = json.loads(target.read_text(encoding='utf-8'))
        self.assertEqual(written['judgment'], judged['judgment'])
        self.assertEqual(written['rejudged_from']['sha256'], hashlib.sha256(before).hexdigest())
        self.assertEqual(written['runs'], json.loads(before)['runs'])

    def test_command_line_runs_without_ros_environment(self):
        # ROS 경로 변수를 지운 환경에서도 재판정이 돈다(rclpy를 읽지 않음).
        self.path.write_text(json.dumps(wheel_record(), ensure_ascii=False), encoding='utf-8')
        env = {key: value for key, value in os.environ.items()
               if not key.startswith(('ROS', 'AMENT', 'COLCON')) and key not in ('PYTHONPATH', 'LD_LIBRARY_PATH')}

        completed = subprocess.run(
            [sys.executable, str(SCRIPTS / 'a2_criteria.py'), 'wheel', '--rejudge', str(self.path),
             '--criteria', str(CRITERIA)],
            capture_output=True, text=True, env=env, timeout=60,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn('[판정] PASS', completed.stdout)
        self.assertTrue((Path(self.folder.name) / 'result.judged.json').exists())


# 묶음 판정 입력을 만든다(경로, 원본 바이트, 종류)
def batch_inputs(records):
    return [(f'run{index}.json', json.dumps(record, ensure_ascii=False).encode('utf-8'), None)
            for index, record in enumerate(records)]


# 동시 로봇 수·월드 해시를 기록한 정지 결과
def staged_record(count, truth_offset=0.0, repeats=3):
    record = stop_record(repeats=repeats)
    record['conditions'].update({'active_robot_count': count, 'world_sha256': 'w', 'payload_kg': 0.0})

    for runs in record['runs'].values():
        for run in runs:
            run['truth_stop_s'] += truth_offset

    return record


class BatchJudgment(unittest.TestCase):
    def setUp(self):
        self.criteria, self.source = a2_criteria.load_criteria(CRITERIA)

    def judge(self, records):
        return a2_criteria.judge_batch(batch_inputs(records), self.criteria, self.source)

    def coverage(self, result, item_id):
        return next(item for item in result['coverage'] if item['id'] == item_id)

    def test_wheel_batch_passes(self):
        result = self.judge([wheel_record(), wheel_record(repeats=4)])
        self.assertEqual(result['judgment'], 'PASS')
        self.assertEqual({item['status'] for item in result['coverage']}, {'met'})
        self.assertEqual(result['unmet'], [])
        self.assertEqual([item['result'] for item in result['batch_items']], ['not_applicable'] * 2)

    def test_fail_run_fails_batch(self):
        result = self.judge([wheel_record(), wheel_record(turn_ratio=1.02)])
        self.assertEqual(result['judgment'], 'FAIL')
        self.assertEqual(result['failures'], [{'path': 'run1.json', 'items': ['A2-O2']}])

    def test_stop_batch_lists_unmet_items(self):
        result = self.judge([staged_record(1)])
        self.assertEqual(result['judgment'], 'not_judged')
        self.assertEqual(result['judged_subset'], 'PASS')
        self.assertEqual(self.coverage(result, 'A2-S1')['status'], 'met')
        self.assertEqual(self.coverage(result, 'A2-S5')['status'], 'recorded')

        # 정지 방식 하나·팔 없는 로봇만 있으면 S2·S6은 판정된 적이 없어 미충족이다.
        for item_id in ('A2-S2', 'A2-S6'):
            self.assertIn(item_id, result['unmet'])

        # 부하 단계 정보가 있는 결과라 S7 반복은 묶음에서 판정됐다. LD-1은 L0 외 부하 회차가 없어 미충족이다.
        self.assertEqual(self.coverage(result, 'A2-S7')['status'], 'met')
        self.assertIn('batch', self.coverage(result, 'A2-S7')['evidence'])
        self.assertEqual(result['unmet'].count('LD-1'), 1)

        # S3은 유지 확인은 판정됐지만 재가속 지표가 없어 미충족이며 빠진 세부 지표를 적는다.
        self.assertEqual(self.coverage(result, 'A2-S3')['missing_metrics'], ['정지 뒤 재가속 횟수'])
        self.assertIn('A2-S3', result['unmet'])
        self.assertNotIn('A2-S5', result['unmet'])

    def test_repeats_counted_across_runs(self):
        result = self.judge([staged_record(1, repeats=1), staged_record(1, repeats=2)])
        self.assertEqual(result['batch_items'][0]['result'], 'PASS')

        result = self.judge([staged_record(1, repeats=1), staged_record(1, repeats=1)])
        item = result['batch_items'][0]
        self.assertEqual(item['result'], 'not_judged')
        self.assertIn('반복 수 부족', item['reason'])

    def test_missing_repeat_conditions_are_not_judged(self):
        result = self.judge([stop_record()])
        item = result['batch_items'][0]
        self.assertEqual(item['result'], 'not_judged')
        self.assertIn('active_robot_count', item['reason'])

        # 옛 결과(부하 단계 필드 없음): 묶음 항목은 ID 하나로만 미충족에 나오고, 빠진 세부 지표는 coverage에 있다.
        self.assertEqual(result['unmet'].count('A2-S7'), 1)
        self.assertEqual(result['unmet'].count('LD-1'), 1)
        self.assertFalse(any('(묶음)' in item_id for item_id in result['unmet']))
        self.assertTrue(self.coverage(result, 'A2-S7')['missing_metrics'][0].endswith('(묶음)'))
        self.assertEqual(self.coverage(result, 'LD-1')['evidence'], [])

    def test_wheel_only_batch_has_no_stop_batch_items_in_coverage(self):
        result = self.judge([wheel_record()])
        self.assertNotIn('A2-S7', [item['id'] for item in result['coverage']])
        self.assertNotIn('LD-1', [item['id'] for item in result['coverage']])

    def test_load_invariance_needs_stage_information(self):
        item = self.judge([stop_record()])['batch_items'][1]
        self.assertEqual((item['result'], item['reason']), ('not_judged', '부하 단계 정보 없음(3단계)'))

    def test_load_invariance_compares_with_l0(self):
        limit = self.criteria['load']['max_stop_time_difference_from_l0_s']
        result = self.judge([staged_record(1), staged_record(2, truth_offset=limit / 2)])
        self.assertEqual(result['batch_items'][1]['result'], 'PASS')
        self.assertAlmostEqual(result['batch_items'][1]['value'], limit / 2)

        result = self.judge([staged_record(1), staged_record(2, truth_offset=limit + 1e-3)])
        self.assertEqual(result['batch_items'][1]['result'], 'FAIL')
        self.assertEqual(result['judgment'], 'FAIL')
        self.assertIn({'path': 'batch', 'items': ['LD-1']}, result['failures'])

    def test_load_invariance_skips_different_worlds(self):
        other = staged_record(2, truth_offset=1.0)
        other['conditions']['world_sha256'] = 'other'
        item = self.judge([staged_record(1), other])['batch_items'][1]
        self.assertEqual(item['result'], 'not_judged')
        self.assertIn('L0와 비교할', item['reason'])

    def test_incomplete_run_is_not_evidence(self):
        result = self.judge([wheel_record(status='failed')])
        self.assertEqual(self.coverage(result, 'A2-O1')['status'], 'unmet')
        self.assertFalse(result['runs'][0]['valid'])

    def test_batch_command_keeps_inputs_and_refuses_existing_output(self):
        with tempfile.TemporaryDirectory() as folder:
            paths = []

            for index, record in enumerate((wheel_record(), staged_record(1))):
                path = Path(folder) / f'r{index}.json'
                path.write_text(json.dumps(record, ensure_ascii=False), encoding='utf-8')
                paths.append(path)

            before = [path.read_bytes() for path in paths]
            output = Path(folder) / 'out' / 'batch.json'
            target, result = a2_criteria.batch(None, paths, CRITERIA, output)

            self.assertEqual([path.read_bytes() for path in paths], before)
            self.assertEqual(json.loads(target.read_text(encoding='utf-8'))['judgment'], result['judgment'])
            self.assertEqual(result['runs'][0]['sha256'], hashlib.sha256(before[0]).hexdigest())
            self.assertEqual([run['kind'] for run in result['runs']], ['wheel', 'stop'])

            with self.assertRaises(ValueError):
                a2_criteria.batch(None, paths, CRITERIA, output)


# get_topic_names_and_types·count_publishers만 흉내 내는 그래프 조회 모의 노드
class FakeGraphNode:
    def __init__(self, publishers):
        self.publishers = publishers

    def get_topic_names_and_types(self):
        return [(name, ['sensor_msgs/msg/JointState']) for name in self.publishers]

    def count_publishers(self, name):
        return self.publishers[name]


# 가짜 /proc 항목 하나를 만든다(cmdline·environ은 NUL 구분)
def fake_process(root, pid, argv, environ):
    folder = Path(root) / str(pid)
    folder.mkdir(parents=True)
    (folder / 'cmdline').write_bytes(b'\0'.join(word.encode() for word in argv) + b'\0')
    (folder / 'environ').write_bytes(b'\0'.join(f'{k}={v}'.encode() for k, v in environ.items()) + b'\0')


class LoadConditions(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.root = Path(self.folder.name)
        self.world = self.root / 'ax_test.world'
        self.world.write_text('<sdf/>', encoding='utf-8')
        self.proc = self.root / 'proc'
        self.proc.mkdir()
        self.node = FakeGraphNode({
            '/ax/robot_2/joint_states': 1,
            '/ax/robot_1/joint_states': 2,
            '/ax/robot_3/joint_states': 0,
        })
        self.env = {'GZ_PARTITION': 'p1', 'GALLIUM_DRIVER': 'd3d12'}

    def tearDown(self):
        self.folder.cleanup()

    def test_active_robots_need_publishers(self):
        conditions = a2_criteria.measurement_load_conditions(self.node, None, self.env, self.proc)

        self.assertEqual(conditions['active_robot_ids'], ['robot_1', 'robot_2'])
        self.assertEqual(conditions['active_robot_count'], 2)
        self.assertIn('joint_states', conditions['active_robot_source'])

    def test_world_matches_server_in_same_partition(self):
        fake_process(self.proc, 10, ['ruby', '/usr/bin/gz', 'sim', '-r', '-s', str(self.world)], {'GZ_PARTITION': 'p1'})
        conditions = a2_criteria.measurement_load_conditions(self.node, self.world, self.env, self.proc)

        self.assertEqual(conditions['world_sha256'], hashlib.sha256(b'<sdf/>').hexdigest())
        self.assertEqual(conditions['world_file'], str(self.world.resolve()))
        self.assertTrue(conditions['world_check']['match'])

    def test_ruby_process_title_is_parsed(self):
        # gz(ruby)는 제목을 공백으로 이은 한 문자열로 바꿔 쓴다. 실측(2026-11-14)에서 본 형태다.
        title = f'gz sim -r -s --headless-rendering {self.world} --force-version 8' + ' ' * 40
        fake_process(self.proc, 13, [title], {'GZ_PARTITION': 'p1'})
        check = a2_criteria.measurement_load_conditions(self.node, self.world, self.env, self.proc)['world_check']

        self.assertTrue(check['match'])

    def test_relative_world_uses_process_cwd(self):
        fake_process(self.proc, 14, ['gz sim -s ax_test.world'], {'GZ_PARTITION': 'p1'})
        (self.proc / '14' / 'cwd').symlink_to(self.root)
        check = a2_criteria.measurement_load_conditions(self.node, self.world, self.env, self.proc)['world_check']

        self.assertEqual(check['server_worlds'], [str(self.world.resolve())])

    def test_other_partition_falls_back_to_argument(self):
        fake_process(self.proc, 11, ['gz', 'sim', str(self.world)], {'GZ_PARTITION': 'other'})
        check = a2_criteria.measurement_load_conditions(self.node, self.world, self.env, self.proc)['world_check']

        self.assertEqual((check['method'], check['match']), ('인자 기준', None))

    def test_different_server_world_is_recorded_as_mismatch(self):
        other = self.root / 'other.world'
        other.write_text('<sdf>x</sdf>', encoding='utf-8')
        fake_process(self.proc, 12, ['gz', 'sim', '-s', str(other)], {'GZ_PARTITION': 'p1'})
        check = a2_criteria.measurement_load_conditions(self.node, self.world, self.env, self.proc)['world_check']

        self.assertIs(check['match'], False)
        self.assertEqual(check['server_worlds'], [str(other.resolve())])

    def test_missing_world_and_renderer_values(self):
        conditions = a2_criteria.measurement_load_conditions(self.node, None, self.env, self.proc)

        self.assertIsNone(conditions['world_sha256'])
        self.assertEqual(conditions['world_reason'], '--world-file 없음')
        self.assertEqual(conditions['renderer']['GALLIUM_DRIVER'], 'd3d12')
        self.assertIsNone(conditions['renderer']['LIBGL_ALWAYS_SOFTWARE'])
        self.assertIn('측정 도구 프로세스', conditions['renderer']['source'])

    def test_recorded_fields_drive_batch_repeats(self):
        # 측정 도구가 남긴 필드 그대로 묶음 반복 수가 판정된다.
        record = stop_record(repeats=3)
        record['conditions']['payload_kg'] = 0.0
        record['conditions'].update(a2_criteria.measurement_load_conditions(self.node, self.world, self.env, self.proc))
        criteria, source = a2_criteria.load_criteria(CRITERIA)
        result = a2_criteria.judge_batch(batch_inputs([record]), criteria, source)

        self.assertEqual(result['batch_items'][0]['result'], 'PASS')
        self.assertEqual(result['runs'][0]['conditions']['active_robot_ids'], ['robot_1', 'robot_2'])


# --criteria 없이 실행할 때 결과 형식이 그대로인지(정지 도구 기록 함수). rclpy가 없으면 건너뛴다.
class StopToolWithoutCriteria(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('check_stop_feedback', SCRIPTS / 'check_stop_feedback.py')

        try:
            self.tool = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.tool)
        except ImportError as error:
            self.skipTest(f'ROS 환경 필요: {error}')

    def test_record_has_no_criteria_fields(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'result.json'
            record = self.tool.new_record('robot_2', 1)
            self.tool.write_record(path, record)
            written = json.loads(path.read_text(encoding='utf-8'))

        for key in ('criteria', 'criteria_items', 'judgment', 'judgment_reason', 'judged_subset'):
            self.assertNotIn(key, written)

    def test_parser_defaults_keep_measurement_mode(self):
        arguments = self.tool.build_parser().parse_args([])
        self.assertIsNone(arguments.criteria)
        self.assertIsNone(arguments.rejudge)
        self.assertIsNone(arguments.world_file)

    def test_scenarios_shared_with_rejudge(self):
        self.assertIs(self.tool.drive_scenarios, a2_criteria.drive_scenarios)
        names = [item[0] for item in a2_criteria.drive_scenarios(0.67, 0.5)]
        self.assertEqual(list(a2_criteria.recorded_commands(stop_record())), names)


if __name__ == '__main__':
    unittest.main()
