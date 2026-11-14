# B2 위치 추정 결과의 개발용 기준 판정(check_localization.py --criteria·--rejudge)을 ROS 없이 검사한다.
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'src/ax_localization/scripts'
CRITERIA_PATH = ROOT / 'src/ax_localization/config/localization_criteria_dev.yaml'
MAP_INFO_PATH = ROOT / 'src/ax_localization/maps/ax_warehouse/map_info.yaml'
sys.path.insert(0, str(SCRIPTS))

spec = importlib.util.spec_from_file_location('check_localization', SCRIPTS / 'check_localization.py')
CHECK = importlib.util.module_from_spec(spec)
spec.loader.exec_module(CHECK)
sys.path.insert(0, str(ROOT / 'tools/tests'))
import b2_event_fixture as EVENTS  # noqa: E402

COLUMNS = ['sim_time_s', 'est_x', 'est_y', 'est_yaw', 'true_x', 'true_y', 'true_yaw',
           'position_error_m', 'yaw_error_rad', 'odom_speed_m_s', 'odom_yaw_rate_rad_s']


# 기준 파일을 읽는다(시험마다 복사해서 쓴다).
def load_criteria():
    return yaml.safe_load(CRITERIA_PATH.read_text(encoding='utf-8'))


# 오차 시계열 한 행을 만든다(위치·방향 오차, odom 선속도·각속도).
def row(time_s, position, yaw, speed, rate):
    return [time_s, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, position, yaw, speed, rate]


# 정지 행과 주행 행으로 가짜 결과 dict를 만든다.
def fake_result(still_rows, moving_rows, status='completed', odom_gap=0.02, scan_rate=10.0, scan_gap=0.1):
    rows = still_rows + moving_rows

    return {
        'status': status,
        'judgment': 'not_judged',
        'judgment_reason': '위치 추정 정확도 기준 미확정(수치만 기록)',
        'robots': {'robot_2': {
            'scan': {'messages': 100, 'rate_hz': scan_rate, 'max_gap_s': scan_gap},
            'odom': {'messages': 500, 'rate_hz': 50.0, 'max_gap_s': odom_gap},
            'errors': {'samples': len(rows), 'series': {'columns': COLUMNS, 'rows': rows}} if rows else None,
        }},
    }


# 대상이 일치하는 대조 자료(채택 지도 map_info와 같은 해시)
def matching_context():
    info = yaml.safe_load(MAP_INFO_PATH.read_text(encoding='utf-8'))
    recorded = {'yaml': info['files']['map.yaml'], 'image': info['files']['map.pgm']}

    return {'map_info': info, 'recorded_map': recorded, 'recorded_world_sha256': info['world']['world_sha256']}


# 정상 범위 결과(정지 0.01 m·주행 0.05 m)
def good_result():
    still = [row(t, 0.01, 0.005, 0.0, 0.0) for t in range(5)]
    moving = [row(5 + t, 0.05, 0.01, 0.3, 0.0) for t in range(5)]

    return fake_result(still, moving)


# 항목 목록에서 (id, metric)으로 하나를 찾는다.
def find(items, item_id, metric):
    return next(item for item in items if item['id'] == item_id and item['metric'] == metric)


REF = {'path': 'criteria.yaml', 'sha256': '0' * 64}


# 항목별 비교와 전체 판정 규칙
class JudgeRules(unittest.TestCase):

    # 1단계 항목이 모두 기준 안이면 항목은 PASS, 지표 없는 항목이 남아 전체는 not_judged + judged_subset PASS다.
    def test_subset_pass_is_not_overall_pass(self):
        judged = CHECK.judge_localization(good_result(), load_criteria(), REF, matching_context())

        for item in judged['criteria_items']:
            if item['id'] in ('B2-L1', 'B2-L2', 'B2-L9'):
                self.assertEqual(item['result'], 'PASS', item)

        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertEqual(judged['judged_subset'], 'PASS')

        for item_id in ('B2-L3', 'B2-L4', 'B2-L5', 'B2-L6', 'B2-L7', 'LD-3'):
            self.assertIn(item_id, judged['judgment_reason'])
            listed = [item for item in judged['criteria_items'] if item['id'] == item_id]
            self.assertTrue(listed and all(item['result'] == 'not_judged' and item['reason'] for item in listed))

        # 반복(B2-L8)·부하 비교(LD-2)는 묶음 판정 항목이라 회차에서는 not_applicable(사유 있음)이다.
        for item_id in ('B2-L8', 'LD-2'):
            listed = [item for item in judged['criteria_items'] if item['id'] == item_id]
            self.assertTrue(listed and all(item['result'] == 'not_applicable' and item['reason'] for item in listed))

        self.assertEqual(judged['criteria'], REF)

    # 모든 기준 항목이 판정되고 PASS일 때만 전체 PASS다(B2-L3·LD-3은 원시 근거, B2-L4~L7은 합성 사건 기록으로 PASS).
    def test_overall_pass_when_all_items_judged(self):
        judged = CHECK.judge_localization(good_result(), load_criteria(), REF, matching_context(), full_evidence())

        self.assertEqual(judged['judgment'], 'PASS')
        self.assertNotIn('judged_subset', judged)

    # 한 항목이라도 넘으면 전체 FAIL이다(주행 최대 오차 0.2 m > 0.15 m).
    def test_fail(self):
        result = good_result()
        result['robots']['robot_2']['errors']['series']['rows'][-1][7] = 0.2
        judged = CHECK.judge_localization(result, load_criteria(), REF, matching_context())

        self.assertEqual(find(judged['criteria_items'], 'B2-L2', 'moving.max_position_error_m')['result'], 'FAIL')
        self.assertEqual(judged['judgment'], 'FAIL')
        self.assertIn('B2-L2', judged['judgment_reason'])

    # 경계값은 포함한다(정지 0.05 m·0.05 rad = 한계, 스캔 수신률 0.9 = 한계, 공백 0.5 s = 한계).
    def test_boundary_inclusive(self):
        self.assertEqual(load_criteria()['stationary']['max_yaw_error_rad'], 0.05)
        still = [row(t, 0.05, 0.05, 0.0, 0.0) for t in range(3)]
        moving = [row(3 + t, 0.05, 0.05, 0.3, 0.0) for t in range(3)]
        result = fake_result(still, moving, scan_rate=9.0, scan_gap=0.5)
        items = CHECK.judge_localization(result, load_criteria(), REF, matching_context())['criteria_items']

        for metric in ('stationary.max_position_error_m', 'stationary.max_yaw_error_rad'):
            self.assertEqual(find(items, 'B2-L1', metric)['result'], 'PASS')

        self.assertEqual(find(items, 'B2-L2', 'moving.max_yaw_error_rad')['result'], 'PASS')
        self.assertEqual(find(items, 'B2-L9', 'load.min_scan_receive_ratio')['result'], 'PASS')
        self.assertEqual(find(items, 'B2-L9', 'load.max_scan_gap_s')['result'], 'PASS')

        # 경계 바로 밖은 FAIL
        still = [row(t, 0.05, 0.0501, 0.0, 0.0) for t in range(3)]
        result = fake_result(still, moving, scan_rate=8.99, scan_gap=0.501)
        items = CHECK.judge_localization(result, load_criteria(), REF, matching_context())['criteria_items']
        self.assertEqual(find(items, 'B2-L1', 'stationary.max_yaw_error_rad')['result'], 'FAIL')
        self.assertEqual(find(items, 'B2-L9', 'load.min_scan_receive_ratio')['result'], 'FAIL')
        self.assertEqual(find(items, 'B2-L9', 'load.max_scan_gap_s')['result'], 'FAIL')

    # 정지 구간은 odom 속도가 D-06 값 이하인 표본이다. 0.01 m/s·0.02 rad/s는 정지, 그보다 크면 주행이다.
    def test_segment_threshold_d06(self):
        rows = [row(0, 0.01, 0.0, 0.01, 0.02), row(1, 0.01, 0.0, 0.0101, 0.0), row(2, 0.01, 0.0, 0.0, 0.0201)]
        still, moving = CHECK.split_by_motion({'series': {'columns': COLUMNS, 'rows': rows}})

        self.assertEqual([item[0] for item in still], [0])
        self.assertEqual([item[0] for item in moving], [1, 2])

    # 주행 구간 표본이 없으면 B2-L2는 not_applicable(사유 "주행 구간 없음")이다.
    def test_no_moving_segment(self):
        still = [row(t, 0.01, 0.005, 0.0, 0.0) for t in range(5)]
        judged = CHECK.judge_localization(fake_result(still, []), load_criteria(), REF, matching_context())

        for item in judged['criteria_items']:
            if item['id'] == 'B2-L2':
                self.assertEqual(item['result'], 'not_applicable')
                self.assertIn('주행 구간 없음', item['reason'])

        self.assertNotIn('B2-L2', judged['judgment_reason'])

        self.assertEqual(find(judged['criteria_items'], 'B2-L1', 'stationary.max_position_error_m')['result'], 'PASS')
        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertEqual(judged['criteria_method']['segment_samples']['robot_2'],
                         {'stationary_samples': 5, 'moving_samples': 0})

    # 비유한·없는 값은 not_judged다.
    def test_nonfinite_value(self):
        result = good_result()
        result['robots']['robot_2']['scan']['max_gap_s'] = math.nan
        result['robots']['robot_2']['scan']['rate_hz'] = None
        items = CHECK.judge_localization(result, load_criteria(), REF, matching_context())['criteria_items']

        self.assertEqual(find(items, 'B2-L9', 'load.max_scan_gap_s')['result'], 'not_judged')
        self.assertEqual(find(items, 'B2-L9', 'load.min_scan_receive_ratio')['result'], 'not_judged')

    # status incomplete면 FAIL 항목이 있어도 전체 not_judged(사유 status incomplete)다.
    def test_incomplete(self):
        result = good_result()
        result['status'] = 'incomplete'
        result['robots']['robot_2']['errors']['series']['rows'][-1][7] = 0.2
        judged = CHECK.judge_localization(result, load_criteria(), REF, matching_context())

        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertTrue(judged['judgment_reason'].startswith('status incomplete'))

    # 기준에 max_sample_interval_s가 있으면(다른 기준 파일 호환) odom 기록 간격이 넘을 때 전체 not_judged(SAMPLE_GAP)다.
    # 같으면 통과한다. B2 기준 파일에는 이 키가 없으므로 시험용 기준에 키를 명시한다.
    def test_sample_gap_when_key_present(self):
        criteria = load_criteria()
        criteria['load']['max_sample_interval_s'] = 0.05
        result = good_result()
        result['robots']['robot_2']['odom']['max_gap_s'] = 0.125
        judged = CHECK.judge_localization(result, criteria, REF, matching_context())

        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertTrue(judged['judgment_reason'].startswith('SAMPLE_GAP'))

        result['robots']['robot_2']['odom']['max_gap_s'] = 0.05
        judged = CHECK.judge_localization(result, criteria, REF, matching_context())
        self.assertNotIn('SAMPLE_GAP', judged['judgment_reason'])

    # 사용자 결정(2026-11-14): B2 기준 파일에서 표본 간격 조건을 뺐다(A2 정지 시각용 조건, B2 오차 표본은 0.5 s 간격).
    # 기준 완화가 아니라 적용 대상 정리다. 키가 없으면 odom 간격이 커도 SAMPLE_GAP으로 무효 처리하지 않는다.
    def test_sample_gap_not_applied_without_key(self):
        criteria = load_criteria()
        self.assertNotIn('max_sample_interval_s', criteria['load'])
        result = good_result()
        result['robots']['robot_2']['odom']['max_gap_s'] = 0.125
        judged = CHECK.judge_localization(result, criteria, REF, matching_context())

        self.assertNotIn('SAMPLE_GAP', judged['judgment_reason'])
        self.assertEqual(judged['judged_subset'], 'PASS')
        self.assertEqual(judged['criteria_method']['sample_gap_basis'], '표본 간격 조건: 기준 파일에 없음(적용 안 함)')

    # rtf_min이 null이면 RTF로 판정하지 않고 기록만 한다.
    def test_rtf_null_recorded_only(self):
        judged = CHECK.judge_localization(good_result(), load_criteria(), REF, matching_context())

        self.assertIsNone(load_criteria()['load']['rtf_min'])
        self.assertIn('기록만', judged['criteria_method']['rtf'])
        self.assertFalse(any('RTF' in item['id'] for item in judged['criteria_items']))

    # 정지 구간 표본이 없으면 B2-L1은 not_applicable, 오차 표본이 아예 없으면 not_judged다.
    def test_segment_absent_vs_no_errors(self):
        moving = [row(t, 0.05, 0.01, 0.3, 0.0) for t in range(5)]
        judged = CHECK.judge_localization(fake_result([], moving), load_criteria(), REF, matching_context())
        items = judged['criteria_items']
        self.assertEqual(find(items, 'B2-L1', 'stationary.max_position_error_m')['result'], 'not_applicable')
        self.assertIn('정지 구간 없음', find(items, 'B2-L1', 'stationary.max_position_error_m')['reason'])

        judged = CHECK.judge_localization(fake_result([], []), load_criteria(), REF, matching_context())
        items = judged['criteria_items']
        self.assertEqual(find(items, 'B2-L1', 'stationary.max_position_error_m')['result'], 'not_judged')
        self.assertEqual(find(items, 'B2-L2', 'moving.max_position_error_m')['result'], 'not_judged')

    # 사건 시험이 노출된 회차에서 not_applicable(주행 구간 없음 등)만 남으면 회차 판정은 PASS가 될 수 있다.
    def test_pass_with_not_applicable_only(self):
        still = [row(t, 0.01, 0.005, 0.0, 0.0) for t in range(5)]
        judged = CHECK.judge_localization(fake_result(still, []), load_criteria(), REF, matching_context(),
                                          full_evidence(stationary=True))

        self.assertEqual(judged['judgment'], 'PASS')
        self.assertIn('not_applicable', {item['result'] for item in judged['criteria_items']})

    # record_only 항목은 판정하지 않고 값을 남긴다(recorded). FAIL 값이어도 회차 FAIL로 만들지 않는다.
    def test_record_only(self):
        criteria = load_criteria()
        criteria['record_only'] = ['B2-L9']
        result = good_result()
        result['robots']['robot_2']['scan']['max_gap_s'] = 2.0

        judged = CHECK.judge_localization(result, criteria, REF, matching_context(), full_evidence())
        gap = find(judged['criteria_items'], 'B2-L9', 'load.max_scan_gap_s')
        self.assertEqual(gap['result'], 'recorded')
        self.assertEqual(gap['value'], 2.0)
        self.assertEqual(judged['judgment'], 'PASS')


# 대상(지도 revision·해시, 월드 해시) 대조
class TargetCheck(unittest.TestCase):

    # 회차 지도 해시가 채택 지도와 다르면 전체 not_judged "대상 불일치"다.
    def test_map_hash_mismatch(self):
        context = matching_context()
        context['recorded_map'] = dict(context['recorded_map'], yaml='f' * 64)
        judged = CHECK.judge_localization(good_result(), load_criteria(), REF, context)

        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertTrue(judged['judgment_reason'].startswith('대상 불일치'))

    # 기준 revision과 map_info revision이 다르면 대상 불일치다.
    def test_revision_mismatch(self):
        criteria = load_criteria()
        criteria['target']['map_revision'] = 2
        judged = CHECK.judge_localization(good_result(), criteria, REF, matching_context())

        self.assertTrue(judged['judgment_reason'].startswith('대상 불일치'))

    # 회차 조건이 없으면 대조하지 못한 사실을 사유에 남기고 판정은 계속한다.
    def test_unchecked_recorded(self):
        context = copy.deepcopy(matching_context())
        context.update(recorded_map=None, recorded_world_sha256=None)
        judged = CHECK.judge_localization(good_result(), load_criteria(), REF, context)

        self.assertIn('대상 대조 불가', judged['judgment_reason'])
        self.assertEqual(judged['judged_subset'], 'PASS')
        self.assertTrue(judged['criteria_method']['target_unchecked'])

    # 기준 파일 위치에서 저장소 루트를 찾아 채택 지도의 map_info.yaml을 읽는다.
    def test_target_context_reads_map_info(self):
        context = CHECK.target_context(load_criteria(), CRITERIA_PATH, None)

        self.assertEqual(context['map_info']['map_revision'], 1)
        self.assertIsNone(context['recorded_map'])


# CLI: 과거 결과 재판정과 --criteria 없는 기존 출력
class CommandLine(unittest.TestCase):

    # --rejudge는 원본을 바꾸지 않고 같은 폴더에 *.judged.json을 새로 쓴다(ROS 없이).
    def test_rejudge_writes_new_file(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder) / 'run'
            (run / 'result').mkdir(parents=True)
            context = matching_context()
            conditions = {'map': {'yaml': context['recorded_map']['yaml'], 'image': context['recorded_map']['image']},
                          'files_sha256': {}}
            (run / 'conditions.json').write_text(json.dumps(conditions), encoding='utf-8')
            original = run / 'result' / 'localization_result.json'
            original.write_text(json.dumps(good_result(), ensure_ascii=False), encoding='utf-8')
            before = original.read_bytes()

            completed = subprocess.run(
                [sys.executable, str(SCRIPTS / 'check_localization.py'), '--rejudge', str(original),
                 '--criteria', str(CRITERIA_PATH)],
                capture_output=True, text=True, timeout=60)

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(original.read_bytes(), before)
            judged_path = run / 'result' / 'localization_result.judged.json'
            judged = json.loads(judged_path.read_text(encoding='utf-8'))
            self.assertEqual(judged['criteria']['sha256'], hashlib.sha256(CRITERIA_PATH.read_bytes()).hexdigest())
            self.assertEqual(judged['rejudged_from']['sha256'], hashlib.sha256(before).hexdigest())
            self.assertEqual(judged['rejudged_from']['original_judgment'], 'not_judged')
            self.assertEqual(judged['judged_subset'], 'PASS')
            self.assertIn('회차 월드 파일 해시 기록 없음', judged['judgment_reason'])

    # --rejudge에 --criteria가 없으면 인자 오류로 끝나고 파일을 쓰지 않는다.
    def test_rejudge_requires_criteria(self):
        with tempfile.TemporaryDirectory() as folder:
            original = Path(folder) / 'localization_result.json'
            original.write_text(json.dumps(good_result()), encoding='utf-8')
            completed = subprocess.run(
                [sys.executable, str(SCRIPTS / 'check_localization.py'), '--rejudge', str(original)],
                capture_output=True, text=True, timeout=60)

            self.assertEqual(completed.returncode, 2)
            self.assertEqual(sorted(path.name for path in Path(folder).iterdir()), ['localization_result.json'])

    # --criteria가 없으면 summarize 결과의 판정 문자열은 기존 그대로이고 기준 필드가 없다.
    def test_without_criteria_unchanged(self):
        arguments = SimpleNamespace(scan='filtered', duration=1.0, world_map=[0.0, 0.0, 0.0], world_map_source='x',
                                    tf_future=1.0, label='')
        records = {'robot_2': {'map_odom': [], 'odom': [], 'amcl_pose': [], 'initialpose': [], 'scan': []}}
        result = CHECK.summarize(records, {}, CHECK.se2(0.0, 0.0, 0.0), {}, [], [], arguments)

        self.assertEqual(result['judgment'], 'not_judged')
        self.assertEqual(result['judgment_reason'], '위치 추정 정확도 기준 미확정(수치만 기록)')

        for key in ('criteria', 'criteria_items', 'judged_subset', 'criteria_method'):
            self.assertNotIn(key, result)


# 같은 동작 설정 서명(시험용)
SIGNATURE = {name: 'a' * 64 for name in CHECK.OPERATING_FILES}


# 회차 판정에 회차 근거(경로 완주·활성 연속성·동작 설정 서명)와 감독 세션을 넣는다(시험용, 실제 근거는 원시 폴더에서 만든다).
def evidenced(judgment, name, route=True, continuity='constant', signature=SIGNATURE, route_robot='robot_2',
              sessions=None):
    judgment['criteria_method']['run_evidence'] = {
        'route': {'verified': route, 'reason': '' if route else '시험: 경로 근거 없음', 'robot_id': route_robot},
        'active_continuity': {'status': continuity}, 'operating_signature': signature}

    for item in judgment['criteria_items']:
        if item['id'] == 'B2-L3':
            item.setdefault('health_ok', {}).setdefault('health', {})['sessions'] = sessions or [f'session-{name}']

    return judgment


# 결과의 측정 대상 로봇마다 robot_2와 같은 오차 자료를 둔다(두 로봇 회차 시험용).
def with_robots(result, robot_ids):
    for robot_id in robot_ids:
        result['robots'].setdefault(robot_id, copy.deepcopy(result['robots']['robot_2']))

    return result


# 묶음 판정용 회차 하나(경로, 원본, 회차 판정, sha256, 조건)를 만든다. staged면 활성 로봇 실측 기록(=측정 대상)과
# 회차 근거를 넣는다. staged=False는 활성 로봇 기록이 없는 옛 기록이다.
def batch_run(name, result, robot_ids=('robot_2',), criteria=None, staged=True, **evidence):
    result = with_robots(copy.deepcopy(result), robot_ids)
    result['conditions'] = {'robot_ids': list(robot_ids)}

    if staged:
        result['conditions'].update(active_robot_ids=sorted(robot_ids), active_robot_count=len(robot_ids))

    judgment = CHECK.judge_localization(result, criteria or load_criteria(), REF, matching_context())

    return (name, result, evidenced(judgment, name, **evidence), '0' * 64, None)


# 여러 회차 묶음 판정(충족·반복 수·FAIL·입력 무변경)
class BatchJudge(unittest.TestCase):

    # 지정한 항목이 모두 어느 회차에서든 판정·PASS면 묶음 PASS다(1단계 미지표 항목을 빼고 본 경우).
    def test_batch_pass(self):
        runs = [batch_run(f'r{index}', good_result()) for index in range(3)]

        with mock.patch.object(CHECK, 'BATCH_COVERAGE_IDS', ('B2-L1', 'B2-L2', 'B2-L9')):
            judged = CHECK.judge_batch(runs, load_criteria(), REF)

        self.assertEqual(judged['judgment'], 'PASS')
        self.assertTrue(all(entry['satisfied'] for entry in judged['coverage']))

    # FAIL 회차가 하나라도 있으면 묶음 FAIL이고 그 회차를 사유에 적는다.
    def test_batch_fail(self):
        bad = good_result()
        bad['robots']['robot_2']['errors']['series']['rows'][-1][7] = 0.2
        judged = CHECK.judge_batch([batch_run('ok', good_result()), batch_run('bad', bad)], load_criteria(), REF)

        self.assertEqual(judged['judgment'], 'FAIL')
        self.assertIn('bad', judged['judgment_reason'])

    # 실제 기준으로는 지표 없는 항목·반복 부족이 미충족으로 나열되고 묶음은 not_judged + judged_subset이다.
    def test_batch_unmet_listed(self):
        still = [row(t, 0.01, 0.005, 0.0, 0.0) for t in range(5)]
        runs = [batch_run('still', fake_result(still, [])), batch_run('move', good_result())]
        judged = CHECK.judge_batch(runs, load_criteria(), REF)

        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertEqual(judged['judged_subset'], 'PASS')

        for item_id in ('B2-L3', 'B2-L4', 'B2-L5', 'B2-L6', 'B2-L7', 'B2-L8', 'LD-2', 'LD-3'):
            self.assertIn(item_id, judged['unmet'])

        coverage = {entry['id']: entry for entry in judged['coverage']}
        self.assertTrue(coverage['B2-L2']['satisfied'])
        self.assertEqual(coverage['B2-L2']['runs'], ['move'])
        ld2 = next(item for item in judged['batch_items'] if item['id'] == 'LD-2')
        self.assertTrue(ld2['reason'].startswith('부하 단계 정보 없음'), ld2['reason'])

    # B2-L8: robot_2 단독 2회는 3회 요구에 부족(미충족, FAIL 아님), 두 로봇 동시 1회는 충족이다.
    def test_repeat_counts(self):
        runs = [batch_run('s1', good_result()), batch_run('s2', good_result()),
                batch_run('pair', good_result(), robot_ids=('robot_1', 'robot_2'))]
        judged = CHECK.judge_batch(runs, load_criteria(), REF)
        items = {item['metric']: item for item in judged['batch_items'] if item['id'] == 'B2-L8'}

        self.assertEqual(items['repeats.robot_2_single_runs']['result'], 'not_judged')
        self.assertIn('회차 수 부족 2/3', items['repeats.robot_2_single_runs']['reason'])
        self.assertEqual(items['repeats.two_robot_simultaneous_runs_min']['result'], 'PASS')
        self.assertEqual(items['repeats.two_robot_simultaneous_runs_min']['runs'], ['pair'])
        # 경로 근거(시험용 run_evidence)가 있는 회차만 세며 min_route는 그 회차 수로 판정한다.
        self.assertEqual(items['repeats.min_route']['result'], 'PASS')
        self.assertEqual(items['repeats.min_route']['runs'], ['pair', 's1', 's2'])
        self.assertNotEqual(judged['judgment'], 'FAIL')

        # 단독 3회면 그 지표는 PASS
        runs.append(batch_run('s3', good_result()))
        judged = CHECK.judge_batch(runs, load_criteria(), REF)
        items = {item['metric']: item for item in judged['batch_items'] if item['id'] == 'B2-L8'}
        self.assertEqual(items['repeats.robot_2_single_runs']['result'], 'PASS')

    # B2-L8은 주행 표본이 있는 회차(B2-L2가 PASS·FAIL로 판정된 회차)만 센다. 정지만 한 회차는 빼며,
    # 이는 기준값 변경이 아니라 기준 문구("경로 center 이상")에 맞춘 해석 정리다(2026-11-14).
    def test_repeat_counts_exclude_stationary_only(self):
        still = [row(t, 0.01, 0.005, 0.0, 0.0) for t in range(5)]
        runs = [batch_run(f'still{index}', fake_result(still, [])) for index in range(3)]
        runs.append(batch_run('move', good_result()))
        judged = CHECK.judge_batch(runs, load_criteria(), REF)
        single = next(item for item in judged['batch_items'] if item['metric'] == 'repeats.robot_2_single_runs')

        self.assertEqual(single['value'], 1)
        self.assertEqual(single['runs'], ['move'])
        self.assertEqual(single['result'], 'not_judged')
        self.assertIn('회차 수 부족 1/3', single['reason'])
        self.assertEqual([run['moving_judged'] for run in judged['runs']], [False, False, False, True])

    # 로봇 목록을 읽을 수 없는 회차가 있으면 반복 항목은 not_judged(사유)다.
    def test_repeat_unreadable_robots(self):
        name, result, judgment, digest, conditions = batch_run('x', good_result())
        result['conditions'] = {}
        result['robots'] = None
        judged = CHECK.judge_batch([(name, result, judgment, digest, conditions)], load_criteria(), REF)
        items = [item for item in judged['batch_items'] if item['id'] == 'B2-L8' and 'runs' in item['metric']]

        self.assertTrue(items and all(item['result'] == 'not_judged' and '읽을 수 없는' in item['reason']
                                      for item in items))

    # CLI --batch: 입력 원본은 바뀌지 않고 출력 JSON 하나를 쓴다.
    def test_batch_cli_inputs_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            paths = []

            for index in range(2):
                path = Path(folder) / f'run{index}' / 'localization_result.json'
                path.parent.mkdir()
                path.write_text(json.dumps(good_result(), ensure_ascii=False), encoding='utf-8')
                paths.append(path)

            before = [path.read_bytes() for path in paths]
            output = Path(folder) / 'batch' / 'batch.json'
            completed = subprocess.run(
                [sys.executable, str(SCRIPTS / 'check_localization.py'), '--batch', *map(str, paths),
                 '--criteria', str(CRITERIA_PATH), '--output', str(output)],
                capture_output=True, text=True, timeout=60)

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual([path.read_bytes() for path in paths], before)
            judged = json.loads(output.read_text(encoding='utf-8'))
            self.assertEqual([run['sha256'] for run in judged['runs']],
                             [hashlib.sha256(data).hexdigest() for data in before])
            self.assertEqual(judged['criteria']['sha256'], hashlib.sha256(CRITERIA_PATH.read_bytes()).hexdigest())
            self.assertEqual(sorted(entry.name for entry in Path(folder).iterdir()), ['batch', 'run0', 'run1'])


# 부하 단계 정보를 넣은 회차(활성 로봇 수·월드 해시)를 만든다. moving_p95: 주행 오차 값(m)
# robot_ids를 주지 않으면 활성 로봇과 같게 둔다(활성 로봇이 모두 측정 대상).
def staged_run(name, active_ids, world='w' * 64, moving_error=0.05, robot_ids=None, **evidence):
    robot_ids = tuple(sorted(active_ids)) if robot_ids is None else robot_ids
    still = [row(t, 0.01, 0.005, 0.0, 0.0) for t in range(5)]
    moving = [row(5 + t, moving_error, 0.01, 0.3, 0.0) for t in range(5)]
    result = with_robots(fake_result(still, moving), robot_ids)
    result['conditions'] = {'robot_ids': list(robot_ids), 'active_robot_ids': sorted(active_ids),
                            'active_robot_count': len(active_ids), 'world_sha256': world}
    context = matching_context()
    context['recorded_world_sha256'] = None
    judgment = CHECK.judge_localization(result, load_criteria(), REF, context)

    return (name, result, evidenced(judgment, name, **evidence), '0' * 64, None)


# 반복·부하 근거 조건(경로 완주·활성 불변·같은 설정·고유 세션), 2026-11-14 B2-L8·LD-2 근거 연결
class RepeatEvidence(unittest.TestCase):

    # 반복 항목 dict
    def items(self, runs):
        judged = CHECK.judge_batch(runs, load_criteria(), REF)

        return {item['metric']: item for item in judged['batch_items'] if item['id'] == 'B2-L8'}, judged

    # 같은 세션(같은 회차를 두 번 넣거나 복사본·재판정 사본)은 한 번만 센다.
    def test_duplicate_sessions_counted_once(self):
        runs = [batch_run('a', good_result()), batch_run('a_copy', good_result(), sessions=['session-a']),
                batch_run('b', good_result())]
        items, _ = self.items(runs)
        single = items['repeats.robot_2_single_runs']

        self.assertEqual((single['value'], single['runs']), (2, ['a', 'b']))
        self.assertIn('같은 세션 중복(a)', dict(single['excluded'])['a_copy'])

    # 경로 근거 없음·활성 로봇 변화·확인 불가·설정 서명 없음·주행 없음·무효 회차는 반복 근거가 아니다.
    def test_excluded_runs(self):
        still = [row(t, 0.01, 0.005, 0.0, 0.0) for t in range(5)]
        bad = good_result()
        bad['status'] = 'incomplete'
        runs = [batch_run('no_route', good_result(), route=False), batch_run('changed', good_result(), continuity='changed'),
                batch_run('unverified', good_result(), continuity='unverified'),
                batch_run('no_signature', good_result(), signature=None), batch_run('still', fake_result(still, [])),
                batch_run('invalid', bad), batch_run('ok', good_result())]
        items, _ = self.items(runs)
        excluded = dict(items['repeats.robot_2_single_runs']['excluded'])

        self.assertEqual(items['repeats.robot_2_single_runs']['runs'], ['ok'])
        self.assertIn('경로 근거 없음', excluded['no_route'])
        self.assertIn('활성 로봇 changed', excluded['changed'])
        self.assertIn('활성 로봇 unverified', excluded['unverified'])
        self.assertIn('동작 설정 서명 없음', excluded['no_signature'])
        self.assertIn('주행 표본 없음', excluded['still'])
        self.assertIn('무효 회차', excluded['invalid'])

    # 경로 근거가 있는 회차가 없으면 min_route는 not_judged다.
    def test_min_route_without_evidence(self):
        items, _ = self.items([batch_run('x', good_result(), route=False)])

        self.assertEqual(items['repeats.min_route']['result'], 'not_judged')
        self.assertIn('검증된 회차 없음', items['repeats.min_route']['reason'])

    # 동작 설정이 다른 회차가 섞이면 반복 수를 확정하지 않고, LD-2도 다른 설정끼리 비교하지 않는다.
    def test_mixed_settings(self):
        other = dict(SIGNATURE, **{'amcl.yaml': 'b' * 64})
        runs = [staged_run('l0a', ['robot_2']), staged_run('l0b', ['robot_2']),
                staged_run('l1', ['robot_1', 'robot_2'], signature=other)]
        items, judged = self.items(runs)
        ld2 = find(judged['batch_items'], 'LD-2', 'load.max_moving_p95_increase_from_l0_m')

        self.assertEqual(items['repeats.robot_2_single_runs']['result'], 'not_judged')
        self.assertIn('동작 설정·월드가 다른', items['repeats.robot_2_single_runs']['reason'])
        self.assertEqual(ld2['result'], 'not_judged')
        self.assertIn('짝 없음', ld2['reason'])

    # 활성 2대지만 robot_1을 측정하지 않았거나(측정 대상 robot_2뿐) robot_1 오차 자료가 없으면 동시 활성 근거가 아니고
    # LD-2 비교도 하지 않는다.
    def test_two_robot_data_required(self):
        unmeasured = staged_run('pair', ['robot_1', 'robot_2'], robot_ids=('robot_2',))
        name, result, judgment, digest, conditions = staged_run('pair2', ['robot_1', 'robot_2'])
        judgment['criteria_items'] = [item for item in judgment['criteria_items']
                                      if not (item['robot'] == 'robot_1' and item['id'] in ('B2-L1', 'B2-L2'))]
        runs = [staged_run('l0', ['robot_2']), unmeasured, (name, result, judgment, digest, conditions)]
        items, judged = self.items(runs)
        ld2 = find(judged['batch_items'], 'LD-2', 'load.max_moving_p95_increase_from_l0_m')

        self.assertEqual(items['repeats.two_robot_simultaneous_runs_min']['runs'], [])
        self.assertEqual(ld2['result'], 'not_judged')
        self.assertIn('로봇 자료 부족', ld2['reason'])

    # LD-2 비교는 L0(robot_2 단독)에 있는 로봇만 한다. 기준선 값·회차를 기록한다.
    def test_ld2_records_baseline(self):
        runs = [staged_run('l0a', ['robot_2'], moving_error=0.04), staged_run('l0b', ['robot_2'], moving_error=0.06),
                staged_run('l1', ['robot_1', 'robot_2'], moving_error=0.07)]
        ld2 = find(CHECK.judge_batch(runs, load_criteria(), REF)['batch_items'], 'LD-2',
                   'load.max_moving_p95_increase_from_l0_m')

        self.assertEqual([entry['robot'] for entry in ld2['comparisons']], ['robot_2'])
        self.assertEqual([entry[0] for entry in ld2['comparisons'][0]['l0_values']], ['l0a', 'l0b'])

    # 회차 근거 원시 폴더 함수: 경로 완주(기준 경로·실행 전 해시·주행 결과·관문), 부분 주행·해시 불일치·결과 없음은 미검증
    def test_route_evidence(self):
        criteria = load_criteria()
        route_path, route_sha = CHECK.criteria_route(criteria)
        drive = {'status': 'completed', 'error': None, 'robot_id': 'robot_2', 'waypoints_reached': 2,
                 'route': {'path': str(route_path), 'sha256': route_sha, 'partial': False,
                           'waypoints': [[1.4, 0.0, False], [12.4, 0.0, False]]},
                 'world': {'file': 'planned/ax_warehouse.world'},
                 'stops': [{'reason': '관측 관문 P1', 'confirmed': True}, {'reason': '관측 관문 P2', 'confirmed': True}]}
        conditions = {'files_sha256': {'route': {'path': str(route_path), 'sha256': route_sha}}}
        cases = {'ok': (drive, conditions, True),
                 'partial': (dict(drive, waypoints_reached=1, route=dict(drive['route'], partial=True)), conditions, False),
                 'other_route': (dict(drive, route=dict(drive['route'], sha256='c' * 64)), conditions, False),
                 'no_pre_hash': (drive, {}, False),
                 'failed': (dict(drive, status='failed'), conditions, False),
                 'gate_unconfirmed': (dict(drive, stops=[{'reason': '관측 관문 P1', 'confirmed': False}]), conditions,
                                      False),
                 'missing': (None, conditions, False)}

        for name, (record, cond, expected) in cases.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as folder:
                if record is not None:
                    (Path(folder) / 'drive').mkdir()
                    (Path(folder) / 'drive' / 'drive_result.json').write_text(json.dumps(record), encoding='utf-8')

                evidence = CHECK.route_evidence(Path(folder), cond, criteria)
                self.assertEqual(evidence['verified'], expected, evidence['reason'])

    # 활성 로봇 연속성: 한 집합 유지 constant, 바뀌면 changed, 감시 공백·기록 없음은 unverified
    def test_active_continuity(self):
        watch = [{'recv_mono': 100.0 + t} for t in range(0, 31)]
        same = [{'recv_mono': 101.0, 'active_robot_ids': ['robot_2']}]
        added = same + [{'recv_mono': 115.0, 'active_robot_ids': ['robot_1', 'robot_2']}]

        self.assertEqual(CHECK.active_continuity(same, watch, [100.0, 130.0])['status'], 'constant')
        self.assertEqual(CHECK.active_continuity(added, watch, [100.0, 130.0])['status'], 'changed')
        self.assertEqual(CHECK.active_continuity(same, watch[:10] + watch[20:], [100.0, 130.0])['status'], 'unverified')
        self.assertEqual(CHECK.active_continuity([], watch, [100.0, 130.0])['status'], 'unverified')

        # 시작 직후 그래프 탐색 전 빈 집합(3 s 안)은 변화가 아니고, 그 뒤의 빈 집합·다른 집합은 변화다.
        warm = [{'recv_mono': 100.5, 'active_robot_ids': []}] + [dict(same[0])]
        late = [{'recv_mono': 100.5, 'active_robot_ids': []}, {'recv_mono': 110.0, 'active_robot_ids': ['robot_2']}]
        dropped = warm + [{'recv_mono': 120.0, 'active_robot_ids': []}]
        result = CHECK.active_continuity(warm, watch, [100.0, 130.0])

        self.assertEqual((result['status'], result['discovery_warmup_records']), ('constant', 1))
        self.assertEqual(CHECK.active_continuity(late, watch, [100.0, 130.0])['status'], 'changed')
        self.assertEqual(CHECK.active_continuity(dropped, watch, [100.0, 130.0])['status'], 'changed')
        self.assertEqual(CHECK.active_continuity(same, watch, [90.0, 130.0])['status'], 'unverified')

    # 동작 설정 서명은 정해진 파일이 모두 있을 때만 만든다(평가 도구 해시는 넣지 않음).
    def test_operating_signature(self):
        files = {name: {'sha256': 'd' * 64} for name in CHECK.OPERATING_FILES}

        self.assertEqual(set(CHECK.operating_signature({'files_sha256': files})), set(CHECK.OPERATING_FILES))
        self.assertNotIn('check_localization.py', CHECK.OPERATING_FILES)
        files.pop('route')
        self.assertIsNone(CHECK.operating_signature({'files_sha256': files}))


# 측정 도구가 결과 conditions에 남기는 부하 단계 정보(그래프 조회는 모의)
class LoadStageRecord(unittest.TestCase):

    # /ax/<id>/joint_states에 발행자가 있는 로봇만 활성으로 보고 정렬한다. 구독자만 있는 토픽은 뺀다.
    def test_active_robots_from_topics(self):
        names = ['/ax/robot_2/joint_states', '/ax/robot_1/joint_states', '/ax/robot_1/diff_drive_controller/odom',
                 '/ax/robot_3/odom', '/clock', '/ax//joint_states', '/ax/robot_4/sub/joint_states',
                 '/ax/robot_5/joint_states']
        publishers = {'/ax/robot_1/joint_states': 1, '/ax/robot_2/joint_states': 2, '/ax/robot_5/joint_states': 0}

        self.assertEqual(CHECK.active_robots_from_topics(names, lambda name: publishers.get(name, 1)),
                         ['robot_1', 'robot_2'])

    # 월드 파일 경로·sha256·렌더러 환경·활성 로봇을 기록한다. 기존 필드 이름은 건드리지 않는다.
    def test_load_stage_conditions(self):
        with tempfile.TemporaryDirectory() as folder:
            world = Path(folder) / 'a.world'
            world.write_text('<sdf/>', encoding='utf-8')
            environ = {'LIBGL_ALWAYS_SOFTWARE': '1', 'GALLIUM_DRIVER': 'd3d12'}
            record = CHECK.load_stage_conditions(world, ['robot_2', 'robot_1'], environ)

        self.assertEqual(record['active_robot_ids'], ['robot_1', 'robot_2'])
        self.assertEqual(record['active_robot_count'], 2)
        self.assertIn('joint_states', record['active_robot_source'])
        self.assertEqual(record['world_sha256'], hashlib.sha256(b'<sdf/>').hexdigest())
        self.assertEqual(record['world_check'], {'method': '인자 기준'})
        self.assertEqual(record['renderer']['env'], {'LIBGL_ALWAYS_SOFTWARE': '1', 'GALLIUM_DRIVER': 'd3d12',
                                                     'MESA_D3D12_DEFAULT_ADAPTER_NAME': None})
        self.assertIn('측정 도구 프로세스', record['renderer']['source'])

    # --world-file이 없으면 경로·해시는 null이고 사유를 남긴다.
    def test_load_stage_without_world(self):
        record = CHECK.load_stage_conditions(None, [], {})

        self.assertIsNone(record['world_file'])
        self.assertIsNone(record['world_sha256'])
        self.assertEqual(record['world_reason'], '--world-file 미지정')
        self.assertEqual(record['active_robot_count'], 0)

    # 같은 GZ_PARTITION의 gz sim 명령줄 월드 파일을 찾아 해시를 비교한다(가짜 /proc).
    def test_gz_world_check(self):
        with tempfile.TemporaryDirectory() as folder:
            world = Path(folder) / 'w.world'
            world.write_text('<sdf/>', encoding='utf-8')
            proc = Path(folder) / 'proc'

            for pid, partition in (('10', 'mine'), ('11', 'other')):
                (proc / pid).mkdir(parents=True)
                (proc / pid / 'cmdline').write_bytes(b'\0'.join([b'gz', b'sim', b'-r', b'-s', str(world).encode()]))
                (proc / pid / 'environ').write_bytes(b'GZ_PARTITION=' + partition.encode() + b'\0')

            # ruby gz는 프로세스 제목을 공백 포함 한 문자열로 바꾼다.
            (proc / '12').mkdir()
            (proc / '12' / 'cmdline').write_bytes(f'gz sim -r -s --headless-rendering {world}'.encode() + b'\0')
            (proc / '12' / 'environ').write_bytes(b'GZ_PARTITION=title\0')
            digest = hashlib.sha256(b'<sdf/>').hexdigest()
            found = CHECK.gz_world_check(digest, 'mine', proc)
            missing = CHECK.gz_world_check(digest, 'none', proc)
            title = CHECK.gz_world_check(digest, 'title', proc)
            self.assertTrue(title['matches'])

        self.assertEqual(found['method'], 'gz sim 서버 명령줄')
        self.assertTrue(found['matches'])
        self.assertEqual(list(found['gz_world_files']), [str(world)])
        self.assertEqual(missing['method'], '인자 기준')
        self.assertIsNone(missing['matches'])

    # 결과 conditions.world_sha256을 지도 적용 월드 목록(applicable_worlds.yaml)과 대조한다.
    # 2026-11-14 사용자 결정: 목록의 단색(작성)·질감(기준) 월드 모두 대상으로 인정하고 실제 쓴 월드를 기록한다.
    # 목록에 없는 해시는 불일치다.
    def test_target_uses_result_world(self):
        worlds = yaml.safe_load((MAP_INFO_PATH.parent / 'applicable_worlds.yaml').read_text(encoding='utf-8'))
        sha = {entry['launch_name']: entry['sha256'] for entry in worlds['applicable_worlds']}
        cases = ((sha['planned/ax_warehouse.world'], 'planned/ax_warehouse.world'),
                 (sha['planned/ax_warehouse_plain.world'], 'planned/ax_warehouse_plain.world'),
                 ('e' * 64, None))

        for world_sha, used in cases:
            result = good_result()
            result['conditions'] = {'world_sha256': world_sha}
            context = CHECK.target_context(load_criteria(), CRITERIA_PATH, None, result)
            mismatches, unchecked = CHECK.check_target(load_criteria()['target'], context)
            judged = CHECK.judge_localization(result, load_criteria(), REF, context)

            self.assertEqual(context['recorded_world_sha256'], world_sha)
            self.assertEqual(judged['criteria_method']['target_world_used'], used)
            self.assertEqual(judged['criteria_method']['target_world_reference'], 'planned/ax_warehouse.world')

            if used is not None:
                self.assertEqual(mismatches, [])
            else:
                self.assertTrue(any('지도 적용 월드 목록' in text for text in mismatches), mismatches)


# 부하 단계 정보로 하는 묶음 판정(B2-L8 동시 회차, LD-2 부하 불변성)
class BatchLoadStage(unittest.TestCase):

    # 동시 회차는 활성 로봇 수로 센다. 측정 대상이 robot_2뿐이어도 활성 2대면 단독이 아니라 동시다.
    def test_simultaneous_by_active_count(self):
        runs = [staged_run('pair', ['robot_1', 'robot_2']), staged_run('solo', ['robot_2'])]
        judged = CHECK.judge_batch(runs, load_criteria(), REF)
        items = {item['metric']: item for item in judged['batch_items'] if item['id'] == 'B2-L8'}

        self.assertEqual(items['repeats.two_robot_simultaneous_runs_min']['runs'], ['pair'])
        self.assertEqual(items['repeats.two_robot_simultaneous_runs_min']['result'], 'PASS')
        self.assertEqual(items['repeats.robot_2_single_runs']['runs'], ['solo'])
        self.assertNotIn('활성 로봇 수 기록 없음', items['repeats.robot_2_single_runs']['reason'])

    # 옛 기록(활성 로봇 수 실측 없음)은 robot_ids 길이로 대신 세지 않는다(반복 근거에서 제외, 사유 기록).
    def test_old_records_not_counted(self):
        judged = CHECK.judge_batch([batch_run('old', good_result(), staged=False)], load_criteria(), REF)
        single = next(item for item in judged['batch_items'] if item['metric'] == 'repeats.robot_2_single_runs')

        self.assertEqual((single['value'], single['runs']), (0, []))
        self.assertIn('활성 로봇 수 실측 기록 없음', single['excluded'][0][1])
        self.assertEqual(judged['runs'][0]['active_robots_source'],
                         'active_robot_count 없음, 측정 대상 robot_ids 길이 사용')

    # LD-2: 같은 월드의 L0(1대) 평균 대비 다대 회차 주행 p95 증가가 +0.03 m 이하면 PASS, 경계 포함.
    def test_ld2_pass_and_boundary(self):
        runs = [staged_run('l0a', ['robot_2'], moving_error=0.0), staged_run('l0b', ['robot_2'], moving_error=0.0),
                staged_run('l1', ['robot_1', 'robot_2'], moving_error=0.03)]
        judged = CHECK.judge_batch(runs, load_criteria(), REF)
        ld2 = next(item for item in judged['batch_items'] if item['id'] == 'LD-2')

        self.assertEqual(ld2['value'], 0.03)
        self.assertEqual(ld2['result'], 'PASS')
        self.assertEqual(ld2['comparisons'][0]['l0_runs'], 2)
        self.assertTrue(next(entry for entry in judged['coverage'] if entry['id'] == 'LD-2')['satisfied'])

        # L0 평균(0.04·0.06 → 0.05) 기준으로 증가량을 계산한다.
        runs = [staged_run('l0a', ['robot_2'], moving_error=0.04), staged_run('l0b', ['robot_2'], moving_error=0.06),
                staged_run('l1', ['robot_1', 'robot_2'], moving_error=0.07)]
        ld2 = next(item for item in CHECK.judge_batch(runs, load_criteria(), REF)['batch_items']
                   if item['id'] == 'LD-2')
        self.assertAlmostEqual(ld2['comparisons'][0]['l0_mean_p95_m'], 0.05, places=9)
        self.assertAlmostEqual(ld2['value'], 0.02, places=9)
        self.assertEqual(ld2['result'], 'PASS')

    # LD-2 증가가 한계를 넘으면 묶음 FAIL이고 사유에 LD-2를 적는다.
    def test_ld2_fail_makes_batch_fail(self):
        runs = [staged_run('l0', ['robot_2'], moving_error=0.03),
                staged_run('l1', ['robot_1', 'robot_2'], moving_error=0.09)]
        judged = CHECK.judge_batch(runs, load_criteria(), REF)
        ld2 = next(item for item in judged['batch_items'] if item['id'] == 'LD-2')

        self.assertEqual(ld2['result'], 'FAIL')
        self.assertEqual(judged['judgment'], 'FAIL')
        self.assertIn('LD-2', judged['judgment_reason'])

    # 월드 sha256이 다른 회차끼리는 비교하지 않는다(짝 없음 → not_judged).
    def test_ld2_different_worlds_not_compared(self):
        runs = [staged_run('l0', ['robot_2'], world='a' * 64),
                staged_run('l1', ['robot_1', 'robot_2'], world='b' * 64)]
        ld2 = next(item for item in CHECK.judge_batch(runs, load_criteria(), REF)['batch_items']
                   if item['id'] == 'LD-2')

        self.assertEqual(ld2['result'], 'not_judged')
        self.assertIn('짝 없음', ld2['reason'])

    # 부하 단계 기록이 없는 옛 기록만 있으면 LD-2는 "부하 단계 정보 없음"이다.
    def test_ld2_without_stage_info(self):
        judged = CHECK.judge_batch([batch_run('old', good_result(), staged=False)], load_criteria(), REF)
        ld2 = next(item for item in judged['batch_items'] if item['id'] == 'LD-2')

        self.assertEqual(ld2['result'], 'not_judged')
        self.assertTrue(ld2['reason'].startswith('부하 단계 정보 없음'))


# 외부 검토 지적 3건(묶음 출력 덮어쓰기, 조건 파일 월드 해시 형식, 무효 회차 근거 제외)
class ReviewFixes(unittest.TestCase):

    # 묶음 CLI를 실행한다.
    def run_batch(self, inputs, output):
        return subprocess.run(
            [sys.executable, str(SCRIPTS / 'check_localization.py'), '--batch', *map(str, inputs),
             '--criteria', str(CRITERIA_PATH), '--output', str(output)],
            capture_output=True, text=True, timeout=60)

    # --output이 입력 결과와 같으면(경로 표기가 달라도 resolve 기준) 거부하고 원본은 그대로다.
    def test_batch_output_same_as_input(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'run' / 'localization_result.json'
            source.parent.mkdir()
            source.write_text(json.dumps(good_result(), ensure_ascii=False), encoding='utf-8')
            before = source.read_bytes()
            alias = Path(folder) / 'run' / '..' / 'run' / 'localization_result.json'
            completed = self.run_batch([source], alias)

            self.assertEqual(completed.returncode, 2, completed.stderr)
            self.assertIn('입력 결과와 같음', completed.stderr)
            self.assertEqual(source.read_bytes(), before)

    # --output이 이미 있는 파일이면 거부하고 그 파일도 바꾸지 않는다.
    def test_batch_output_exists(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'localization_result.json'
            source.write_text(json.dumps(good_result(), ensure_ascii=False), encoding='utf-8')
            existing = Path(folder) / 'batch.json'
            existing.write_text('keep', encoding='utf-8')
            completed = self.run_batch([source], existing)

            self.assertEqual(completed.returncode, 2, completed.stderr)
            self.assertIn('이미 있음', completed.stderr)
            self.assertEqual(existing.read_text(encoding='utf-8'), 'keep')

            with self.assertRaises(ValueError):
                CHECK.batch([source], CRITERIA_PATH, existing)

    # 조건 파일 files_sha256.world의 실제 형식({path, sha256})에서 sha256을 꺼내 대조한다.
    def test_world_hash_dict_format(self):
        worlds = yaml.safe_load((MAP_INFO_PATH.parent / 'applicable_worlds.yaml').read_text(encoding='utf-8'))
        texture = next(entry['sha256'] for entry in worlds['applicable_worlds']
                       if entry['launch_name'] == 'planned/ax_warehouse.world')
        recorded = matching_context()['recorded_map']

        for digest, mismatch in ((texture, False), ('d' * 64, True)):
            world = {'path': '/x/ax_warehouse.world', 'sha256': digest}
            conditions = {'map': recorded, 'files_sha256': {'world': world}}
            context = CHECK.target_context(load_criteria(), CRITERIA_PATH, conditions, good_result())
            mismatches, unchecked = CHECK.check_target(load_criteria()['target'], context)

            self.assertEqual(context['recorded_world_sha256'], digest)
            self.assertEqual(bool(mismatches), mismatch, mismatches)
            self.assertFalse(any('월드' in text for text in unchecked), unchecked)

        # 옛 형식(문자열)은 그대로, 알 수 없는 형식은 None과 사유
        self.assertEqual(CHECK.world_hash(texture), (texture, None))
        self.assertEqual(CHECK.world_hash({'path': 'x'})[0], None)
        self.assertIsNone(CHECK.world_hash(42)[0])
        conditions = {'map': recorded, 'files_sha256': {'world': 42}}
        context = CHECK.target_context(load_criteria(), CRITERIA_PATH, conditions, good_result())
        self.assertIn('월드 해시 형식을 알 수 없음', ' '.join(CHECK.check_target(load_criteria()['target'], context)[1]))

    # incomplete 회차만 있으면 B2-L1·L2·L9도 충족 근거가 없다.
    def test_invalid_runs_not_evidence(self):
        bad = good_result()
        bad['status'] = 'incomplete'
        judged = CHECK.judge_batch([batch_run('bad', bad)], load_criteria(), REF)
        coverage = {entry['id']: entry for entry in judged['coverage']}

        for item_id in ('B2-L1', 'B2-L2', 'B2-L9'):
            self.assertFalse(coverage[item_id]['satisfied'], item_id)

        self.assertFalse(judged['runs'][0]['valid'])

    # 유효 회차와 무효 회차(incomplete·SAMPLE_GAP·대상 불일치)가 섞이면 유효 회차만 근거다.
    def test_mixed_runs_only_valid_evidence(self):
        incomplete = good_result()
        incomplete['status'] = 'incomplete'
        gap_criteria = load_criteria()
        gap_criteria['load']['max_sample_interval_s'] = 0.05
        gap = good_result()
        gap['robots']['robot_2']['odom']['max_gap_s'] = 0.2
        context = matching_context()
        context['recorded_map'] = dict(context['recorded_map'], yaml='f' * 64)
        mismatch_result = good_result()
        mismatch_result['conditions'] = {'robot_ids': ['robot_2']}
        mismatch_judgment = CHECK.judge_localization(mismatch_result, load_criteria(), REF, context)
        mismatch = ('mismatch', mismatch_result, mismatch_judgment, '0' * 64, None)
        runs = [batch_run('ok', good_result()), batch_run('incomplete', incomplete),
                batch_run('gap', gap, criteria=gap_criteria), mismatch]
        judged = CHECK.judge_batch(runs, load_criteria(), REF)
        coverage = {entry['id']: entry for entry in judged['coverage']}

        self.assertEqual([run['valid'] for run in judged['runs']], [True, False, False, False])

        for item_id in ('B2-L1', 'B2-L2', 'B2-L9'):
            self.assertEqual(coverage[item_id]['runs'], ['ok'], item_id)

        single = next(item for item in judged['batch_items'] if item['metric'] == 'repeats.robot_2_single_runs')
        self.assertEqual(single['runs'], ['ok'])


# 월드 정책 회귀(2026-11-14 사용자 결정): 같은 revision의 applicable_worlds에 등록된 단색·질감 해시는 둘 다 인정,
# 미등록 해시와 revision 불일치는 거절. 조건 파일 실제 형식({path, sha256})으로 확인하고 실제 출처를 기록한다.
class WorldPolicy(unittest.TestCase):

    # 지도 적용 월드 목록을 읽는다.
    def worlds(self):
        return yaml.safe_load((MAP_INFO_PATH.parent / 'applicable_worlds.yaml').read_text(encoding='utf-8'))

    # 조건 파일 형식으로 대상 대조와 회차 판정을 만든다.
    def judge(self, digest, criteria=None):
        criteria = criteria or load_criteria()
        conditions = {'map': matching_context()['recorded_map'],
                      'files_sha256': {'world': {'path': '/x/world.world', 'sha256': digest}}}
        context = CHECK.target_context(criteria, CRITERIA_PATH, conditions, good_result())

        return CHECK.judge_localization(good_result(), criteria, REF, context)

    # 단색(작성)·질감(기준) 두 등록 월드 모두 인정하고 실제 출처를 남긴다.
    def test_both_registered_worlds_accepted(self):
        for entry in self.worlds()['applicable_worlds']:
            judged = self.judge(entry['sha256'])
            method = judged['criteria_method']

            self.assertEqual(method['target_mismatch'], [], entry['launch_name'])
            self.assertEqual(method['target_world_used'], entry['launch_name'])
            self.assertEqual(method['target_world_source']['registered']['sha256'], entry['sha256'])
            self.assertEqual(method['target_world_source']['registered']['basis'], entry['basis'])
            self.assertEqual(method['target_world_source']['applicable_revision'], 1)
            self.assertEqual(method['target_world_source']['authored_world'], 'planned/ax_warehouse_plain.world')
            self.assertEqual(judged['judged_subset'], 'PASS')

        self.assertEqual(len(self.worlds()['applicable_worlds']), 2)

    # 미등록 해시는 대상 불일치로 거절한다.
    def test_unregistered_world_rejected(self):
        judged = self.judge('c' * 64)

        self.assertTrue(judged['judgment_reason'].startswith('대상 불일치'))
        self.assertIsNone(judged['criteria_method']['target_world_source']['registered'])

    # 기준 revision이 지도 적용 목록 revision과 다르면 등록 해시여도 거절한다.
    def test_revision_mismatch_rejected(self):
        criteria = load_criteria()
        criteria['target']['map_revision'] = 2
        texture = next(entry['sha256'] for entry in self.worlds()['applicable_worlds']
                       if entry['launch_name'] == 'planned/ax_warehouse.world')
        judged = self.judge(texture, criteria)

        self.assertTrue(judged['judgment_reason'].startswith('대상 불일치'))
        self.assertTrue(any('applicable_worlds revision 1 ≠ 기준 2' in text
                            for text in judged['criteria_method']['target_mismatch']))


# health 원시 기록 한 줄(sim stamp, 상태, 유효성, 세션)
def hrow(stamp, state='OK', valid=True, session='s1'):
    return {'stamp': stamp, 'state': state, 'valid': valid, 'session': session, 'reason': ''}


# odom 원시 기록 한 줄(sim stamp, 선속도, 각속도)
def orow(stamp, v=0.0, w=0.0):
    return {'stamp': stamp, 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'v': v, 'w': w}


# start~end(포함)를 step 간격으로 나눈 시각 목록(0.25 s처럼 이진수로 정확한 간격을 쓴다)
def times(start, end, step=0.25):
    count = int(round((end - start) / step))

    return [start + step * index for index in range(count + 1)]


# 0~10 s 주행(0.3 m/s) 뒤 10~20 s 정지하는 odom
def drive_then_stop():
    return [orow(t, 0.3 if t < 10.0 else 0.0) for t in times(0.0, 20.0)]


# 시각마다 상태를 정하는 함수로 health 기록을 만든다.
def health_by(state_of, start=0.0, end=20.0, step=0.25):
    return [hrow(t, *state_of(t)) for t in times(start, end, step)]


# 회차 결과와 원시 근거로 B2-L3 항목(로봇별)을 꺼낸다.
def l3_items(evidence, result=None):
    judged = CHECK.judge_localization(result or good_result(), load_criteria(), REF, matching_context(), evidence)

    return [item for item in judged['criteria_items'] if item['id'] == 'B2-L3'], judged


# 로봇별 health_ok 기록으로 원시 근거 dict를 만든다(LD-3은 확인된 무발생).
def evidence_of(records):
    return {'source': {'how': '시험'}, 'robots': {robot: {'health_ok': record} for robot, record in records.items()},
            'wall_clock': CHECK.wall_clock_summary(CHECK.collector_wall_status({'recording': {'duration_met': True}}),
                                                   {robot: CHECK.supervisor_wall_status(ticks(), [100.0, 200.0])
                                                    for robot in records})}


# 감독 진단 tick 한 줄
def tick(mono, failure=None, sim=None, session='s1'):
    return {'kind': 'tick', 'sim': mono if sim is None else sim, 'mono': mono, 'reinit_failure': failure,
            'session': session}


# start~end를 0.5 s 간격으로 덮는 tick(정상 연속)
def ticks(start=100.0, end=200.0):
    return [tick(mono) for mono in times(start, end, 0.5)]


# B2-L3 PASS(또는 정지 전용 not_applicable)·LD-3 무발생 확인 근거(다른 항목 시험용)
# B2-L4~L7은 합성 사건 기록(b2_event_fixture 기본 시나리오, 모든 사건 시험 PASS)으로 채운다.
def full_evidence(stationary=False):
    odom = [orow(t) for t in times(0.0, 20.0)] if stationary else drive_then_stop()
    evidence = evidence_of({'robot_2': CHECK.health_ok_ratio(health_by(lambda t: ('OK', True)), odom)})
    evidence['robots']['robot_2']['events'] = EVENTS.evidence(CHECK, load_criteria())

    return evidence


# B2-L3: 정상 주행 중 health OK 시간 비율(sim time)
class HealthOkRatio(unittest.TestCase):

    # 주행 10 s 중 OK 9 s → 0.90(경계 포함) PASS, 8.75 s → 0.875 FAIL. 정지 구간의 OK는 분모·분자에 넣지 않는다.
    def test_boundary(self):
        for degraded_from, ratio, expected in ((9.0, 0.9, 'PASS'), (8.75, 0.875, 'FAIL')):
            with self.subTest(degraded_from=degraded_from):
                health = health_by(lambda t: ('DEGRADED', True) if degraded_from <= t < 10.0 else ('OK', True))
                record = CHECK.health_ok_ratio(health, drive_then_stop())
                items, _ = l3_items(evidence_of({'robot_2': record}))

                self.assertEqual(record['status'], 'measured')
                self.assertEqual(record['moving_s'], 10.0)
                self.assertEqual(record['ratio'], ratio)
                self.assertEqual(record['unknown_s'], 0.0)
                self.assertEqual([item['result'] for item in items], [expected])
                self.assertEqual(items[0]['value'], ratio)

    # 경계 바로 아래 값은 FAIL이다(비교 함수 직접).
    def test_just_below(self):
        record = {'status': 'measured', 'ratio': 0.9 - 1e-12, 'moving_s': 10.0, 'ok_s': 9.0 - 1e-11}
        item = CHECK.health_ratio_item('robot_2', record, 0.9)

        self.assertEqual(item['result'], 'FAIL')

    # 정지 구간 OK가 길어도 주행 비율은 같다(회차 전체 비율로 대신하지 않음).
    def test_stationary_ok_not_counted(self):
        health = health_by(lambda t: ('LOST', True) if 5.0 <= t < 10.0 else ('OK', True), end=100.0)
        odom = [orow(t, 0.3 if t < 10.0 else 0.0) for t in times(0.0, 100.0)]
        record = CHECK.health_ok_ratio(health, odom)

        self.assertEqual(record['ratio'], 0.5)
        self.assertEqual(record['moving_s'], 10.0)

    # 불규칙 재발행·중복 메시지는 시간 비율을 바꾸지 않는다(개수 비율이 아님).
    def test_irregular_and_duplicates(self):
        regular = health_by(lambda t: ('LOST', True) if 5.0 <= t < 10.0 else ('OK', True))
        irregular = []

        for t in times(0.0, 20.0, 0.125):
            state = ('LOST', True) if 5.0 <= t < 10.0 else ('OK', True)
            # OK 구간에는 메시지를 몰아 보내고 같은 메시지를 중복한다(개수로 세면 OK가 많아진다).
            copies = 3 if state[0] == 'OK' else 1

            if state[0] == 'OK' or t % 0.5 == 0:
                irregular += [hrow(t, *state)] * copies

        first = CHECK.health_ok_ratio(regular, drive_then_stop())
        second = CHECK.health_ok_ratio(irregular, drive_then_stop())

        self.assertEqual(first['ratio'], 0.5)
        self.assertEqual(second['ratio'], 0.5)
        self.assertGreater(second['health']['duplicates'], 0)

    # 양 로봇 자료는 따로 계산하고 로봇별 항목으로 남긴다.
    def test_two_robots(self):
        good = CHECK.health_ok_ratio(health_by(lambda t: ('OK', True)), drive_then_stop())
        bad = CHECK.health_ok_ratio(health_by(lambda t: ('LOST', True) if t < 5.0 else ('OK', True)), drive_then_stop())
        result = good_result()
        result['robots']['robot_1'] = copy.deepcopy(result['robots']['robot_2'])
        items, _ = l3_items(evidence_of({'robot_1': good, 'robot_2': bad}), result)

        self.assertEqual({item['robot']: (item['value'], item['result']) for item in items},
                         {'robot_1': (1.0, 'PASS'), 'robot_2': (0.5, 'FAIL')})

    # valid=false인 OK는 분자에 넣지 않는다. UNKNOWN·DEGRADED·LOST도 같다.
    def test_invalid_ok_not_counted(self):
        for state in (('OK', False), ('UNKNOWN', False), ('DEGRADED', True), ('LOST', True)):
            with self.subTest(state=state):
                health = health_by(lambda t: state if t < 5.0 else ('OK', True))

                self.assertEqual(CHECK.health_ok_ratio(health, drive_then_stop())['ratio'], 0.5)

    # health 공백(1.0 s 초과)·미수신·비유한·시각 역행·세션 변경 공백은 PASS가 되지 않는다(not_judged, 사유 있음).
    def test_unverifiable_health(self):
        ok = lambda t: ('OK', True)  # noqa: E731
        gap = [row_ for row_ in health_by(ok) if not 3.0 < row_['stamp'] < 6.0]
        nonfinite = health_by(ok)
        nonfinite[10]['stamp'] = math.nan
        reversed_ = health_by(ok)
        reversed_[20]['stamp'] = 1.0
        session = [hrow(t, session='s1') for t in times(0.0, 5.0)] + [hrow(t, session='s2') for t in times(6.5, 20.0)]
        cases = {'공백': gap, '미수신': [], '비유한': nonfinite, '역행': reversed_, '세션 변경': session}

        for name, health in cases.items():
            with self.subTest(case=name):
                record = CHECK.health_ok_ratio(health, drive_then_stop())
                items, judged = l3_items(evidence_of({'robot_2': record}))

                self.assertEqual(record['status'], 'not_judged')
                self.assertTrue(record['reason'])
                self.assertEqual(items[0]['result'], 'not_judged')
                self.assertNotEqual(judged['judgment'], 'PASS')

        # 확인 불가 시간을 분모에서 빼지 않는다: 공백 3~6 s 중 1 s 유지 뒤 2 s가 unknown
        record = CHECK.health_ok_ratio(gap, drive_then_stop())
        self.assertEqual(record['unknown_s'], 2.0)
        self.assertEqual(record['moving_s'], 10.0)
        self.assertEqual(CHECK.health_ok_ratio(session, drive_then_stop())['health']['session_breaks'], 1)

    # 세션이 바뀌어도 시각이 이어지고 공백이 없으면 판정한다(상태는 세션을 넘어 이어 붙이지 않음).
    def test_session_change_without_gap(self):
        health = [hrow(t, session='s1') for t in times(0.0, 5.0)] + [hrow(t, session='s2') for t in times(5.25, 20.0)]
        record = CHECK.health_ok_ratio(health, drive_then_stop())

        self.assertEqual((record['status'], record['ratio']), ('measured', 1.0))

    # 정지만 한 회차는 not_applicable, odom이 없으면(주행 여부 확인 불가) not_judged
    def test_stationary_only_and_no_odom(self):
        still = CHECK.health_ok_ratio(health_by(lambda t: ('OK', True)), [orow(t) for t in times(0.0, 20.0)])
        missing = CHECK.health_ok_ratio(health_by(lambda t: ('OK', True)), [])
        odom_gap = CHECK.health_ok_ratio(health_by(lambda t: ('OK', True)),
                                         [row_ for row_ in drive_then_stop() if not 2.0 < row_['stamp'] < 3.0])

        self.assertEqual(still['status'], 'not_applicable')
        self.assertEqual(l3_items(evidence_of({'robot_2': still}))[0][0]['result'], 'not_applicable')
        self.assertEqual(missing['status'], 'not_judged')
        self.assertEqual(odom_gap['status'], 'not_judged')
        self.assertIn('odom', odom_gap['reason'])

    # 명시된 오류 주입 구간만 분모에서 빼고 근거를 남긴다. 표시가 없으면 관측 주행 전체를 쓴다.
    def test_explicit_exclusion(self):
        health = health_by(lambda t: ('LOST', True) if 2.0 <= t < 4.0 else ('OK', True))
        marked = [{'start_sim': 2.0, 'end_sim': 4.0, 'reason': 'AMCL 재시작 주입', 'source': 'scenario/events.jsonl'}]
        plain = CHECK.health_ok_ratio(health, drive_then_stop())
        excluded = CHECK.health_ok_ratio(health, drive_then_stop(), marked)

        self.assertEqual(plain['ratio'], 0.8)
        self.assertEqual(plain['excluded'], [])
        self.assertIn('관측된 주행 전체', plain['scope'])
        self.assertEqual((excluded['ratio'], excluded['moving_s'], excluded['excluded_moving_s']), (1.0, 8.0, 2.0))
        self.assertEqual(excluded['excluded'][0]['reason'], 'AMCL 재시작 주입')

        # 근거(사유·출처)가 없는 표시는 받아들이지 않는다(낮은 health만 보고 사후에 빼지 않음).
        broken = CHECK.health_ok_ratio(health, drive_then_stop(), [{'start_sim': 2.0, 'end_sim': 4.0}])
        self.assertEqual(broken['status'], 'not_judged')

    # 원시 근거가 없으면(과거 결과만) B2-L3은 not_judged다.
    def test_without_raw(self):
        items, _ = l3_items(None)

        self.assertEqual([item['result'] for item in items], ['not_judged'])
        self.assertIn('원시', items[0]['reason'])


# 원시 기록 폴더(조건 파일·health·odom·감독 진단)와 결과 파일을 만든다. 반환: 결과 파일 경로
def write_run(folder, health, odom, supervisor=None, result=None, exclusions=None, robot='robot_2',
              mono=(100.0, 200.0)):
    run = Path(folder)
    (run / 'raw' / robot).mkdir(parents=True, exist_ok=True)
    (run / 'result').mkdir(exist_ok=True)
    context = matching_context()
    conditions = {'robot_ids': [robot], 'time_reference': {'monotonic_s': mono[0]},
                  'map': {'yaml': context['recorded_map']['yaml'], 'image': context['recorded_map']['image']},
                  'files_sha256': {}}
    (run / 'conditions.json').write_text(json.dumps(conditions), encoding='utf-8')

    for stream, rows in (('health', health), ('odom', odom)):
        (run / 'raw' / robot / f'{stream}.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows),
                                                             encoding='utf-8')

    if supervisor is not None:
        (run / 'supervisor').mkdir(exist_ok=True)
        (run / 'supervisor' / f'{robot}_supervisor.jsonl').write_text(
            ''.join(json.dumps(r) + '\n' for r in supervisor), encoding='utf-8')

    if exclusions is not None:
        (run / 'health_exclusions.json').write_text(json.dumps(exclusions), encoding='utf-8')

    result = copy.deepcopy(result or good_result())
    result.setdefault('conditions', {})['robot_ids'] = [robot]
    result['recording'] = {'duration_met': True,
                           'termination': CHECK.collector_termination(0.0, 20.0, 20.0, mono[0], mono[1], 900.0,
                                                                      False, False)}
    result['raw'] = {'dir': str(run)}
    path = run / 'result' / 'localization_result.json'
    path.write_text(json.dumps(result, ensure_ascii=False), encoding='utf-8')

    return path


# 실시간 경로와 재판정 경로가 같은 계산을 쓰는지, 원본을 바꾸지 않는지
class RawEvidencePaths(unittest.TestCase):

    # 같은 원시 자료로 실시간(측정 직후) 판정과 --rejudge 판정의 B2-L3·LD-3 항목이 같다.
    def test_live_equals_rejudge(self):
        with tempfile.TemporaryDirectory() as folder:
            health = health_by(lambda t: ('DEGRADED', True) if t < 0.5 else ('OK', True))
            path = write_run(folder, health, drive_then_stop(), [tick(m) for m in times(101.0, 200.0, 0.5)])
            result = json.loads(path.read_text(encoding='utf-8'))
            criteria, reference = CHECK.load_criteria(CRITERIA_PATH)
            conditions = json.loads((Path(folder) / 'conditions.json').read_text(encoding='utf-8'))
            live = CHECK.criteria_judgment(result, criteria, reference, CRITERIA_PATH, conditions, Path(folder),
                                           {'how': '측정 중 --raw-dir'})
            before = path.read_bytes()
            output, judged = CHECK.rejudge(path, CRITERIA_PATH)

            pick = lambda data: [item for item in data['criteria_items'] if item['id'] in ('B2-L3', 'LD-3')]  # noqa
            self.assertEqual(pick(live), pick(judged))
            self.assertEqual([item['result'] for item in pick(live)], ['PASS', 'PASS'])
            self.assertEqual(find(pick(live), 'B2-L3', 'moving.min_health_ok_ratio')['value'], 0.95)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(judged['criteria_method']['raw_evidence_source']['how'], '결과 옆 회차 폴더')
            self.assertIn('sha256', judged['criteria_method']['raw_evidence_source']['files']['robot_2']['health'])

    # 명시한 원시 폴더(--raw-dir)를 우선하고 출처를 남긴다. 기록된 raw.dir이 없어도 읽는다.
    def test_explicit_raw_dir(self):
        with tempfile.TemporaryDirectory() as folder:
            source = write_run(Path(folder) / 'raw_run', health_by(lambda t: ('OK', True)), drive_then_stop(),
                               [tick(m) for m in times(101.0, 200.0, 0.5)])
            lonely = Path(folder) / 'elsewhere' / 'localization_result.json'
            lonely.parent.mkdir()
            data = json.loads(source.read_text(encoding='utf-8'))
            data['raw'] = {'dir': '/없는/폴더'}
            lonely.write_text(json.dumps(data), encoding='utf-8')
            _, without = CHECK.rejudge(lonely, CRITERIA_PATH)
            Path(lonely.with_name('localization_result.judged.json')).unlink()
            _, explicit = CHECK.rejudge(lonely, CRITERIA_PATH, raw_dir=Path(folder) / 'raw_run')

            self.assertEqual(find(without['criteria_items'], 'B2-L3', 'moving.min_health_ok_ratio')['result'],
                             'not_judged')
            self.assertEqual(find(explicit['criteria_items'], 'B2-L3', 'moving.min_health_ok_ratio')['result'],
                             'PASS')
            self.assertEqual(explicit['criteria_method']['raw_evidence_source']['how'], '--raw-dir 지정')

    # 원시 폴더의 health_exclusions.json(명시 표시)을 읽어 구간을 뺀다.
    def test_exclusion_file(self):
        with tempfile.TemporaryDirectory() as folder:
            health = health_by(lambda t: ('LOST', True) if 2.0 <= t < 4.0 else ('OK', True))
            marks = {'schema': 'ax_b2_health_exclusions/1', 'intervals': [
                {'robot_id': 'robot_2', 'start_sim': 2.0, 'end_sim': 4.0, 'reason': '주입', 'source': '시나리오'}]}
            path = write_run(folder, health, drive_then_stop(), [tick(m) for m in times(101.0, 200.0, 0.5)],
                             exclusions=marks)
            _, judged = CHECK.rejudge(path, CRITERIA_PATH)
            item = find(judged['criteria_items'], 'B2-L3', 'moving.min_health_ok_ratio')

            self.assertEqual((item['value'], item['result']), (1.0, 'PASS'))
            self.assertEqual(item['health_ok']['excluded'][0]['source'], '시나리오')


# LD-3: 벽시계 제한 회차
class WallClockLimit(unittest.TestCase):

    # 수집기: 요청 sim 시간 충족·종료 파일·중단 신호는 벽시계 제한이 아니다. 상한 도달로 기록이 잘리면 제한이다.
    def test_collector_causes(self):
        cases = {
            '기록 충족': ((0.0, 30.0, 30.0, 0.0, 40.0, 900.0, False, False), False),
            '종료 파일': ((0.0, 10.0, 30.0, 0.0, 40.0, 900.0, True, False), False),
            'Ctrl+C': ((0.0, 10.0, 30.0, 0.0, 40.0, 900.0, False, True), False),
            '벽시계 상한': ((0.0, 10.0, 30.0, 0.0, 900.0, 900.0, False, False), True),
            'sim 정체': ((5.0, 5.0, 30.0, 0.0, 901.0, 900.0, False, False), True),
            '경계 직전': ((0.0, 10.0, 30.0, 0.0, 899.999, 900.0, False, False), False),
        }

        for name, (arguments, limited) in cases.items():
            with self.subTest(case=name):
                record = CHECK.collector_termination(*arguments)

                self.assertEqual(record['wall_limited'], limited)
                self.assertEqual(record['wall_limit_s'], 900.0)
                status = CHECK.collector_wall_status({'recording': {'termination': record}})
                self.assertEqual((status['limited'], status['level']), (limited, 'confirmed'))

        both = CHECK.collector_termination(0.0, 30.0, 30.0, 0.0, 900.0, 900.0, False, False)
        self.assertEqual((sorted(both['causes']), both['wall_limited']), (['SIM_DURATION', 'WALL_LIMIT'], False))

    # 옛 기록: 기록 시간을 채웠으면 수집기 상한이 자르지 않은 것이 확인되고, 부족하면 원인 미확인이다.
    def test_collector_old_records(self):
        self.assertFalse(CHECK.collector_wall_status({'recording': {'duration_met': True}})['limited'])
        self.assertIsNone(CHECK.collector_wall_status({'recording': {'duration_met': False}})['limited'])
        self.assertIsNone(CHECK.collector_wall_status({})['limited'])

    # 감독 진단: 재위치·의도 벽시계 시간초과는 제한, sim만은 아니다. 세부 없는 옛 TIMEOUT·AMCL_UNAVAILABLE은 미확인
    def test_supervisor_events(self):
        wall = {'sim_expired': False, 'wall_expired': True, 'wall_limit_s': 60.0, 'wall_elapsed_s': 61.0}
        sim = {'sim_expired': True, 'wall_expired': False, 'wall_limit_s': 60.0, 'wall_elapsed_s': 16.0}
        finished = lambda timeout, mono=150.0: {'kind': 'reinit_finished', 'mono': mono, 'sim': 50.0,  # noqa
                                                'result': 'REINIT_FAILED_AMCL_UNAVAILABLE', 'timeout': timeout}
        closed = lambda reason, timeout=None: {'kind': 'tick', 'mono': 150.0, 'sim': 50.0, 'estimate_events': [  # noqa
            {'event': 'INTENT_CLOSED', 'reason': reason, **({'timeout': timeout} if timeout else {})}]}
        base = [tick(m) for m in times(101.0, 200.0, 0.5)]
        cases = {
            '재위치 벽시계': ([finished(wall)], True),
            '재위치 sim만': ([finished(sim)], False),
            '의도 TIMEOUT_WALL': ([closed('TIMEOUT_WALL')], True),
            '의도 sim·벽시계 동시': ([closed('TIMEOUT', dict(sim, wall_expired=True))], True),
            '의도 sim만': ([closed('TIMEOUT', sim)], False),
            '옛 의도 TIMEOUT': ([closed('TIMEOUT')], None),
            '옛 AMCL_UNAVAILABLE': ([tick(150.0, 'REINIT_FAILED_AMCL_UNAVAILABLE')], None),
            '세부 있는 AMCL_UNAVAILABLE': ([tick(150.0, 'REINIT_FAILED_AMCL_UNAVAILABLE'), finished(sim)], False),
            '수집 구간 밖': ([finished(wall, mono=50.0)], False),
        }

        for name, (extra, limited) in cases.items():
            with self.subTest(case=name):
                # 진단 파일은 시각 순서로 쓰이므로 사건 줄을 mono 순서 위치에 넣는다(안정 정렬).
                status = CHECK.supervisor_wall_status(sorted(base + extra, key=lambda r: r['mono']), [100.0, 200.0])

                self.assertIs(status['limited'], limited)

        self.assertIsNone(CHECK.supervisor_wall_status(None, [100.0, 200.0])['limited'])
        # 진단이 수집 끝까지 이어지지 않으면(중간에 꺼졌을 수 있음) 무발생을 확인하지 못한다.
        self.assertIsNone(CHECK.supervisor_wall_status([tick(101.0), tick(120.0)], [100.0, 200.0])['limited'])

    # 회차 판정: 확인된 제한이 있으면 LD-3 FAIL 항목, 회차 전체는 not_judged(incomplete 취급)·묶음 근거 무효
    def test_run_level(self):
        wall = {'sim_expired': False, 'wall_expired': True, 'wall_limit_s': 60.0}
        limited = {'kind': 'reinit_finished', 'mono': 150.0, 'sim': 50.0, 'result': 'REINIT_FAILED_AMCL_UNAVAILABLE',
                   'timeout': wall}
        rows = [tick(m) for m in times(101.0, 200.0, 0.5)]
        evidence = evidence_of({'robot_2': CHECK.health_ok_ratio(health_by(lambda t: ('OK', True)),
                                                                 drive_then_stop())})
        evidence['wall_clock'] = CHECK.wall_clock_summary(
            CHECK.collector_wall_status({'recording': {'duration_met': True}}),
            {'robot_2': CHECK.supervisor_wall_status(sorted(rows + [limited], key=lambda r: r['mono']),
                                                     [100.0, 200.0])})
        judged = CHECK.judge_localization(good_result(), load_criteria(), REF, matching_context(), evidence)
        item = find(judged['criteria_items'], 'LD-3', 'load.max_wall_clock_limited_runs')

        self.assertEqual((item['value'], item['result']), (1, 'FAIL'))
        self.assertEqual(judged['judgment'], 'not_judged')
        self.assertIn('벽시계 제한', judged['judgment_reason'])
        self.assertFalse(CHECK.run_valid('completed', judged))


# 묶음 LD-3용 회차(원시 폴더 키, 벽시계 제한 여부: True·False·None, status)
def wall_run(name, limited, raw_dir=None, status='completed', digest=None):
    collector = {'recording': {'duration_met': limited is not True}}
    term = CHECK.collector_termination(0.0, 30.0 if limited is not True else 10.0, 30.0, 0.0,
                                       900.0 if limited is True else 40.0, 900.0, False, False)
    wall = CHECK.wall_clock_summary(
        CHECK.collector_wall_status({'recording': {'termination': term}} if limited is not None else collector),
        {'robot_2': CHECK.supervisor_wall_status(ticks() if limited is not None else None, [100.0, 200.0])})
    evidence = {'source': {'how': '시험', 'raw_dir': raw_dir or f'/runs/{name}'}, 'robots': {}, 'wall_clock': wall}
    result = good_result()
    result['status'] = status if limited is not True else 'incomplete'
    result['conditions'] = {'robot_ids': ['robot_2']}
    judgment = CHECK.judge_localization(result, load_criteria(), REF, matching_context(), evidence)

    return (name, result, judgment, digest or hashlib.sha256(name.encode()).hexdigest(), None)


# LD-3 묶음 집계: 고유 회차 수, 확인 수준, FAIL 보존
class WallClockBatch(unittest.TestCase):

    # LD-3 묶음 항목
    def item(self, runs):
        judged = CHECK.judge_batch(runs, load_criteria(), REF)

        return find(judged['batch_items'], 'LD-3', 'load.max_wall_clock_limited_runs'), judged

    # 모든 회차에서 무발생이 확인되면 0건 PASS
    def test_confirmed_none(self):
        item, _ = self.item([wall_run('a', False), wall_run('b', False)])

        self.assertEqual((item['value'], item['result']), (0, 'PASS'))

    # 벽시계 제한으로 incomplete인 회차 하나 → LD-3 FAIL·묶음 FAIL(incomplete 회차도 집계에서 빼지 않음)
    def test_limited_run_fails_batch(self):
        item, judged = self.item([wall_run('a', False), wall_run('cut', True)])

        self.assertEqual((item['value'], item['result']), (1, 'FAIL'))
        self.assertEqual(judged['judgment'], 'FAIL')
        self.assertEqual(item['limited_runs'], ['cut'])
        self.assertEqual(judged['runs'][1]['status'], 'incomplete')
        self.assertFalse(judged['runs'][1]['valid'])

    # 같은 기록의 중복 제출(같은 원시 폴더·같은 sha256)은 회차를 늘리지 않는다.
    def test_duplicates_count_once(self):
        same_dir = [wall_run('cut', True, raw_dir='/runs/x'), wall_run('cut_judged', True, raw_dir='/runs/x')]
        same_hash = [wall_run('p1', True, raw_dir=None, digest='a' * 64),
                     wall_run('p2', True, raw_dir=None, digest='a' * 64)]

        for name, runs in (('같은 폴더', same_dir), ('같은 해시', same_hash)):
            with self.subTest(case=name):
                if name == '같은 해시':
                    for run in runs:
                        run[2]['criteria_method']['raw_evidence_source']['raw_dir'] = None

                item, _ = self.item(runs)
                self.assertEqual(item['value'], 1)

    # 옛 기록만 있으면 0으로 메우지 않고 not_judged와 확인 불가 회차 목록을 낸다.
    def test_unverified_not_judged(self):
        item, _ = self.item([wall_run('a', False), wall_run('old', None)])

        self.assertEqual(item['result'], 'not_judged')
        self.assertEqual(item['unverified_runs'], ['old'])

    # 확인된 제한과 미확인 회차가 섞여도 FAIL을 숨기지 않는다.
    def test_fail_with_unverified(self):
        item, judged = self.item([wall_run('old', None), wall_run('cut', True)])

        self.assertEqual(item['result'], 'FAIL')
        self.assertEqual(item['unverified_runs'], ['old'])
        self.assertEqual(judged['judgment'], 'FAIL')


# LD-3 감독 진단 연속성(검토 P2): 시작·중간·끝 tick 연속성과 단조 시각이 확인돼야 무발생을 확정한다.
class SupervisorCoverage(unittest.TestCase):

    # 수집 구간 [0, 100]의 상태
    def status(self, rows, health=None, window=(0.0, 100.0)):
        return CHECK.supervisor_wall_status(rows, list(window) if window else None, health_rows=health)

    # 정상 연속 tick이면 무발생 확인
    def test_continuous(self):
        result = self.status(ticks(0.0, 100.0))

        self.assertEqual((result['limited'], result['level'], result['reason']), (False, 'confirmed', ''))

    # 끝 tick 하나·처음과 끝 두 개뿐이면 확인 불가(검토 재현)
    def test_start_missing(self):
        for rows in ([tick(100.0)], [tick(0.0), tick(100.0)]):
            with self.subTest(count=len(rows)):
                result = self.status(rows)

                self.assertIsNone(result['limited'])
                self.assertTrue(result['reason'])

    # 중간 큰 공백(허용 1.0 s 초과)은 확인 불가
    def test_internal_gap(self):
        result = self.status(ticks(0.0, 40.0) + ticks(60.0, 100.0))

        self.assertIsNone(result['limited'])
        self.assertIn('공백', result['reason'])

    # 수집 끝 뒤의 tick이 끝을 대신 덮지 않는다.
    def test_outside_tick_does_not_cover_end(self):
        result = self.status(ticks(0.0, 50.0) + [tick(300.0)])

        self.assertIsNone(result['limited'])

    # 단조 시각 역행·비유한 tick은 확인 불가
    def test_monotonic_validity(self):
        backward = ticks(0.0, 100.0)
        backward[50]['mono'] = 1.0
        nonfinite = ticks(0.0, 100.0)
        nonfinite[50]['mono'] = math.nan

        for name, rows in (('역행', backward), ('비유한', nonfinite)):
            with self.subTest(case=name):
                self.assertIsNone(self.status(rows)['limited'])

    # 수집 구간을 모르면 연속성을 확인할 수 없다.
    def test_window_unknown(self):
        self.assertIsNone(self.status(ticks(0.0, 100.0), window=None)['limited'])

    # 확인된 벽시계 시간초과는 공백이 있어도 FAIL 근거로 유지한다.
    def test_confirmed_failure_with_gap(self):
        event = {'kind': 'reinit_finished', 'mono': 30.0, 'sim': 30.0, 'result': 'REINIT_FAILED_AMCL_UNAVAILABLE',
                 'timeout': {'sim_expired': False, 'wall_expired': True}}
        result = self.status(ticks(0.0, 40.0) + [event] + ticks(60.0, 100.0))

        self.assertIs(result['limited'], True)
        self.assertIn('공백', result['reason'])

    # 감독 노드가 수집 시작 뒤에 켜졌으면 health 기록으로 그 전 미발행을 증명한다: 첫 health 수신이 첫 tick 이후이고
    # health 세션이 진단 세션에 모두 있어야 한다. 증명이 없으면 시작 누락이다.
    def test_late_start_proof(self):
        rows = ticks(12.0, 100.0)
        late = [{'recv_mono': 13.0 + 0.2 * i, 'session': 's1'} for i in range(10)]
        early = [{'recv_mono': 5.0, 'session': 's1'}] + late
        foreign = late + [{'recv_mono': 50.0, 'session': 'other'}]

        self.assertIs(self.status(rows, late)['limited'], False)

        for name, health in (('health 없음', None), ('첫 tick 전 health', early), ('다른 세션', foreign)):
            with self.subTest(case=name):
                result = self.status(rows, health)

                self.assertIsNone(result['limited'])
                self.assertIn('시작', result['reason'])


if __name__ == '__main__':
    unittest.main()
