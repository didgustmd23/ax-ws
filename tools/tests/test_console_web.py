"""관제 웹 화면 JS 점검(브라우저 대체 아님). node가 필요하다(없으면 실패).

확인:
- web/app.js·index.html 안 스크립트 문법(node --check)
- 가짜 DOM에서 ConsoleState가 만든 실제 형식의 상태 6종(정상·만료·전부 미수집·무효·표시 여유·시계 무효)을 렌더:
  예외 0, 출력에 undefined·NaN·[object 없음, svg·table 태그 짝 일치, 만료·미수집 칸이 "값 아님"으로 표시
- 외부 자산 참조 없음(http·https·CDN·@import·웹폰트)
"""
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / 'src' / 'ax_operations_console'
WEB = PKG / 'web'
sys.path.insert(0, str(PKG))

from ax_operations_console.console_state import Clocks, ConsoleState, Sample  # noqa: E402

ROBOTS = [{'robot_id': f'robot_{i}', 'role': 'MANIPULATOR' if i % 2 else 'TRANSPORTER',
           'spare': i > 2} for i in range(1, 5)]

# 가짜 DOM 렌더 하네스(node에서 실행). app.js 경로와 표본 JSON 파일 경로를 인자로 받는다
HARNESS = r"""
const fs = require('fs');
const vm = require('vm');
const els = {};
const el = id => (els[id] ??= {
  id, innerHTML: '', textContent: '', className: '', style: {setProperty() {}},
  classList: {add() {}, remove() {}, toggle() {}}, querySelectorAll: () => [], onclick: null});
let t = 0;
const ctx = {
  document: {
    getElementById: el, querySelector: () => null, documentElement: {},
    body: {set innerHTML(v) {}, classList: {toggle() {}}}},
  getComputedStyle: () => ({getPropertyValue: () => '#808080'}),
  performance: {now: () => (t += 1100)},
  window: {addEventListener() {}, devicePixelRatio: 1},
  console, Date, Math, JSON, Object, String, Number, Array,
};
vm.createContext(ctx);
const exportNames = ';this.buildSkeleton=buildSkeleton;this.renderAll=renderAll;' +
  'this.select=select;this.setConn=setConn;';
vm.runInContext(fs.readFileSync(process.argv[2], 'utf8') + exportNames, ctx);
const view = {init() {}, update(r) { view.n = r.length; }};
ctx.buildSkeleton(view);
const samples = JSON.parse(fs.readFileSync(process.argv[3], 'utf8'));
const areas = ['kpi-run', 'kpi-task', 'tasks', 'cards', 'dbody', 'no-source', 'sim'];
const report = {};
for (const [name, snap] of Object.entries(samples)) {
  const pass = {};
  for (const sel of [null, 'robot_1']) {
    if (sel) ctx.select(sel); else ctx.renderAll(snap);
    for (const id of areas) {
      const h = (els[id]?.innerHTML ?? '') + (els[id]?.textContent ?? '');
      const count = re => (h.match(re) || []).length;
      pass[`${sel}:${id}`] = {
        bad: /undefined|NaN|\[object/.test(h),
        svg: count(/<svg/g) - count(/<\/svg>/g),
        table: count(/<table/g) - count(/<\/table>/g)};
    }
    if (sel) ctx.select(sel);
  }
  ctx.setConn(false, '연결 끊김');
  const cards = els.cards.innerHTML;
  const count = re => (cards.match(re) || []).length;
  report[name] = {pass, stale: count(/nv stale/g), missing: count(/nv missing/g),
    invalid: count(/nv invalid/g), grace: count(/dot ok grace/g), robots: view.n};
}
console.log(JSON.stringify(report));
"""

VALUES = {
    'actuation': {'state': 'RUNNING', 'drive_allowed': True, 'arm_allowed': False,
                  'block_reasons': ['HANDOFF_LOCK'], 'linear_speed': 0.2},
    'safety': {'level': 'CLEAR', 'reason': '', 'max_linear_speed': 0.67},
    'health': {'state': 'DEGRADED', 'position_std_m': 0.12},
    'executor': {'task_id': 't1', 'local_phase': 'PICKING', 'paused': False,
                 'last_result': 'UNSPECIFIED'},
    'pose': {'x': 1.0, 'y': -2.0, 'yaw': 0.5, 'frame': 'map'},
    'battery': {'percentage': 0.18, 'present': True},
}
TASK_ROWS = [{'task_id': 't1', 'state': 'RUNNING', 'transfer': 'LOCKED', 'manipulator': 'robot_1',
              'transporter': 'robot_2', 'source': 'station_src', 'destination': 'station_dst'}]


# 모든 로봇의 6항목과 작업 목록을 sim 100 s에 반영한 상태
def filled(valid=True, grace=0.0):
    state = ConsoleState(ROBOTS, 'dev_config', display_grace_s=grace)
    clocks = Clocks(100.0, 50.0)

    for robot in ROBOTS:
        for kind, value in VALUES.items():
            meta = kind not in ('pose', 'battery')
            smp = Sample(value=value, stamp=100.0, session='s' if meta else None,
                         revision=1 if meta else None, valid=valid,
                         reason='' if valid else 'SENSOR_FAULT', source_id='src', schema=2)
            state.update_robot(robot['robot_id'], kind, smp, clocks)

    tasks = Sample(value=TASK_ROWS, stamp=100.0, session='d', revision=1, source_id='d2')
    state.update_tasks(tasks, clocks)
    return state


# 시험용 표본 상태 6종(ConsoleState로 만든 실제 형식)
def make_samples():
    out = {
        'normal': filled().build_state(Clocks(100.1, 50.1)),
        'stale': filled().build_state(Clocks(110.0, 60.0)),
        'missing': ConsoleState(ROBOTS, 'dev_config').build_state(Clocks(100.0, 50.0)),
        'invalid': filled(valid=False).build_state(Clocks(100.1, 50.1)),
        'grace': filled(grace=0.5).build_state(Clocks(100.4, 50.4)),
        'clock_invalid': filled().build_state(Clocks(0.0, 50.1)),
    }

    for state in out.values():
        state['topics'] = {'profile': 'dev', 'tasks': '/ax/dev/d2_mock/fleet_tasks'}

    return out


class WebTest(unittest.TestCase):
    """node로 JS를 점검한다."""

    # node 경로(없으면 실패: skip하지 않음)
    @classmethod
    def setUpClass(cls):
        cls.node = shutil.which('node')
        assert cls.node, 'node가 없어 웹 JS 점검을 할 수 없음'
        cls.tmp = tempfile.TemporaryDirectory()

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    # app.js와 index.html 안 스크립트 문법
    def test_syntax(self):
        subprocess.run([self.node, '--check', str(WEB / 'app.js')], check=True)
        html = (WEB / 'index.html').read_text(encoding='utf-8')
        inline = re.findall(r'<script>(.*?)</script>', html, re.S)
        self.assertEqual(len(inline), 1)
        path = Path(self.tmp.name) / 'inline.js'
        path.write_text(inline[0], encoding='utf-8')
        subprocess.run([self.node, '--check', str(path)], check=True)

    # 가짜 DOM 렌더 6종
    def test_fake_dom_render(self):
        tmp = Path(self.tmp.name)
        (tmp / 'harness.js').write_text(HARNESS, encoding='utf-8')
        (tmp / 'samples.json').write_text(json.dumps(make_samples()), encoding='utf-8')
        command = [self.node, str(tmp / 'harness.js'), str(WEB / 'app.js'),
                   str(tmp / 'samples.json')]
        result = subprocess.run(command, capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)

        for name, entry in report.items():
            for key, check in entry['pass'].items():
                with self.subTest(sample=name, area=key):
                    self.assertEqual(check, {'bad': False, 'svg': 0, 'table': 0})
            self.assertEqual(entry['robots'], 4)

        # 카드 칸 수: 로봇 4대 × (구동·허가·안전·위치 신뢰·배터리·배정) 6칸
        normal = report['normal']
        self.assertEqual(normal['stale'] + normal['missing'] + normal['invalid'], 0)
        self.assertEqual(report['stale']['stale'], 24)
        self.assertEqual(report['missing']['missing'], 24)
        self.assertEqual(report['invalid']['invalid'], 20)   # 배정은 작업 목록 기준이라 정상
        # 구동(0.3 s 기준) 2칸 × 4대가 만료 직후 표시 여유
        self.assertGreaterEqual(report['grace']['grace'], 8)
        self.assertEqual(report['clock_invalid']['invalid'], 24)

    # 외부 자산 참조 없음
    def test_no_external_assets(self):
        for path in WEB.iterdir():
            text = path.read_text(encoding='utf-8')
            with self.subTest(file=path.name):
                self.assertNotRegex(text, r'https?://|//cdn|@import|@font-face|fonts\.googleapis')


if __name__ == '__main__':
    unittest.main()
