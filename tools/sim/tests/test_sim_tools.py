"""ROS·Gazebo 없이 tools/sim을 회귀 시험한다(무해한 sleep 프로세스만 사용)."""
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import ax_regen_physics  # noqa: E402
import ax_sim  # noqa: E402
import ax_verify  # noqa: E402
import axrun  # noqa: E402


# 조건이 참이 될 때까지 최대 timeout 초 동안 0.05 s 간격으로 확인한다.
def wait_for(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


# 시험마다 별도 AX_RUN_ROOT를 써서 실제 할당 기록을 건드리지 않는 기반 시험 클래스.
class TempRoot(unittest.TestCase):
    # 임시 AX_RUN_ROOT를 환경에 설정하고 자식 프로세스 정리를 등록한다.
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix='ax_sim_test_'))
        patcher = mock.patch.dict(os.environ, {'AX_RUN_ROOT': str(self.tmp / 'runs')})
        patcher.start()
        self.addCleanup(patcher.stop)
        self.children = []
        self.addCleanup(self._cleanup)

    # 이 시험이 띄운 자식 프로세스만 종료·회수하고 임시 폴더를 지운다.
    def _cleanup(self):
        for child in self.children:
            if child.poll() is None:
                child.kill()
            child.wait()
        subprocess.run(['rm', '-rf', str(self.tmp)])

    # 정리 대상으로 기록한 자식 프로세스를 띄운다(기본 sleep 30).
    def spawn(self, *cmd):
        child = subprocess.Popen(cmd or ('sleep', '30'))
        self.children.append(child)
        return child

    # ax_sim start처럼 소유 기록을 먼저 초기화한 뒤 cmd로 띄운 자식 하나를 소유하는 실행 폴더를
    # 만들고 (실행 폴더, 자식)을 돌려준다.
    def owned_run(self, *cmd):
        run = self.tmp / f'run{len(list(self.tmp.glob("run*")))}'
        run.mkdir()
        subprocess.run([sys.executable, HERE / 'own_procs.py', 'init', run / 'own'], check=True)
        child = self.spawn(*cmd)
        (run / 'own/owned.txt').write_text(
            f'{child.pid} {axrun.own_procs.start_time(child.pid)}\n')
        (run / 'token').write_text('test-token-not-used-by-anyone\n')
        axrun.write_json(run / 'run.json', {'run_id': run.name, 'status': 'RUNNING'})
        return run, child


# 결과표 상태 조합에 따른 종료 코드를 시험한다.
class SummaryExitCodes(unittest.TestCase):
    # 주어진 상태들로 결과표를 만들어 종료 코드를 돌려준다.
    def code(self, *states):
        with tempfile.TemporaryDirectory() as tmp:
            summary = axrun.Summary(tmp)
            for i, state in enumerate(states):
                summary.add(f's{i}', state)
            return summary.exit_code()

    # PASS·FAIL·BLOCKED·NOT_RUN·INFO 조합의 종료 코드(0·1·2)를 확인한다.
    def test_codes(self):
        self.assertEqual(self.code('PASS', 'INFO'), 0)
        self.assertEqual(self.code('PASS', 'FAIL', 'BLOCKED'), 1)
        self.assertEqual(self.code('PASS', 'BLOCKED'), 2)      # 필수 단계 미실행: 미완료
        self.assertEqual(self.code('PASS', 'NOT_RUN'), 2)
        self.assertEqual(self.code('INFO'), 2)                 # 아무것도 검증하지 않으면 성공이 아님


# 변경 수집·경로 분류·링크 검사를 시험한다.
class Classification(unittest.TestCase):
    # 경로별로 선택되는 검증 종류와 미분류 경로를 확인한다.
    def test_paths(self):
        c = ax_verify.classify
        self.assertTrue({'model', 'unit', 'pkg:ax_description'}
                        <= c('src/ax_description/config/sensor_mounts.yaml'))
        self.assertIn('model', c('src/ax_description/urdf/ros2_control.xacro'))
        self.assertIn('physics', c('src/ax_description/config/physical_properties.json'))
        self.assertTrue({'pkg:ax_control', 'unit'}
                        <= c('src/ax_control/launch/arm_control.launch.py'))
        self.assertNotIn('comm-smoke', c('src/ax_control/config/ax1_controllers.yaml'))
        self.assertEqual(c('tools/sim/ax_sim.py'), {'simtools'})
        self.assertIn('docs', c('docs/README.md'))
        self.assertEqual(c('maps/warehouse.pgm'), set())       # 미분류로 보고됨

    # 임시 저장소에서 staged·unstaged·untracked·삭제·이름 변경이 모두 수집되는지 확인한다.
    def test_collect_staged_unstaged_untracked_deleted_renamed(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            git = lambda *a: subprocess.run(['git', '-C', tmp, *a], check=True,  # noqa
                                            capture_output=True)
            git('init', '-q')
            git('config', 'user.email', 't@t')
            git('config', 'user.name', 't')

            # 기준 커밋을 만든 뒤 변경 종류마다 하나씩 만든다.
            for name in ('keep.md', 'gone.py', 'old.yaml', 'staged.xacro'):
                (repo / name).write_text(name * 20 + '\n')
            git('add', '.')
            git('commit', '-qm', 'base')
            (repo / 'keep.md').write_text('changed\n')                       # unstaged 변경
            (repo / 'staged.xacro').write_text('staged\n')
            git('add', 'staged.xacro')
            (repo / 'gone.py').unlink()                                      # 삭제
            git('mv', 'old.yaml', 'new.yaml')                                # 이름 변경
            (repo / 'sub').mkdir()
            (repo / 'sub/new file.txt').write_text('x\n')                  # untracked 파일

            with mock.patch.object(ax_verify, 'WS', repo):
                changes = dict((p, s) for s, p in ax_verify.collect_changes())

        self.assertEqual(changes['keep.md'], 'unstaged')
        self.assertEqual(changes['staged.xacro'], 'staged')
        self.assertEqual(changes['gone.py'], 'deleted')
        self.assertEqual(changes['new.yaml'], 'renamed-to')
        self.assertEqual(changes['old.yaml'], 'renamed-from')
        self.assertEqual(changes['sub/new file.txt'], 'untracked')

    # 코드 블록·외부 링크·앵커를 제외하고 없는 대상만 깨진 링크로 찾는지 확인한다.
    def test_broken_links(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'docs').mkdir()
            (root / 'docs/a.md').write_text('a')
            (root / 'x.md').write_text('[ok](docs/a.md#sec) [bad](docs/none.md) [web](https://x.y) '
                                       '`[code](nope.md)` ![img](docs/img.png)\n'
                                       '```\n[f](f.md)\n```\n')
            with mock.patch.object(ax_verify, 'WS', root):
                self.assertEqual(ax_verify.broken_links('x.md'),
                                 ['docs/none.md', 'docs/img.png'])


# domain 할당·회수 규칙을 시험한다.
class DomainAllocation(TempRoot):
    # 임시 할당 폴더의 domain 기록을 읽는다.
    def record(self, domain):
        return axrun.read_json(axrun.run_root() / 'alloc' / f'domain_{domain}.json')

    # 시작 담당 프로세스가 살아 있는 STARTING 기록은 회수되지 않는지 확인한다.
    def test_starting_record_is_not_reclaimed_while_starter_runs(self):
        # domain은 잡혔지만 소유 기록은 아직 없음(own/ 폴더 없음): 시작 담당은 이 프로세스다.
        first = ax_sim.allocate('r1', self.tmp / 'missing_run', requested=70)
        self.assertEqual(first, 70)
        self.assertEqual(self.record(70)['state'], 'STARTING')
        self.assertIsNone(ax_sim.allocate('r2', self.tmp / 'r2', requested=70))
        self.assertNotEqual(ax_sim.allocate('r3', self.tmp / 'r3'), 70)

    # 시작 담당이 죽고 소유 프로세스도 없는 STARTING 기록은 회수되고 로그가 남는지 확인한다.
    def test_starting_record_with_dead_starter_and_no_process_is_reclaimed(self):
        starter = self.spawn('true')
        starter.wait()
        axrun.run_root().joinpath('alloc').mkdir(parents=True)
        axrun.write_json(axrun.run_root() / 'alloc/domain_71.json',
                         {'run_id': 'dead', 'run_dir': str(self.tmp / 'nothing'),
                          'state': 'STARTING', 'starter': [starter.pid, 1]})
        self.assertEqual(ax_sim.allocate('r2', self.tmp / 'r2', requested=71), 71)
        self.assertEqual(self.record(71)['run_id'], 'r2')
        self.assertIn('reclaimed', (axrun.run_root() / 'alloc/reclaimed.log').read_text())

    # 소유 프로세스가 실행 중이면 기록을 유지하고, 좀비만 남으면 회수 가능한지 확인한다.
    def test_running_record_kept_while_owned_process_runs_and_zombie_does_not_count(self):
        run, child = self.owned_run('sleep', '30')
        record = {'run_id': 'busy', 'run_dir': str(run), 'state': 'RUNNING'}
        self.assertIsNone(ax_sim.reclaim_reason(record))
        child.kill()
        self.assertTrue(wait_for(lambda: axrun.proc_state(child.pid) == 'Z'))   # 아직 회수되지 않음
        running, zombies = axrun.owned_processes(run / 'own')
        self.assertEqual((len(running), len(zombies)), (0, 1))
        self.assertIsNotNone(ax_sim.reclaim_reason(record))

    # 읽을 수 없는 기록은 보수적으로 유지하는지 확인한다.
    def test_unreadable_record_is_kept(self):
        self.assertIsNone(ax_sim.reclaim_reason(None))


# 소유 프로세스 정지 동작을 시험한다.
class Stop(TempRoot):
    # 정지가 소유 프로세스에만 신호를 보내고, 반복해도 안전하며 소유 기록을 남기는지 확인한다.
    def test_stop_signals_owned_only_is_repeatable_and_keeps_records(self):
        other = self.spawn('sleep', '31')
        run, owned = self.owned_run('sleep', '30')
        self.assertEqual(ax_sim.stop(run), 0)
        self.assertTrue(wait_for(lambda: owned.poll() is not None))
        self.assertIsNone(other.poll())                     # 소유하지 않은 프로세스는 건드리지 않음
        first = axrun.read_json(run / 'stop.json')
        self.assertEqual(first['result'], 'STOPPED')
        self.assertTrue((run / 'own/owned.txt').exists())                 # 소유 기록은 남김

        self.assertEqual(ax_sim.stop(run), 0)                              # 반복: 안전하며 신호 없음
        again = axrun.read_json(run / 'stop.json')
        self.assertEqual(again['signalled'], {})
        self.assertEqual(again['repeat_of'], first['finished'])

    # 정지 후 남은 좀비는 정지 실패로 세지 않고 따로 기록하는지 확인한다.
    def test_zombie_left_after_stop_is_not_stop_failed(self):
        run, child = self.owned_run('sleep', '30')
        os.kill(child.pid, 9)
        self.assertTrue(wait_for(lambda: axrun.proc_state(child.pid) == 'Z'))
        # 기존 도우미는 좀비를 아직 살아 있다고 나열한다(시작 시각만 확인) ...
        alive = subprocess.run([sys.executable, HERE / 'own_procs.py', 'alive', run / 'own'],
                               capture_output=True, text=True).stdout
        self.assertIn(str(child.pid), alive)
        # ... 정지 계층은 좀비를 따로 구분한다.
        self.assertEqual(ax_sim.stop(run), 0)
        result = axrun.read_json(run / 'stop.json')
        self.assertEqual(result['result'], 'STOPPED')
        self.assertEqual([z[0] for z in result['zombies']], [child.pid])

    # 소유 기록이 없는 실행 폴더는 종료 코드 2인지 확인한다.
    def test_stop_without_ownership_record(self):
        run = self.tmp / 'bare'
        run.mkdir()
        axrun.write_json(run / 'run.json', {'run_id': 'bare'})
        self.assertEqual(ax_sim.stop(run), 2)


# 물리 특성 생성 파일 check·regen 도구를 가짜 생성기로 시험한다.
class Physics(TempRoot):
    # 가짜 작업 트리(메시 하나와 생성 파일 5개)를 만든다.
    def tree(self, outputs):
        ws = self.tmp / 'ws'
        (ws / 'src/ax_description/meshes').mkdir(parents=True)
        (ws / 'src/ax_description/meshes/m.obj').write_text('v 0 0 0\n')
        for out in ax_regen_physics.OUTPUTS:
            (ws / out).parent.mkdir(parents=True, exist_ok=True)
            (ws / out).write_text(outputs.get(out, 'old\n'))
        return ws

    # 생성 파일 중 앞의 count개만 "new"로 쓰는 가짜 생성기를 만든다.
    def fake_generator(self, ws, count):
        script = ws / 'gen.py'
        names = list(ax_regen_physics.OUTPUTS[:count])
        script.write_text('from pathlib import Path\nROOT = Path(__file__).resolve().parents[1]\n'
                          f'for n in {names!r}:\n    (ROOT / n).write_text("new\\n")\n')
        return script

    # 가짜 작업 트리·생성기로 check 또는 regen을 실행해 (단계별 상태, 작업 폴더)를 돌려준다.
    def run_check(self, ws, generator, action):
        work = self.tmp / f'work_{action}'
        work.mkdir()
        summary = axrun.Summary(work)
        with mock.patch.object(ax_regen_physics, 'WS', ws), \
             mock.patch.object(ax_regen_physics, 'GENERATOR', generator):
            getattr(ax_regen_physics, action)(work, summary)
        return {r['step']: r['status'] for r in summary.rows}, work

    # check가 작업 트리를 바꾸지 않고 차이를 FAIL과 diff로 남기는지 확인한다.
    def test_check_never_writes_working_tree(self):
        ws = self.tree({})
        states, work = self.run_check(ws, self.fake_generator(ws, 5), 'check')
        self.assertEqual(states['physics:compare'], 'FAIL')
        self.assertTrue(all((ws / o).read_text() == 'old\n' for o in ax_regen_physics.OUTPUTS))
        self.assertIn('+new', (work / 'diff.txt').read_text())

    # 줄끝만 다른 경우를 따로 표시하는지 확인한다.
    def test_line_ending_only_difference_is_reported_as_such(self):
        ws = self.tree({o: 'new\r\n' for o in ax_regen_physics.OUTPUTS})
        states, work = self.run_check(ws, self.fake_generator(ws, 5), 'check')
        self.assertEqual(states['physics:compare'], 'FAIL')                # 바이트는 여전히 다름
        self.assertIn('line endings differ', (work / 'diff.txt').read_text())

    # 생성 파일 5개가 모두 만들어지지 않으면 regen이 막히고 작업 트리가 그대로인지 확인한다.
    def test_regen_blocked_unless_all_five_outputs_generated(self):
        ws = self.tree({})
        states, _ = self.run_check(ws, self.fake_generator(ws, 4), 'regen')
        self.assertEqual(states['physics:generate'], 'FAIL')
        self.assertEqual(states['physics:regen'], 'BLOCKED')
        self.assertTrue(all((ws / o).read_text() == 'old\n' for o in ax_regen_physics.OUTPUTS))

    # 교체 도중 실패하면 이미 바꾼 파일을 되돌리고 임시 파일을 남기지 않는지 확인한다.
    def test_regen_restores_on_failure_midway(self):
        ws = self.tree({})
        real_replace, calls = os.replace, []

        # 작업 트리 대상의 세 번째 os.replace에서 실패를 흉내 낸다.
        def flaky(src, dst):
            if str(dst).startswith(str(ws)):
                calls.append(dst)
            if str(dst).startswith(str(ws)) and len(calls) == 3:   # 작업 트리의 세 번째 파일
                raise OSError('disk full (simulated)')
            return real_replace(src, dst)

        with mock.patch.object(ax_regen_physics.os, 'replace', flaky):
            states, work = self.run_check(ws, self.fake_generator(ws, 5), 'regen')
        self.assertEqual(states['physics:regen'], 'FAIL')
        self.assertEqual(json.loads((work / 'regen.json').read_text())['result'], 'RESTORED')
        self.assertTrue(all((ws / o).read_text() == 'old\n' for o in ax_regen_physics.OUTPUTS))
        self.assertEqual(list(ws.rglob('*.axregen.tmp')), [])

    # 준비 단계 실패 후 regen이 FAIL·RESTORED이고 작업 트리와 임시 파일 흔적이 없는지 확인한다.
    def assert_failed_staging_left_no_trace(self, ws, states, work):
        self.assertEqual(states['physics:regen'], 'FAIL')
        self.assertEqual(json.loads((work / 'regen.json').read_text())['result'], 'RESTORED')
        self.assertTrue(all((ws / o).read_text() == 'old\n' for o in ax_regen_physics.OUTPUTS))
        self.assertEqual(list(ws.rglob('*.axregen.tmp')), [])

    # 임시 사본을 반쯤 쓰다 실패해도 임시 파일을 지우는지 확인한다.
    def test_regen_partial_temp_copy_failure_removes_temp(self):
        ws = self.tree({})
        real_copy, temps = ax_regen_physics.shutil.copy2, []

        # 두 번째 임시 사본을 반만 쓰고 실패를 흉내 낸다.
        def partial(src, dst, *args, **kwargs):
            if str(dst).endswith('.axregen.tmp'):
                temps.append(dst)
                if len(temps) == 2:                        # 두 번째 임시 사본: 반만 쓴 뒤 오류
                    Path(dst).write_bytes(Path(src).read_bytes()[:2])
                    raise OSError('copy interrupted (simulated)')
            return real_copy(src, dst, *args, **kwargs)

        with mock.patch.object(ax_regen_physics.shutil, 'copy2', partial):
            states, work = self.run_check(ws, self.fake_generator(ws, 5), 'regen')
        self.assertEqual(len(temps), 2)
        self.assert_failed_staging_left_no_trace(ws, states, work)

    # fsync 실패 시에도 임시 파일을 지우고 되돌리는지 확인한다.
    def test_regen_fsync_failure_removes_temp(self):
        ws = self.tree({})
        with mock.patch.object(ax_regen_physics.os, 'fsync',
                               side_effect=OSError('fsync (simulated)')):
            states, work = self.run_check(ws, self.fake_generator(ws, 5), 'regen')
        self.assert_failed_staging_left_no_trace(ws, states, work)

    # regen이 5개를 모두 교체하고 백업을 남기며, 교체 후 생성기 출력과 같은지 확인한다.
    def test_regen_replaces_all(self):
        ws = self.tree({})
        work = self.tmp / 'work_regen'
        work.mkdir()
        summary = axrun.Summary(work)
        with mock.patch.object(ax_regen_physics, 'WS', ws), \
             mock.patch.object(ax_regen_physics, 'GENERATOR', self.fake_generator(ws, 5)):
            ax_regen_physics.regen(work, summary)

        states = {r['step']: r['status'] for r in summary.rows}
        self.assertEqual(states['physics:compare'], 'INFO')     # regen 전 차이는 입력
        self.assertEqual(states['physics:regen'], 'PASS')
        self.assertEqual(states['physics:verify'], 'PASS')      # 교체 후 생성기 출력과 바이트가 같음
        self.assertEqual(summary.exit_code(), 0)
        self.assertTrue(all((ws / o).read_text() == 'new\n' for o in ax_regen_physics.OUTPUTS))
        self.assertTrue(all((work / 'backup' / o).read_text() == 'old\n'
                            for o in ax_regen_physics.OUTPUTS))


if __name__ == '__main__':
    unittest.main()
