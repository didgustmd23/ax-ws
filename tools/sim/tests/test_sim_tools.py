"""Regression tests of tools/sim without ROS or Gazebo (harmless sleep processes only)."""
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


def wait_for(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


class TempRoot(unittest.TestCase):
    """Each test gets its own AX_RUN_ROOT, so it never touches real allocation records."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix='ax_sim_test_'))
        patcher = mock.patch.dict(os.environ, {'AX_RUN_ROOT': str(self.tmp / 'runs')})
        patcher.start()
        self.addCleanup(patcher.stop)
        self.children = []
        self.addCleanup(self._cleanup)

    def _cleanup(self):
        for child in self.children:
            if child.poll() is None:
                child.kill()
            child.wait()
        subprocess.run(['rm', '-rf', str(self.tmp)])

    def spawn(self, *cmd):
        child = subprocess.Popen(cmd or ('sleep', '30'))
        self.children.append(child)
        return child

    def owned_run(self, *cmd):
        """Run folder whose ownership state (initialised first, as ax_sim start does) owns one
        child started from `cmd`; returns (run folder, child)."""
        run = self.tmp / f'run{len(list(self.tmp.glob("run*")))}'
        run.mkdir()
        subprocess.run([sys.executable, HERE / 'own_procs.py', 'init', run / 'own'], check=True)
        child = self.spawn(*cmd)
        (run / 'own/owned.txt').write_text(f'{child.pid} {axrun.own_procs.start_time(child.pid)}\n')
        (run / 'token').write_text('test-token-not-used-by-anyone\n')
        axrun.write_json(run / 'run.json', {'run_id': run.name, 'status': 'RUNNING'})
        return run, child


class SummaryExitCodes(unittest.TestCase):
    def code(self, *states):
        with tempfile.TemporaryDirectory() as tmp:
            summary = axrun.Summary(tmp)
            for i, state in enumerate(states):
                summary.add(f's{i}', state)
            return summary.exit_code()

    def test_codes(self):
        self.assertEqual(self.code('PASS', 'INFO'), 0)
        self.assertEqual(self.code('PASS', 'FAIL', 'BLOCKED'), 1)
        self.assertEqual(self.code('PASS', 'BLOCKED'), 2)      # required step not run: incomplete
        self.assertEqual(self.code('PASS', 'NOT_RUN'), 2)
        self.assertEqual(self.code('INFO'), 2)                 # nothing verified is not success


class Classification(unittest.TestCase):
    def test_paths(self):
        c = ax_verify.classify
        self.assertTrue({'model', 'unit', 'pkg:ax_description'}
                        <= c('src/ax_description/config/sensor_mounts.yaml'))
        self.assertIn('model', c('src/ax_description/urdf/ros2_control.xacro'))
        self.assertIn('physics', c('src/ax_description/config/physical_properties.json'))
        self.assertTrue({'pkg:ax_control', 'unit'} <= c('src/ax_control/launch/arm_control.launch.py'))
        self.assertNotIn('comm-smoke', c('src/ax_control/config/ax1_controllers.yaml'))
        self.assertEqual(c('tools/sim/ax_sim.py'), {'simtools'})
        self.assertIn('docs', c('docs/README.md'))
        self.assertEqual(c('maps/warehouse.pgm'), set())       # reported as unclassified

    def test_collect_staged_unstaged_untracked_deleted_renamed(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            git = lambda *a: subprocess.run(['git', '-C', tmp, *a], check=True, capture_output=True)  # noqa
            git('init', '-q'); git('config', 'user.email', 't@t'); git('config', 'user.name', 't')
            for name in ('keep.md', 'gone.py', 'old.yaml', 'staged.xacro'):
                (repo / name).write_text(name * 20 + '\n')
            git('add', '.'); git('commit', '-qm', 'base')
            (repo / 'keep.md').write_text('changed\n')                       # unstaged
            (repo / 'staged.xacro').write_text('staged\n'); git('add', 'staged.xacro')
            (repo / 'gone.py').unlink()                                      # deleted
            git('mv', 'old.yaml', 'new.yaml')                                # renamed
            (repo / 'sub').mkdir(); (repo / 'sub/new file.txt').write_text('x\n')  # untracked
            with mock.patch.object(ax_verify, 'WS', repo):
                changes = dict((p, s) for s, p in ax_verify.collect_changes())
        self.assertEqual(changes['keep.md'], 'unstaged')
        self.assertEqual(changes['staged.xacro'], 'staged')
        self.assertEqual(changes['gone.py'], 'deleted')
        self.assertEqual(changes['new.yaml'], 'renamed-to')
        self.assertEqual(changes['old.yaml'], 'renamed-from')
        self.assertEqual(changes['sub/new file.txt'], 'untracked')

    def test_broken_links(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'docs').mkdir(); (root / 'docs/a.md').write_text('a')
            (root / 'x.md').write_text('[ok](docs/a.md#sec) [bad](docs/none.md) [web](https://x.y) '
                                       '`[code](nope.md)` ![img](docs/img.png)\n```\n[f](f.md)\n```\n')
            with mock.patch.object(ax_verify, 'WS', root):
                self.assertEqual(ax_verify.broken_links('x.md'), ['docs/none.md', 'docs/img.png'])


class DomainAllocation(TempRoot):
    def record(self, domain):
        return axrun.read_json(axrun.run_root() / 'alloc' / f'domain_{domain}.json')

    def test_starting_record_is_not_reclaimed_while_starter_runs(self):
        # Domain taken, ownership not recorded yet (no own/ folder): the starter is this process.
        first = ax_sim.allocate('r1', self.tmp / 'missing_run', requested=70)
        self.assertEqual(first, 70)
        self.assertEqual(self.record(70)['state'], 'STARTING')
        self.assertIsNone(ax_sim.allocate('r2', self.tmp / 'r2', requested=70))
        self.assertNotEqual(ax_sim.allocate('r3', self.tmp / 'r3'), 70)

    def test_starting_record_with_dead_starter_and_no_process_is_reclaimed(self):
        starter = self.spawn('true'); starter.wait()
        axrun.run_root().joinpath('alloc').mkdir(parents=True)
        axrun.write_json(axrun.run_root() / 'alloc/domain_71.json',
                         {'run_id': 'dead', 'run_dir': str(self.tmp / 'nothing'), 'state': 'STARTING',
                          'starter': [starter.pid, 1]})
        self.assertEqual(ax_sim.allocate('r2', self.tmp / 'r2', requested=71), 71)
        self.assertEqual(self.record(71)['run_id'], 'r2')
        self.assertIn('reclaimed', (axrun.run_root() / 'alloc/reclaimed.log').read_text())

    def test_running_record_kept_while_owned_process_runs_and_zombie_does_not_count(self):
        run, child = self.owned_run('sleep', '30')
        record = {'run_id': 'busy', 'run_dir': str(run), 'state': 'RUNNING'}
        self.assertIsNone(ax_sim.reclaim_reason(record))
        child.kill()
        self.assertTrue(wait_for(lambda: axrun.proc_state(child.pid) == 'Z'))   # not reaped yet
        running, zombies = axrun.owned_processes(run / 'own')
        self.assertEqual((len(running), len(zombies)), (0, 1))
        self.assertIsNotNone(ax_sim.reclaim_reason(record))

    def test_unreadable_record_is_kept(self):
        self.assertIsNone(ax_sim.reclaim_reason(None))


class Stop(TempRoot):
    def test_stop_signals_owned_only_is_repeatable_and_keeps_records(self):
        other = self.spawn('sleep', '31')
        run, owned = self.owned_run('sleep', '30')
        self.assertEqual(ax_sim.stop(run), 0)
        self.assertTrue(wait_for(lambda: owned.poll() is not None))
        self.assertIsNone(other.poll())                                    # not owned, untouched
        first = axrun.read_json(run / 'stop.json')
        self.assertEqual(first['result'], 'STOPPED')
        self.assertTrue((run / 'own/owned.txt').exists())                 # ownership record kept
        self.assertEqual(ax_sim.stop(run), 0)                              # repeat: safe, no signal
        again = axrun.read_json(run / 'stop.json')
        self.assertEqual(again['signalled'], {})
        self.assertEqual(again['repeat_of'], first['finished'])

    def test_zombie_left_after_stop_is_not_stop_failed(self):
        run, child = self.owned_run('sleep', '30')
        os.kill(child.pid, 9)
        self.assertTrue(wait_for(lambda: axrun.proc_state(child.pid) == 'Z'))
        # The existing helper still lists the zombie as alive (start time only) ...
        alive = subprocess.run([sys.executable, HERE / 'own_procs.py', 'alive', run / 'own'],
                               capture_output=True, text=True).stdout
        self.assertIn(str(child.pid), alive)
        # ... the stop layer separates it.
        self.assertEqual(ax_sim.stop(run), 0)
        result = axrun.read_json(run / 'stop.json')
        self.assertEqual(result['result'], 'STOPPED')
        self.assertEqual([z[0] for z in result['zombies']], [child.pid])

    def test_stop_without_ownership_record(self):
        run = self.tmp / 'bare'; run.mkdir()
        axrun.write_json(run / 'run.json', {'run_id': 'bare'})
        self.assertEqual(ax_sim.stop(run), 2)


class Physics(TempRoot):
    def tree(self, outputs):
        ws = self.tmp / 'ws'
        (ws / 'src/ax_description/meshes').mkdir(parents=True)
        (ws / 'src/ax_description/meshes/m.obj').write_text('v 0 0 0\n')
        for out in ax_regen_physics.OUTPUTS:
            (ws / out).parent.mkdir(parents=True, exist_ok=True)
            (ws / out).write_text(outputs.get(out, 'old\n'))
        return ws

    def fake_generator(self, ws, count):
        script = ws / 'gen.py'
        names = list(ax_regen_physics.OUTPUTS[:count])
        script.write_text('from pathlib import Path\nROOT = Path(__file__).resolve().parents[1]\n'
                          f'for n in {names!r}:\n    (ROOT / n).write_text("new\\n")\n')
        return script

    def run_check(self, ws, generator, action):
        work = self.tmp / f'work_{action}'; work.mkdir()
        summary = axrun.Summary(work)
        with mock.patch.object(ax_regen_physics, 'WS', ws), \
             mock.patch.object(ax_regen_physics, 'GENERATOR', generator):
            getattr(ax_regen_physics, action)(work, summary)
        return {r['step']: r['status'] for r in summary.rows}, work

    def test_check_never_writes_working_tree(self):
        ws = self.tree({})
        states, work = self.run_check(ws, self.fake_generator(ws, 5), 'check')
        self.assertEqual(states['physics:compare'], 'FAIL')
        self.assertTrue(all((ws / o).read_text() == 'old\n' for o in ax_regen_physics.OUTPUTS))
        self.assertIn('+new', (work / 'diff.txt').read_text())

    def test_line_ending_only_difference_is_reported_as_such(self):
        ws = self.tree({o: 'new\r\n' for o in ax_regen_physics.OUTPUTS})
        states, work = self.run_check(ws, self.fake_generator(ws, 5), 'check')
        self.assertEqual(states['physics:compare'], 'FAIL')                # bytes still differ
        self.assertIn('line endings differ', (work / 'diff.txt').read_text())

    def test_regen_blocked_unless_all_five_outputs_generated(self):
        ws = self.tree({})
        states, _ = self.run_check(ws, self.fake_generator(ws, 4), 'regen')
        self.assertEqual(states['physics:generate'], 'FAIL')
        self.assertEqual(states['physics:regen'], 'BLOCKED')
        self.assertTrue(all((ws / o).read_text() == 'old\n' for o in ax_regen_physics.OUTPUTS))

    def test_regen_restores_on_failure_midway(self):
        ws = self.tree({})
        real_replace, calls = os.replace, []

        def flaky(src, dst):
            if str(dst).startswith(str(ws)):
                calls.append(dst)
            if str(dst).startswith(str(ws)) and len(calls) == 3:   # third working-tree file
                raise OSError('disk full (simulated)')
            return real_replace(src, dst)
        with mock.patch.object(ax_regen_physics.os, 'replace', flaky):
            states, work = self.run_check(ws, self.fake_generator(ws, 5), 'regen')
        self.assertEqual(states['physics:regen'], 'FAIL')
        self.assertEqual(json.loads((work / 'regen.json').read_text())['result'], 'RESTORED')
        self.assertTrue(all((ws / o).read_text() == 'old\n' for o in ax_regen_physics.OUTPUTS))
        self.assertEqual(list(ws.rglob('*.axregen.tmp')), [])

    def assert_failed_staging_left_no_trace(self, ws, states, work):
        self.assertEqual(states['physics:regen'], 'FAIL')
        self.assertEqual(json.loads((work / 'regen.json').read_text())['result'], 'RESTORED')
        self.assertTrue(all((ws / o).read_text() == 'old\n' for o in ax_regen_physics.OUTPUTS))
        self.assertEqual(list(ws.rglob('*.axregen.tmp')), [])

    def test_regen_partial_temp_copy_failure_removes_temp(self):
        ws = self.tree({})
        real_copy, temps = ax_regen_physics.shutil.copy2, []

        def partial(src, dst, *args, **kwargs):
            if str(dst).endswith('.axregen.tmp'):
                temps.append(dst)
                if len(temps) == 2:                        # second temp: half written, then error
                    Path(dst).write_bytes(Path(src).read_bytes()[:2])
                    raise OSError('copy interrupted (simulated)')
            return real_copy(src, dst, *args, **kwargs)
        with mock.patch.object(ax_regen_physics.shutil, 'copy2', partial):
            states, work = self.run_check(ws, self.fake_generator(ws, 5), 'regen')
        self.assertEqual(len(temps), 2)
        self.assert_failed_staging_left_no_trace(ws, states, work)

    def test_regen_fsync_failure_removes_temp(self):
        ws = self.tree({})
        with mock.patch.object(ax_regen_physics.os, 'fsync', side_effect=OSError('fsync (simulated)')):
            states, work = self.run_check(ws, self.fake_generator(ws, 5), 'regen')
        self.assert_failed_staging_left_no_trace(ws, states, work)

    def test_regen_replaces_all(self):
        ws = self.tree({})
        work = self.tmp / 'work_regen'; work.mkdir()
        summary = axrun.Summary(work)
        with mock.patch.object(ax_regen_physics, 'WS', ws), \
             mock.patch.object(ax_regen_physics, 'GENERATOR', self.fake_generator(ws, 5)):
            ax_regen_physics.regen(work, summary)
        states = {r['step']: r['status'] for r in summary.rows}
        self.assertEqual(states['physics:compare'], 'INFO')     # pre-regen difference is the input
        self.assertEqual(states['physics:regen'], 'PASS')
        self.assertEqual(states['physics:verify'], 'PASS')      # bytes match the generator after
        self.assertEqual(summary.exit_code(), 0)
        self.assertTrue(all((ws / o).read_text() == 'new\n' for o in ax_regen_physics.OUTPUTS))
        self.assertTrue(all((work / 'backup' / o).read_text() == 'old\n' for o in ax_regen_physics.OUTPUTS))


if __name__ == '__main__':
    unittest.main()
