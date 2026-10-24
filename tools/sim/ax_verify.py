#!/usr/bin/env python3
"""Change-type verification for ax-ws with one summary.tsv per run.

  ax_verify.py [auto] [docs] [model] [unit] [physics] [simtools] [comm-smoke] [pkg=NAME,...]
               [--base REF] [--isolated-build] [--sim-model ax1|ax2|both] [--sim-timeout S]

auto   selects kinds from staged, unstaged and untracked changes (renames: both paths, deletions:
       the old path) and, with --base, from commits since REF. Unclassified paths are reported as INFO.
docs   relative links and image paths of Markdown files exist (anchors are not checked)
model  xacro + check_urdf of ax.urdf.xacro (4 grippers) and ax_2.urdf.xacro from this run's build
unit   python3 -m unittest discover tools/tests against this run's build
physics generator run in a copy tree compared with the working tree (never writes the working
       tree; use ax_regen_physics.py regen to replace files explicitly)
simtools regression tests of tools/sim (owned-process helper and these tools)
comm-smoke basic communication smoke: message reception only, not controller activation,
       tracking or stopping (never selected by auto)
pkg=   colcon build + colcon test of the packages, results limited to this run's result folder

Steps that need the workspace build (model, unit, pkg, comm-smoke) depend on a colcon build in the
same run; if it fails they are BLOCKED and never run against an earlier install. Exit codes: see
axrun.py (0 all PASS, 1 FAIL, 2 incomplete, 3 usage). Full logs stay in the run folder.
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote

import axrun
import ax_regen_physics
import ax_sim
from axrun import WS, Summary, write_json

KINDS = ('docs', 'model', 'unit', 'physics', 'simtools', 'comm-smoke')
BUILD_NEEDS = {
    'model': {'ax_description', 'robotiq_description'},
    'unit': {'ax_description', 'robotiq_description', 'ax_control'},   # tests import ax_control
    'comm-smoke': {'ax_description', 'robotiq_description', 'ax_control'},
}
GRIPPERS = ('primitive', 'robotiq', 'parallel_wide', 'suction')


# Change collection --------------------------------------------------------------------------

def _git(*args):
    return subprocess.run(['git', *args], cwd=WS, capture_output=True, check=True).stdout.decode()


def collect_changes(base=None):
    """[(status, path)] of staged, unstaged and untracked changes (+ commits since `base`)."""
    changes = []
    fields = _git('status', '--porcelain=v2', '-z', '--untracked-files=all').split('\0')
    i = 0
    while i < len(fields):
        entry = fields[i]
        i += 1
        if not entry:
            continue
        if entry[0] == '?':
            changes.append(('untracked', entry[2:]))
        elif entry[0] == '1':
            parts = entry.split(' ', 8)
            changes.append((_label(parts[1]), parts[8]))
        elif entry[0] == '2':                       # rename/copy: new path, then original path
            parts = entry.split(' ', 9)
            changes.append(('renamed-to', parts[9]))
            changes.append(('renamed-from', fields[i]))
            i += 1
        elif entry[0] == 'u':
            changes.append(('unmerged', entry.split(' ', 10)[10]))
    if base:
        fields = _git('diff', '-z', '--name-status', '--find-renames', f'{base}...HEAD').split('\0')
        i = 0
        while i < len(fields) - 1:
            status = fields[i]
            if status[:1] in 'RC':
                changes += [('committed-renamed-from', fields[i + 1]), ('committed-renamed-to', fields[i + 2])]
                i += 3
            else:
                kind = {'A': 'added', 'D': 'deleted', 'M': 'modified'}.get(status[:1], status)
                changes.append(('committed-' + kind, fields[i + 1]))
                i += 2
    seen, unique = set(), []
    for item in changes:
        if item not in seen:
            seen.add(item); unique.append(item)
    return unique


def _label(xy):
    if 'D' in xy:
        return 'deleted'
    if xy[0] not in '.':
        return 'staged' + ('+unstaged' if xy[1] != '.' else '')
    return 'unstaged'


def classify(path):
    """Kinds selected by one changed path; empty set = unclassified."""
    kinds = set()
    if path.startswith('src/ax_description/'):
        kinds |= {'model', 'unit', 'pkg:ax_description'}      # incl. config/sensor_mounts.yaml
        if path.startswith('src/ax_description/meshes/parts/') or path in ax_regen_physics.OUTPUTS:
            kinds.add('physics')
    elif path.startswith('src/robotiq_description/'):
        kinds.add('model')                                     # vendored model subset, no tests
    elif path.startswith('src/ax_control/'):
        kinds |= {'pkg:ax_control', 'unit', 'suggest:comm-smoke'}
    elif path == 'tools/generate_physical_properties.py':
        kinds |= {'physics', 'model', 'unit'}
    elif path.startswith('tools/sim/'):
        kinds.add('simtools')
    elif path.startswith('tools/'):
        kinds.add('unit')
    if path.endswith('.md') or path.startswith('docs/'):
        kinds.add('docs')
    return kinds


# Steps --------------------------------------------------------------------------------------

def markdown_files(changes=None):
    if changes is None or any(s.endswith(('deleted', 'renamed-from')) and p.endswith('.md')
                              for s, p in changes):
        listed = _git('ls-files', '-z', '--cached', '--others', '--exclude-standard', '*.md')
        return sorted({p for p in listed.split('\0') if p and (WS / p).exists()})
    return sorted({p for s, p in changes if p.endswith('.md') and (WS / p).exists()})


LINK = re.compile(r'!?\[[^\]]*\]\(\s*<?([^)\s>]+)>?(?:\s+"[^"]*")?\s*\)')


def broken_links(path):
    text = re.sub(r'```.*?```', '', (WS / path).read_text(errors='replace'), flags=re.S)
    text = re.sub(r'`[^`\n]*`', '', text)
    broken = []
    for target in LINK.findall(text):
        if re.match(r'^[a-z][a-z0-9+.-]*:', target, re.I) or target.startswith('#'):
            continue
        file = unquote(target.split('#', 1)[0])
        resolved = WS / file.lstrip('/') if file.startswith('/') else (WS / path).parent / file
        if not resolved.exists():
            broken.append(target)
    return broken


def step_docs(run, summary, changes):
    files = markdown_files(changes)
    log = run / 'docs.log'
    problems = {f: b for f in files if (b := broken_links(f))}
    log.write_text(f'checked {len(files)} files (anchors not checked)\n' + ''.join(
        f'{f}: {t}\n' for f, b in problems.items() for t in b))
    count = sum(map(len, problems.values()))
    summary.add('docs:links', 'FAIL' if count else 'PASS', log=log,
                note=f'{count} broken link(s) in {len(files)} file(s); anchors not checked')


def step_build(run, summary, packages, isolated):
    base = ['--build-base', run / 'build', '--install-base', run / 'install'] if isolated else []
    install = run / 'install' if isolated else WS / 'install'
    env = axrun.ros_env()                     # ROS only: an earlier workspace install is not sourced
    rc, sec = axrun.run_step(['colcon', '--log-base', run / 'colcon_log', 'build', '--symlink-install',
                              '--packages-up-to', *sorted(packages), *base], run / 'build.log', env=env)
    summary.add('build', 'PASS' if rc == 0 else 'FAIL', rc, sec, run / 'build.log',
                f"{' '.join(sorted(packages))} -> {install}")
    if rc:
        return None, None
    env = axrun.ros_env(install)
    log = run / 'env.log'
    wrong = []
    with open(log, 'w') as out:
        for package in sorted(packages):
            prefix = subprocess.run(['ros2', 'pkg', 'prefix', package], env=env, capture_output=True,
                                    text=True).stdout.strip()
            out.write(f'{package}: {prefix}\n')
            if not prefix or not Path(prefix).resolve().is_relative_to(install.resolve()):
                wrong.append(package)
    summary.add('build:env', 'FAIL' if wrong else 'PASS', log=log,
                note=f'not resolved from this build: {wrong}' if wrong else f'packages from {install}')
    return (None, None) if wrong else (install, env)


def step_model(run, summary, env):
    out = run / 'model'
    out.mkdir(exist_ok=True)
    share = subprocess.run(['ros2', 'pkg', 'prefix', '--share', 'ax_description'], env=env,
                           capture_output=True, text=True).stdout.strip()
    variants = [(f'ax:{g}', 'ax.urdf.xacro', [f'gripper_model:={g}']) for g in GRIPPERS]
    variants.append(('ax_2', 'ax_2.urdf.xacro', []))
    for name, source, args in variants:
        urdf = out / f"{name.replace(':', '_')}.urdf"
        cmd = ['bash', '-c', 'xacro "$1" "${@:3}" -o "$2" && check_urdf "$2"', '_',
               f'{share}/urdf/{source}', urdf, *args]
        rc, sec = axrun.run_step(cmd, out / f"{name.replace(':', '_')}.log", env=env)
        summary.add(f'model:{name}', 'PASS' if rc == 0 else 'FAIL', rc, sec,
                    out / f"{name.replace(':', '_')}.log")


def step_unit(run, summary, env, folder='tools/tests', step='unit'):
    log = run / f"{step.replace(':', '_')}.log"
    rc, sec = axrun.run_step([sys.executable, '-m', 'unittest', 'discover', '-s', folder, '-v'],
                             log, env=env)
    summary.add(step, 'PASS' if rc == 0 else 'FAIL', rc, sec, log)


def step_pkg_test(run, summary, env, install, package, isolated):
    base = (['--build-base', run / 'build', '--install-base', install] if isolated else [])
    results = run / 'test_results'
    log = run / f'test_{package}.log'
    rc, sec = axrun.run_step(['colcon', '--log-base', run / 'colcon_log', 'test', '--packages-select',
                              package, '--test-result-base', results, '--return-code-on-test-failure',
                              *base], log, env=env)
    listing = subprocess.run(['colcon', 'test-result', '--test-result-base', results, '--all',
                              '--result-files-only'], env=env, cwd=WS, capture_output=True, text=True)
    files = [f for f in listing.stdout.splitlines() if Path(f).resolve().is_relative_to(
        (results / package).resolve())]
    verbose = subprocess.run(['colcon', 'test-result', '--test-result-base', results / package,
                              '--verbose'], env=env, cwd=WS, capture_output=True, text=True)
    with open(log, 'a') as out:
        out.write('\n== colcon test-result (this run, this package)\n' + verbose.stdout + verbose.stderr)
    if rc:
        status = 'FAIL'
    elif not files:
        status = 'NOT_RUN'
    else:
        status = 'PASS'
    summary.add(f'pkg-test:{package}', status, rc, sec, log, f'{len(files)} result file(s) in {results}')


def step_simtools(run, summary):
    log = run / 'simtools_owned_procs.log'
    rc, sec = axrun.run_step(['bash', axrun.HERE / 'test_owned_procs.sh'], log)
    summary.add('simtools:owned_procs', 'PASS' if rc == 0 else 'FAIL', rc, sec, log)
    step_unit(run, summary, None, str(axrun.HERE / 'tests'), 'simtools:unit')


# Main ---------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('kinds', nargs='*', default=['auto'])
    parser.add_argument('--base')
    parser.add_argument('--isolated-build', action='store_true')
    parser.add_argument('--sim-model', choices=sorted(ax_sim.ROBOTS), default='both')
    parser.add_argument('--sim-timeout', type=float, default=120.0)
    args = parser.parse_args()

    selected, packages = set(), set()
    for kind in args.kinds:
        if kind.startswith('pkg='):
            packages |= {p for p in kind[4:].split(',') if p}
        elif kind in KINDS or kind == 'auto':
            selected.add(kind)
        else:
            parser.print_usage(sys.stderr)
            print(f'unknown kind: {kind}', file=sys.stderr)
            return 3
    run_id, run = axrun.new_run_dir('verify')
    summary = Summary(run)
    changes = None
    if 'auto' in selected:
        selected.discard('auto')
        changes = collect_changes(args.base)
        rows = []
        for status, path in changes:
            kinds = classify(path)
            rows.append(f"{status}\t{path}\t{','.join(sorted(kinds)) or 'UNCLASSIFIED'}\n")
            selected |= {k for k in kinds if k in KINDS}
            packages |= {k[4:] for k in kinds if k.startswith('pkg:')}
            if not kinds:
                summary.add(f'auto:unclassified:{path}', 'INFO', note=f'{status}; no check selected')
            if 'suggest:comm-smoke' in kinds and 'comm-smoke' not in args.kinds:
                summary.add(f'auto:suggest:{path}', 'INFO', note='comm-smoke recommended, not run')
        (run / 'changes.tsv').write_text('status\tpath\tkinds\n' + ''.join(rows))
        summary.add('auto:collect', 'INFO', log=run / 'changes.tsv',
                    note=f'{len(changes)} path(s) incl. staged/untracked; selected '
                    f"{' '.join(sorted(selected | {'pkg=' + p for p in packages})) or 'nothing'}")
    write_json(run / 'run.json', {'run_id': run_id, 'requested': args.kinds, 'selected': sorted(selected),
                                  'packages': sorted(packages), 'git': axrun.git_state(),
                                  'isolated_build': args.isolated_build})

    if 'docs' in selected:
        step_docs(run, summary, changes)
    if 'physics' in selected:
        (run / 'physics').mkdir()
        ax_regen_physics.check(run / 'physics', summary)
    if 'simtools' in selected:
        step_simtools(run, summary)

    needs = set(packages).union(*(BUILD_NEEDS[k] for k in selected if k in BUILD_NEEDS))
    dependent = [k for k in ('model', 'unit', 'comm-smoke') if k in selected]
    dependent += [f'pkg-test:{p}' for p in sorted(packages)]
    if needs:
        install, env = step_build(run, summary, needs, args.isolated_build)
        if install is None:
            for step in dependent:
                summary.add(step, 'BLOCKED', note='build/env of this run did not pass; '
                            'not run against an earlier install')
        else:
            if 'model' in selected:
                step_model(run, summary, env)
            if 'unit' in selected:
                step_unit(run, summary, env)
            for package in sorted(packages):
                step_pkg_test(run, summary, env, install, package, args.isolated_build)
            if 'comm-smoke' in selected:
                (run / 'sim').mkdir()
                ax_sim.smoke(args.sim_model, args.sim_timeout, install_base=install,
                             run=(f'{run_id}_sim', run / 'sim'), summary=summary)
    code = summary.exit_code()
    verdict = {0: 'PASS', 1: 'FAIL', 2: 'INCOMPLETE'}[code]
    print(f'result {verdict} (exit {code}); summary {summary.path}')
    if 'comm-smoke' in selected:
        print('comm-smoke = basic communication smoke: reception only; controller activation, '
              'tracking and stopping are not verified')
    return code


if __name__ == '__main__':
    sys.exit(main())
