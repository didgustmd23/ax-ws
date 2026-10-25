#!/usr/bin/env python3
"""ax-ws 변경 유형별 검증을 실행하고 실행마다 summary.tsv 하나를 남긴다.

  ax_verify.py [auto] [docs] [model] [unit] [physics] [simtools] [comm-smoke] [pkg=NAME,...]
               [--base REF] [--isolated-build] [--sim-model ax1|ax2|both] [--sim-timeout S]

auto   staged·unstaged·untracked 변경(이름 변경은 두 경로, 삭제는 옛 경로)과, --base를 주면 REF
       이후 커밋에서 종류를 고른다. 분류하지 못한 경로는 INFO로 보고한다.
docs   Markdown 파일의 상대 링크·이미지 경로가 있는지 확인(앵커는 검사하지 않음)
model  이번 실행 빌드로 ax.urdf.xacro(집게 4종)와 ax_2.urdf.xacro의 xacro + check_urdf
unit   이번 실행 빌드로 python3 -m unittest discover tools/tests
physics 사본 트리에서 생성기를 실행해 작업 트리와 비교(작업 트리에 쓰지 않음. 파일 교체는
       ax_regen_physics.py regen으로 명시적으로 함)
simtools tools/sim 회귀 시험(소유 프로세스 도우미와 이 도구들)
comm-smoke 기본 통신 smoke: 메시지 수신만 확인하며 제어기 활성·추종·정지는 확인하지 않음
       (auto가 선택하지 않음)
pkg=   패키지 colcon build + colcon test, 결과는 이번 실행의 결과 폴더로 한정

작업공간 빌드가 필요한 단계(model, unit, pkg, comm-smoke)는 같은 실행의 colcon build에 의존한다.
빌드가 실패하면 BLOCKED이며 이전 install로 대신 실행하지 않는다. 종료 코드는 axrun.py 참고
(0 모두 PASS, 1 FAIL, 2 미완료, 3 사용법 오류). 전체 로그는 실행 폴더에 남는다.
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
    'unit': {'ax_description', 'robotiq_description', 'ax_control'},   # 시험이 ax_control을 import함
    'comm-smoke': {'ax_description', 'robotiq_description', 'ax_control'},
}
GRIPPERS = ('primitive', 'robotiq', 'parallel_wide', 'suction')


# 변경 수집 ---------------------------------------------------------------------------------

# 작업공간에서 git 명령을 실행해 표준 출력을 돌려준다(실패하면 예외).
def _git(*args):
    return subprocess.run(['git', *args], cwd=WS, capture_output=True, check=True).stdout.decode()


# staged·unstaged·untracked 변경과(base를 주면) base 이후 커밋 변경을 [(상태, 경로)]로 모은다.
# 이름 변경은 새 경로와 원래 경로를 모두 넣는다.
def collect_changes(base=None):
    changes = []

    # 작업 트리 변경: porcelain v2 항목을 NUL로 나눠 종류별로 읽는다.
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
        elif entry[0] == '2':                       # 이름 변경·복사: 새 경로 다음 원래 경로
            parts = entry.split(' ', 9)
            changes.append(('renamed-to', parts[9]))
            changes.append(('renamed-from', fields[i]))
            i += 1
        elif entry[0] == 'u':
            changes.append(('unmerged', entry.split(' ', 10)[10]))

    # base 이후 커밋 변경: name-status 항목을 읽는다(이름 변경·복사는 경로 두 개).
    if base:
        fields = _git('diff', '-z', '--name-status', '--find-renames',
                      f'{base}...HEAD').split('\0')
        i = 0
        while i < len(fields) - 1:
            status = fields[i]
            if status[:1] in 'RC':
                changes += [('committed-renamed-from', fields[i + 1]),
                            ('committed-renamed-to', fields[i + 2])]
                i += 3
            else:
                kind = {'A': 'added', 'D': 'deleted', 'M': 'modified'}.get(status[:1], status)
                changes.append(('committed-' + kind, fields[i + 1]))
                i += 2

    # 순서를 유지하며 중복을 뺀다.
    seen, unique = set(), []
    for item in changes:
        if item not in seen:
            seen.add(item)
            unique.append(item)
    return unique


# git status의 XY 상태 문자를 deleted·staged·unstaged 표기로 바꾼다.
def _label(xy):
    if 'D' in xy:
        return 'deleted'
    if xy[0] not in '.':
        return 'staged' + ('+unstaged' if xy[1] != '.' else '')
    return 'unstaged'


# 변경 경로 하나로 실행할 검증 종류를 정한다. 빈 집합이면 분류하지 못한 경로다.
def classify(path):
    kinds = set()
    if path.startswith('src/ax_description/'):
        kinds |= {'model', 'unit', 'pkg:ax_description'}      # config/sensor_mounts.yaml 포함
        if path.startswith('src/ax_description/meshes/parts/') or path in ax_regen_physics.OUTPUTS:
            kinds.add('physics')
    elif path.startswith('src/robotiq_description/'):
        kinds.add('model')                                     # 외부 모델 선별본, 시험 없음
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


# 단계 --------------------------------------------------------------------------------------

# 링크를 검사할 Markdown 파일 목록을 정한다. 변경 목록이 없거나 Markdown이 삭제·이름 변경되면
# 저장소 전체를 본다.
def markdown_files(changes=None):
    if changes is None or any(s.endswith(('deleted', 'renamed-from')) and p.endswith('.md')
                              for s, p in changes):
        listed = _git('ls-files', '-z', '--cached', '--others', '--exclude-standard', '*.md')
        return sorted({p for p in listed.split('\0') if p and (WS / p).exists()})
    return sorted({p for s, p in changes if p.endswith('.md') and (WS / p).exists()})


LINK = re.compile(r'!?\[[^\]]*\]\(\s*<?([^)\s>]+)>?(?:\s+"[^"]*")?\s*\)')


# Markdown 파일에서 코드 블록을 뺀 상대 링크·이미지 경로 중 없는 대상을 찾는다(앵커 제외).
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


# 문서 링크 검사 단계를 실행하고 결과를 기록한다.
def step_docs(run, summary, changes):
    files = markdown_files(changes)
    log = run / 'docs.log'
    problems = {f: b for f in files if (b := broken_links(f))}
    log.write_text(f'checked {len(files)} files (anchors not checked)\n' + ''.join(
        f'{f}: {t}\n' for f, b in problems.items() for t in b))
    count = sum(map(len, problems.values()))
    summary.add('docs:links', 'FAIL' if count else 'PASS', log=log,
                note=f'{count} broken link(s) in {len(files)} file(s); anchors not checked')


# 이번 실행에서 패키지를 빌드하고, 패키지가 이 빌드의 install에서 찾아지는지 확인한다.
# 성공하면 (install 경로, 환경), 실패하면 (None, None)을 돌려준다.
def step_build(run, summary, packages, isolated):
    base = ['--build-base', run / 'build', '--install-base', run / 'install'] if isolated else []
    install = run / 'install' if isolated else WS / 'install'
    env = axrun.ros_env()                     # ROS만 source: 이전 작업공간 install은 쓰지 않음
    rc, sec = axrun.run_step(['colcon', '--log-base', run / 'colcon_log', 'build',
                              '--symlink-install', '--packages-up-to', *sorted(packages), *base],
                             run / 'build.log', env=env)
    summary.add('build', 'PASS' if rc == 0 else 'FAIL', rc, sec, run / 'build.log',
                f"{' '.join(sorted(packages))} -> {install}")
    if rc:
        return None, None

    # 빌드한 패키지의 prefix가 이번 install 아래인지 확인한다.
    env = axrun.ros_env(install)
    log = run / 'env.log'
    wrong = []
    with open(log, 'w') as out:
        for package in sorted(packages):
            prefix = subprocess.run(['ros2', 'pkg', 'prefix', package], env=env,
                                    capture_output=True, text=True).stdout.strip()
            out.write(f'{package}: {prefix}\n')
            if not prefix or not Path(prefix).resolve().is_relative_to(install.resolve()):
                wrong.append(package)

    summary.add('build:env', 'FAIL' if wrong else 'PASS', log=log,
                note=(f'not resolved from this build: {wrong}' if wrong
                      else f'packages from {install}'))
    return (None, None) if wrong else (install, env)


# 집게 4종 AX_1과 AX_2의 Xacro 변환과 check_urdf를 모델별로 실행한다.
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


# 지정 폴더의 unittest를 실행하고 결과를 기록한다.
def step_unit(run, summary, env, folder='tools/tests', step='unit'):
    log = run / f"{step.replace(':', '_')}.log"
    rc, sec = axrun.run_step([sys.executable, '-m', 'unittest', 'discover', '-s', folder, '-v'],
                             log, env=env)
    summary.add(step, 'PASS' if rc == 0 else 'FAIL', rc, sec, log)


# 패키지 하나의 colcon test를 이번 실행 결과 폴더로 실행하고, 결과 파일이 없으면 NOT_RUN으로 기록한다.
def step_pkg_test(run, summary, env, install, package, isolated):
    base = (['--build-base', run / 'build', '--install-base', install] if isolated else [])
    results = run / 'test_results'
    log = run / f'test_{package}.log'
    rc, sec = axrun.run_step(['colcon', '--log-base', run / 'colcon_log', 'test',
                              '--packages-select', package, '--test-result-base', results,
                              '--return-code-on-test-failure', *base], log, env=env)

    # 이번 실행·이 패키지의 결과 파일만 세고, 상세 결과를 로그 뒤에 붙인다.
    listing = subprocess.run(['colcon', 'test-result', '--test-result-base', results, '--all',
                              '--result-files-only'], env=env, cwd=WS, capture_output=True,
                             text=True)
    files = [f for f in listing.stdout.splitlines() if Path(f).resolve().is_relative_to(
        (results / package).resolve())]
    verbose = subprocess.run(['colcon', 'test-result', '--test-result-base', results / package,
                              '--verbose'], env=env, cwd=WS, capture_output=True, text=True)
    with open(log, 'a') as out:
        out.write('\n== colcon test-result (this run, this package)\n'
                  + verbose.stdout + verbose.stderr)

    if rc:
        status = 'FAIL'
    elif not files:
        status = 'NOT_RUN'
    else:
        status = 'PASS'
    summary.add(f'pkg-test:{package}', status, rc, sec, log,
                f'{len(files)} result file(s) in {results}')


# 소유 프로세스 도구 회귀 시험과 tools/sim 단위 시험을 실행한다.
def step_simtools(run, summary):
    log = run / 'simtools_owned_procs.log'
    rc, sec = axrun.run_step(['bash', axrun.HERE / 'test_owned_procs.sh'], log)
    summary.add('simtools:owned_procs', 'PASS' if rc == 0 else 'FAIL', rc, sec, log)
    step_unit(run, summary, None, str(axrun.HERE / 'tests'), 'simtools:unit')


# 주 실행 -----------------------------------------------------------------------------------

# 검증 종류를 해석하고(auto면 변경 경로로 선택) 문서·물리·도구·빌드 의존 단계를 차례로 실행한 뒤
# summary.tsv 기준 종료 코드를 돌려준다.
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
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

    # auto: 변경 경로를 분류해 검증 종류·패키지를 고르고 changes.tsv에 남긴다.
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
                summary.add(f'auto:unclassified:{path}', 'INFO',
                            note=f'{status}; no check selected')
            if 'suggest:comm-smoke' in kinds and 'comm-smoke' not in args.kinds:
                summary.add(f'auto:suggest:{path}', 'INFO',
                            note='comm-smoke recommended, not run')

        (run / 'changes.tsv').write_text('status\tpath\tkinds\n' + ''.join(rows))
        summary.add('auto:collect', 'INFO', log=run / 'changes.tsv',
                    note=f'{len(changes)} path(s) incl. staged/untracked; selected '
                    f"{' '.join(sorted(selected | {'pkg=' + p for p in packages})) or 'nothing'}")

    write_json(run / 'run.json', {'run_id': run_id, 'requested': args.kinds,
                                  'selected': sorted(selected), 'packages': sorted(packages),
                                  'git': axrun.git_state(), 'isolated_build': args.isolated_build})

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

    # 빌드 의존 단계: 이번 실행 빌드가 통과해야 실행하고, 아니면 BLOCKED로 남긴다.
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
