#!/usr/bin/env python3
"""물리 특성 생성 파일을 비교(check)하거나 명시적으로 다시 생성(regen)한다.

  ax_regen_physics.py check   사본 트리에서 생성해 작업 트리와 비교한다(쓰기 없음)
  ax_regen_physics.py regen   생성 파일 5개가 모두 만들어진 경우에만 작업 트리 파일을 교체한다

tools/generate_physical_properties.py에는 출력 경로 옵션이 없고 모든 경로가 스크립트의
ROOT = parents[1] 기준으로 고정되어 있다. 그래서 사본 트리 안에서 스크립트 사본을 실행하면
사본에 쓴다. 사본 트리는 생성기가 읽기만 하는 입력 링크로 실제 메시를 참조한다. 링크가 원본을
쓰기로부터 보호하지는 않으므로, 실행 전후에 메시 크기·수정 시각과 작업 트리 출력을 비교해 지킨다.
"""
import argparse
import difflib
import hashlib
import os
import shutil
import sys

import axrun
from axrun import WS, write_json

GENERATOR = WS / 'tools/generate_physical_properties.py'
DESCRIPTION = 'src/ax_description'
OUTPUTS = (
    f'{DESCRIPTION}/urdf/physical_properties.xacro',
    f'{DESCRIPTION}/config/physical_properties.json',
    f'{DESCRIPTION}/urdf/ax_2_physical_properties.xacro',
    f'{DESCRIPTION}/config/ax_2_physical_properties.json',
    f'{DESCRIPTION}/urdf/ax_2_geometry.xacro',
)


# 파일의 SHA-256 해시를 돌려준다. 파일이 없으면 None.
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


# 폴더 아래 모든 파일의 (상대 경로, 크기, 수정 시각 ns)를 모아 쓰기 여부를 싸게 확인한다.
def tree_stamp(folder):
    return sorted((str(p.relative_to(folder)), p.stat().st_size, p.stat().st_mtime_ns)
                  for p in folder.rglob('*') if p.is_file())


# work/tree 사본 트리에서 생성기 사본을 실행하고, 원본 메시·작업 트리 출력의 변경과 출력 누락·
# 예상 밖 파일을 확인해 (종료 코드, 로그, 걸린 시간, 문제 목록)을 돌려준다.
def generate_copy(work):
    # 사본 트리를 만들고 생성기 사본과 메시 링크를 둔다.
    tree = work / 'tree'
    (tree / 'tools').mkdir(parents=True)
    for sub in ('urdf', 'config'):
        (tree / DESCRIPTION / sub).mkdir(parents=True)
    shutil.copy2(GENERATOR, tree / 'tools' / GENERATOR.name)
    (tree / DESCRIPTION / 'meshes').symlink_to(WS / DESCRIPTION / 'meshes')   # 생성기가 읽기만 함

    # 실행 전 원본 상태를 기록하고 사본에서 생성기를 실행한다.
    meshes_before = tree_stamp(WS / DESCRIPTION / 'meshes')
    outputs_before = {o: digest(WS / o) for o in OUTPUTS}
    log = work / 'generate.log'
    rc, sec = axrun.run_step([sys.executable, tree / 'tools' / GENERATOR.name], log, cwd=tree)

    # 원본 변경, 출력 누락, 예상 밖 파일을 문제로 모은다.
    problems = []
    if tree_stamp(WS / DESCRIPTION / 'meshes') != meshes_before:
        problems.append('source meshes changed during generation')
    changed = [o for o in OUTPUTS if digest(WS / o) != outputs_before[o]]
    if changed:
        problems.append(f'working-tree outputs changed during generation: {changed}')

    produced = sorted(str(p.relative_to(tree)) for p in (tree / DESCRIPTION).rglob('*')
                      if p.is_file() and 'meshes' not in p.relative_to(tree).parts)
    missing = [o for o in OUTPUTS if not (tree / o).is_file() or (tree / o).stat().st_size == 0]
    if missing:
        problems.append(f'outputs not generated: {missing}')
    extra = sorted(set(produced) - set(OUTPUTS))
    if extra:
        problems.append(f'unexpected generated files: {extra}')
    return rc, log, sec, problems


# 사본 출력과 작업 트리를 바이트 단위로 비교해 (다른 파일, 줄끝만 다른 파일)을 돌려주고 diff를 남긴다.
def compare(work):
    tree, diffs, eol_only, lines = work / 'tree', [], [], []
    for out in OUTPUTS:
        new, old = tree / out, WS / out
        if digest(new) == digest(old):
            continue
        diffs.append(out)
        old_bytes = old.read_bytes() if old.exists() else b''
        if old_bytes.replace(b'\r\n', b'\n') == new.read_bytes().replace(b'\r\n', b'\n'):
            eol_only.append(out)
            lines.append(f'# {out}: same content, line endings differ (CRLF/LF)\n')
            continue
        lines += difflib.unified_diff(old_bytes.decode(errors='replace').splitlines(keepends=True),
                                      new.read_text().splitlines(keepends=True),
                                      f'a/{out}', f'b/{out}')

    (work / 'diff.txt').write_text(''.join(lines))
    return diffs, eol_only


# 사본 트리에서 생성해 비교한다. 차이는 check에서는 FAIL, 명시적 regen 직전에는 예상된 입력이므로
# INFO로 기록한다.
def check(work, summary, diff_status='FAIL'):
    rc, log, sec, problems = generate_copy(work)
    if rc or problems:
        summary.add('physics:generate', 'FAIL', rc, sec, log,
                    '; '.join(problems) or 'generator failed')
        summary.add('physics:compare', 'BLOCKED', note='generation in the copy tree failed')
        return None

    summary.add('physics:generate', 'PASS', rc, sec, log, 'copy tree only, working tree untouched')
    diffs, eol_only = compare(work)
    content = [d for d in diffs if d not in eol_only]
    note = 'all 5 generated files match the generator byte for byte'
    if diffs:
        note = (f'content differs: {content}; ' if content else '') + \
               (f'line endings only (CRLF in working tree): {eol_only}; ' if eol_only else '') + \
               ('regenerate explicitly with ax_regen_physics.py regen' if diff_status == 'FAIL'
                else 'to be replaced by regen')
    summary.add('physics:compare', diff_status if diffs else 'PASS', log=work / 'diff.txt',
                note=note)
    return diffs


# 사본 트리 생성이 완전할 때만 작업 트리 출력을 전부 교체한다(전부 또는 없음). 교체 전에 백업하고,
# 실패하면 이미 바꾼 파일을 되돌린다.
def regen(work, summary):
    diffs = check(work, summary, diff_status='INFO')
    if diffs is None:
        summary.add('physics:regen', 'BLOCKED', note='nothing was replaced')
        return
    if not diffs:
        summary.add('physics:regen', 'PASS', note='already up to date, nothing replaced')
        return

    backup, staged, done = work / 'backup', {}, []
    record = {'replaced': diffs, 'backup': str(backup), 'existed': {}}
    try:
        for out in diffs:                                  # 어떤 변경보다 먼저 모든 파일을 백업
            target = WS / out
            record['existed'][out] = target.exists()
            if target.exists():
                (backup / out).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(target, backup / out)

        for out in diffs:                                  # 대상 옆(같은 파일 시스템)에 임시 사본 준비
            tmp = (WS / out).with_name((WS / out).name + '.axregen.tmp')
            staged[out] = tmp                              # 쓰기 전에 등록: 일부만 쓴
            shutil.copy2(work / 'tree' / out, tmp)         # 사본이나 fsync 실패도 정리됨
            with open(tmp, 'rb') as handle:
                os.fsync(handle.fileno())

        for out in diffs:
            os.replace(staged[out], WS / out)
            del staged[out]
            done.append(out)
    except OSError as error:
        for tmp in staged.values():
            tmp.unlink(missing_ok=True)

        restore_errors = []
        for out in done:                                   # 이미 교체한 파일 되돌리기
            try:
                if record['existed'][out]:
                    shutil.copy2(backup / out, WS / out)
                else:
                    (WS / out).unlink()
            except OSError as restore_error:
                restore_errors.append(f'{out}: {restore_error}')

        record.update(result='RESTORED' if not restore_errors else 'RESTORE_FAILED',
                      error=str(error), restore_errors=restore_errors)
        write_json(work / 'regen.json', record)
        summary.add('physics:regen', 'FAIL',
                    note=f"{record['result']} after {error}; backup {backup}")
        return

    # 교체 후 작업 트리 파일이 생성기 출력과 같은지 다시 확인한다.
    mismatched = [o for o in OUTPUTS if digest(WS / o) != digest(work / 'tree' / o)]
    record.update(result='REPLACED', mismatched_after=mismatched)
    write_json(work / 'regen.json', record)
    summary.add('physics:regen', 'PASS', log=work / 'diff.txt',
                note=f'replaced {len(diffs)} file(s); previous versions in {backup}')
    summary.add('physics:verify', 'FAIL' if mismatched else 'PASS', log=work / 'regen.json',
                note=f'differ from generator output after replace: {mismatched}' if mismatched
                else 'all 5 working-tree files match the generator byte for byte')


# check 또는 regen을 실행 폴더에서 수행하고 종료 코드를 돌려준다.
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('check', 'regen'))
    args = parser.parse_args()
    _, run_dir = axrun.new_run_dir(f'physics-{args.action}')
    summary = axrun.Summary(run_dir)
    (check if args.action == 'check' else regen)(run_dir, summary)
    print(f'summary: {summary.path}')
    return summary.exit_code()


if __name__ == '__main__':
    sys.exit(main())
