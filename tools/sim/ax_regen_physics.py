#!/usr/bin/env python3
"""Generated physical-property files: compare (default) or explicitly regenerate.

  ax_regen_physics.py check   generate in a copy tree and compare with the working tree (no writes)
  ax_regen_physics.py regen   replace the working-tree files, only after all five outputs exist

tools/generate_physical_properties.py has no output option: every path is fixed relative to
ROOT = parents[1] of the script. Running a copy of the script inside a copy tree therefore writes
into the copy. The copy tree holds the real meshes through an input link that the generator only
reads; a link does not protect the source from writes, so mesh size/mtime and the working-tree
outputs are compared before and after as a guard.
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


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def tree_stamp(folder):
    """(relative path, size, mtime_ns) of every file below `folder`: cheap write detection."""
    return sorted((str(p.relative_to(folder)), p.stat().st_size, p.stat().st_mtime_ns)
                  for p in folder.rglob('*') if p.is_file())


def generate_copy(work):
    """Run a copy of the generator in `work/tree`. Returns (exit code, log, list of problems)."""
    tree = work / 'tree'
    (tree / 'tools').mkdir(parents=True)
    for sub in ('urdf', 'config'):
        (tree / DESCRIPTION / sub).mkdir(parents=True)
    shutil.copy2(GENERATOR, tree / 'tools' / GENERATOR.name)
    (tree / DESCRIPTION / 'meshes').symlink_to(WS / DESCRIPTION / 'meshes')   # read by generator
    meshes_before = tree_stamp(WS / DESCRIPTION / 'meshes')
    outputs_before = {o: digest(WS / o) for o in OUTPUTS}
    log = work / 'generate.log'
    rc, sec = axrun.run_step([sys.executable, tree / 'tools' / GENERATOR.name], log, cwd=tree)
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


def compare(work):
    """Byte comparison of copy outputs vs working tree; returns (differing, line-ending-only)."""
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
                                      new.read_text().splitlines(keepends=True), f'a/{out}', f'b/{out}')
    (work / 'diff.txt').write_text(''.join(lines))
    return diffs, eol_only


def check(work, summary, diff_status='FAIL'):
    """Generate in the copy tree and compare. A difference is FAIL for a check and INFO before an
    explicit regen (where it is the expected input, not a result)."""
    rc, log, sec, problems = generate_copy(work)
    if rc or problems:
        summary.add('physics:generate', 'FAIL', rc, sec, log, '; '.join(problems) or 'generator failed')
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
    summary.add('physics:compare', diff_status if diffs else 'PASS', log=work / 'diff.txt', note=note)
    return diffs


def regen(work, summary):
    """Replace the working-tree outputs from a complete copy-tree generation, all or nothing."""
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
        for out in diffs:                                  # back up every file before any change
            target = WS / out
            record['existed'][out] = target.exists()
            if target.exists():
                (backup / out).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(target, backup / out)
        for out in diffs:                                  # stage next to the target (same filesystem)
            tmp = (WS / out).with_name((WS / out).name + '.axregen.tmp')
            staged[out] = tmp                              # registered before writing: a partial
            shutil.copy2(work / 'tree' / out, tmp)         # copy or fsync failure is cleaned up too
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
        for out in done:                                   # undo replacements already made
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
        summary.add('physics:regen', 'FAIL', note=f"{record['result']} after {error}; backup {backup}")
        return
    mismatched = [o for o in OUTPUTS if digest(WS / o) != digest(work / 'tree' / o)]
    record.update(result='REPLACED', mismatched_after=mismatched)
    write_json(work / 'regen.json', record)
    summary.add('physics:regen', 'PASS', log=work / 'diff.txt',
                note=f'replaced {len(diffs)} file(s); previous versions in {backup}')
    summary.add('physics:verify', 'FAIL' if mismatched else 'PASS', log=work / 'regen.json',
                note=f'differ from generator output after replace: {mismatched}' if mismatched
                else 'all 5 working-tree files match the generator byte for byte')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('check', 'regen'))
    args = parser.parse_args()
    _, run_dir = axrun.new_run_dir(f'physics-{args.action}')
    summary = axrun.Summary(run_dir)
    (check if args.action == 'check' else regen)(run_dir, summary)
    print(f'summary: {summary.path}')
    return summary.exit_code()


if __name__ == '__main__':
    sys.exit(main())
