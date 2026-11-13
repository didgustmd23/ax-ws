#!/usr/bin/env python3
"""Audit source syntax, ROS package placement, local links and identical files.

Read tracked/untracked non-ignored files only. Historical evidence and pinned
upstream snapshots are reported separately; this tool never moves or deletes files.
"""
import argparse
import ast
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import re
import subprocess
import urllib.parse
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:
    yaml = None


def audit(root):
    listing = subprocess.check_output(['git', '-C', str(root), 'ls-files', '-co', '--exclude-standard', '-z'])
    paths = sorted({raw.decode('utf-8') for raw in listing.split(b'\0') if raw})
    paths = [name for name in paths if (root / name).is_file()]
    errors, archived_parse_errors, archived_links = [], [], []
    duplicates, manifests = defaultdict(list), defaultdict(list)
    for name in paths:
        path = root / name
        data = path.read_bytes()
        if data:
            duplicates[(len(data), hashlib.sha256(data).hexdigest())].append(name)
        try:
            if path.suffix == '.py':
                ast.parse(data.decode('utf-8-sig'), filename=name)
            elif path.suffix == '.json':
                def reject_nonfinite(value):
                    raise ValueError('Non-standard JSON numeric constant: ' + value)
                json.loads(data.decode('utf-8-sig'), parse_constant=reject_nonfinite)
            elif path.suffix in ('.xml', '.xacro', '.urdf', '.sdf', '.world'):
                tree = ET.fromstring(data)
                if path.name == 'package.xml' and name.startswith('src/'):
                    package = tree.findtext('name')
                    manifests[package].append(name)
                    expected = 'src/' + package + '/package.xml'
                    if name != expected:
                        errors.append({'kind': 'package_location', 'file': name, 'expected': expected})
            elif path.suffix in ('.yaml', '.yml') and yaml is not None:
                # BaseLoader validates YAML syntax without interpreting workflow "on" or ROS tags.
                yaml.load(data.decode('utf-8-sig'), Loader=yaml.BaseLoader)
        except Exception as exc:
            row = {'kind': 'parse_error', 'file': name, 'error': str(exc)}
            if name.startswith('docs/evidence/'):
                archived_parse_errors.append(row)
            else:
                errors.append(row)
        if path.suffix != '.md':
            continue
        for target in re.findall(r'!?\[[^\]\n]+\]\(([^\)\n]+)\)', data.decode('utf-8-sig')):
            target = urllib.parse.unquote(target.split(' "')[0].strip('<>').split('#')[0])
            if not target or re.match(r'[a-zA-Z]+:', target) or target.startswith('/'):
                continue
            if not (path.parent / target).exists():
                row = {'kind': 'missing_local_link', 'file': name, 'target': target}
                if name.startswith(('docs/evidence/', 'world_assets/aws_small_warehouse/upstream/')):
                    archived_links.append(row)
                else:
                    errors.append(row)
    for package, names in manifests.items():
        if len(names) != 1:
            errors.append({'kind': 'duplicate_ros_package', 'package': package, 'files': names})
    groups = []
    for (size, digest), names in duplicates.items():
        if len(names) < 2:
            continue
        if all(Path(name).name == 'LICENSE' for name in names):
            category = 'license_copies'
        elif all(name.startswith('docs/evidence/') for name in names):
            category = 'evidence_history'
        elif all(name.startswith(('src/ax_description/meshes/', 'src/robotiq_description/',
                                  'src/epick_description/', 'world_assets/')) for name in names):
            category = 'model_or_upstream_assets'
        else:
            category = 'review_required'
        groups.append({'bytes': size, 'sha256': digest, 'files': names, 'category': category})
    return {'passed': not errors, 'file_count': len(paths), 'ros_package_count': len(manifests),
            'yaml_checked': yaml is not None, 'errors': errors,
            'historical_evidence_parse_errors': archived_parse_errors,
            'historical_or_upstream_missing_links': archived_links, 'duplicate_groups': groups}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.root.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    summary = {key: result[key] for key in ('passed', 'file_count', 'ros_package_count', 'yaml_checked')}
    summary.update(passed_scope='active_sources_and_package_layout',
                   historical_parse_errors=len(result['historical_evidence_parse_errors']),
                   historical_or_upstream_missing_links=len(result['historical_or_upstream_missing_links']),
                   duplicate_groups=len(result['duplicate_groups']))
    print(json.dumps(summary))
    if result['errors']:
        print(json.dumps(result['errors'], ensure_ascii=True, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
