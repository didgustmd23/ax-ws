#!/usr/bin/env python3
# 원본 CAD 표면을 유지하며 링크 로컬 좌표로 변환한다. 입력·출력 OBJ는 mm, 설정의 이동량은 m, 회전은 rad다.
"""Reproducible joint-local CAD assets; originals and face topology stay intact."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'src/ax_description/meshes/parts/AX_1'
OUTPUT = ROOT / 'src/ax_description/meshes/versions/AX_1_v2'

# 자산 파일의 실제 바이트 SHA-256을 기록한다. 재질 줄바꿈도 보존 대상이다.
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

# 설정된 원점·회전을 정점과 법선에 반영하고 원본/출력 해시·경계·관절 계약을 manifest에 기록한다.
def generate(output):
    specification = ROOT / 'src/ax_description/config/ax1_asset_v2.yaml'
    limits = ROOT / 'src/ax_description/config/arm_joint_limits.yaml'
    settings = yaml.safe_load(specification.read_text())
    output.mkdir(parents=True, exist_ok=True)
    manifest = {'version': settings['version'], 'units': 'mm', 'policy': settings['policy'],
                'specification_sha256': digest(specification), 'joint_limits_sha256': digest(limits), 'parts': []}
    robot = ET.parse(ROOT / 'src/ax_description/urdf/ax.urdf.xacro').getroot()
    joint_frames = {}
    for joint in robot.findall('joint'):
        child = joint.find('child')
        if child is not None and child.get('link') in settings['parts']:
            joint_frames[joint.get('name')] = {'parent': joint.find('parent').get('link'),
                'child': child.get('link'), 'origin': joint.find('origin').attrib if joint.find('origin') is not None else {},
                'axis': joint.find('axis').attrib if joint.find('axis') is not None else {}}
    manifest['joint_frames'] = joint_frames
    manifest['joint_frames_sha256'] = hashlib.sha256(json.dumps(joint_frames, sort_keys=True).encode()).hexdigest()
    materials = set()
    for link, item in settings['parts'].items():
        source = SOURCE / item['source']
        rotation = Rotation.from_euler('xyz', item['rpy']).as_matrix()
        translation = np.array(item['xyz']) * 1000
        lines, before, after = [], [], []
        for line in source.read_text().splitlines():
            if line.startswith('v '):
                point = np.array(list(map(float, line.split()[1:4])))
                local = rotation @ point + translation
                before.append(point)
                after.append(local)
                line = 'v ' + ' '.join(f'{value:.10f}' for value in local)
            elif line.startswith('vn '):
                normal = rotation @ np.array(list(map(float, line.split()[1:4])))
                line = 'vn ' + ' '.join(f'{value:.12g}' for value in normal)
            elif line.startswith('mtllib '):
                materials.update(line.split()[1:])
            lines.append(line)
        destination = output / (link + '.obj')
        destination.write_text('\n'.join(lines) + '\n')
        before, after = np.array(before), np.array(after)
        reconstructed = (after - translation) @ rotation
        error = float(np.max(np.abs(before - reconstructed)))
        if error > 1e-8:
            raise ValueError('Geometry roundtrip failed: ' + link)
        manifest['parts'].append({'link': link, 'source': item['source'], 'source_sha256': digest(source),
            'file': destination.name, 'sha256': digest(destination), 'visual_xyz_m': [0, 0, 0],
            'vertices': len(after), 'faces': sum(line.startswith('f ') for line in lines),
            'bounds_local_mm': [after.min(0).tolist(), after.max(0).tolist()], 'roundtrip_error_mm': error})
    for material in sorted(materials):
        shutil.copyfile(SOURCE / material, output / material)
    shutil.copyfile(SOURCE / 'column_split_manifest.json', output / 'column_split_manifest.json')
    manifest['materials_sha256'] = {name: digest(output / name) for name in sorted(materials)}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Generated', len(manifest['parts']), 'joint-local assets')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.check:
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory)
            generate(candidate)
            failures = [path.name for path in candidate.iterdir() if not (OUTPUT / path.name).exists()
                        or path.read_bytes() != (OUTPUT / path.name).read_bytes()]
            if failures:
                raise SystemExit('Stale v2 assets: ' + ', '.join(failures))
        print('V2 assets match original CAD, pivots and joint limits')
    else:
        generate(OUTPUT)
