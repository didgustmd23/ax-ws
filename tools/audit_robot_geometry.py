#!/usr/bin/env python3
# 두 로봇의 중립 자세에서 링크별 질량·전체 COM·메시 포함 여부·SDF 관절 보존을 검사한다.
"""Independent COM, mesh containment and converted articulation audit."""
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import xacro
from ament_index_python.packages import get_package_share_directory

WORKSPACE = Path(__file__).resolve().parents[1]

ARM_JOINTS = ['column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint']


# 링크 원점의 xyz/rpy를 변환 행렬로 바꿔 중립 자세의 질량 위치를 합산한다.
def transform(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return result


# 설치된 모델을 펼쳐 COM·관성·메시 포함·SDF 변환을 검사하고 log/repair에 수치 결과를 기록한다.
def main():
    share = Path(get_package_share_directory('ax_description'))
    output = WORKSPACE / 'log/repair/model_geometry.json'
    report = {'scope': 'virtual simulation model; not measured hardware mass', 'models': {}}
    for filename in ('ax.urdf.xacro', 'ax_2.urdf.xacro'):
        text = xacro.process_file(str(share / 'urdf' / filename), mappings={'gripper_model': 'epick'}).toxml()
        robot = ET.fromstring(text)
        frames = {'base_link': np.eye(4)}
        pending = list(robot.findall('joint'))
        while pending:
            before = len(pending)
            for joint in pending[:]:
                parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
                if parent in frames:
                    frames[child] = frames[parent] @ transform(joint.find('origin'))
                    pending.remove(joint)
            if len(pending) == before:
                raise RuntimeError('Disconnected joint graph')
        weighted, total, link_com = np.zeros(3), 0.0, {}
        for link in robot.findall('link'):
            inertial = link.find('inertial')
            if inertial is None:
                continue
            mass = float(inertial.find('mass').get('value'))
            com = (frames[link.get('name')] @ transform(inertial.find('origin')))[:3, 3]
            weighted += mass * com
            total += mass
            link_com[link.get('name')] = {'mass_kg': mass, 'com_in_base_m': com.tolist()}
        base = robot.find("link[@name='base_link']")
        visual = base.find('visual')
        mesh = visual.find('geometry/mesh')
        mesh_file = share / mesh.get('filename').split('ax_description/', 1)[1]
        vertices = np.array([[float(value) for value in line.split()[1:4]]
                             for line in mesh_file.read_text().splitlines() if line.startswith('v ')])
        vertices *= np.fromstring(mesh.get('scale'), sep=' ')
        visual_transform = transform(visual.find('origin'))
        vertices = vertices @ visual_transform[:3, :3].T + visual_transform[:3, 3]
        covered = np.zeros(len(vertices), dtype=bool)
        for collision in base.findall('collision'):
            box = collision.find('geometry/box')
            collision_transform = transform(collision.find('origin'))
            points = (vertices - collision_transform[:3, 3]) @ collision_transform[:3, :3]
            covered |= np.all(np.abs(points) <= np.fromstring(box.get('size'), sep=' ') / 2 + 1e-8, axis=1)
        with tempfile.TemporaryDirectory(prefix='ax-geometry-audit-') as directory:
            urdf_file = Path(directory) / 'robot.urdf'
            urdf_file.write_text(text)
            converted = subprocess.run(['gz', 'sdf', '-p', str(urdf_file)], check=True, capture_output=True, text=True)
            sdf = ET.fromstring(converted.stdout)
        sdf_base = sdf.find(".//link[@name='base_link']")
        data = {'total_mass_kg': total, 'com_in_base_m': (weighted / total).tolist(),
                'link_com': link_com, 'body_collision_count': len(base.findall('collision')),
                'body_mesh_extent_m': np.ptp(vertices, axis=0).tolist(),
                'body_mesh_vertices': len(vertices), 'uncovered_body_vertices': int((~covered).sum()),
                'sdf_base_mass_kg': float(sdf_base.findtext('inertial/mass')),
                'sdf_base_inertial_pose': sdf_base.findtext('inertial/pose'),
                'sdf_arm_joints': [joint.get('name') for joint in sdf.findall('.//joint')
                                   if joint.get('name') in ARM_JOINTS]}
        data['passed'] = bool(covered.all())
        if filename == 'ax.urdf.xacro':
            data['passed'] = data['passed'] and set(data['sdf_arm_joints']) == set(ARM_JOINTS)
            data['passed'] = data['passed'] and all(link_com[name]['com_in_base_m'][2] > .5
                for name in ('upper_arm_link', 'forearm_link', 'wrist_link'))
        report['models'][filename] = data
    report['passed'] = all(data['passed'] for data in report['models'].values())
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({name: {key: data[key] for key in ('total_mass_kg', 'com_in_base_m',
        'body_collision_count', 'uncovered_body_vertices', 'sdf_arm_joints', 'passed')}
        for name, data in report['models'].items()}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
