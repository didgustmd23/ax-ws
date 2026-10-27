#!/usr/bin/env python3
"""메시 성분을 보수적으로 감싸는 충돌 상자와 명시적 가상 질량 특성을 생성한다.

Python, numpy, scipy로 어느 폴더에서든 실행할 수 있다. 메시는 바꾸지 않는다.
질량은 가정한 가상 설계를 나타내며 실제 하드웨어 측정값이 아니다.
"""
import hashlib
import json
import itertools
import warnings
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
PARTS = ROOT / 'src/ax_description/meshes/parts/AX_1'
OUT = ROOT / 'src/ax_description/urdf/physical_properties.xacro'
XACRO = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', XACRO)
# 조립 CAD 좌표에서 원본 메시의 원점·피벗(m).
LINKS = {
    'base_link': ('ax_body', [.003, 0, .38277325], 8.),
    'arm_mount_link': ('arm_01_support', [.18, .185, .612], 3.),
    'column_link': ('arm_01_column', [.18, .185, .780], 4.),
    'arm_rotating_base_link': ('arm_02_movie', [.18, .185, 1.3553438], 1.),
    'upper_arm_link': ('arm_03_upperarm', [.2944427, .185, 1.4553438], 4.),
    'forearm_link': ('arm_04_forearm', [.4264427, .185, 2.0153438], 3.),
    'wrist_link': ('gripper_wristhousing', [.5164427, .640, 1.5753438], 1.),
}


# 숫자 목록을 URDF 속성용 공백 구분 문자열(유효숫자 12자리)로 바꾼다.
def numbers(v):
    return ' '.join(f'{x:.12g}' for x in v)


# OBJ 메시를 읽어 mm→m 변환과 피벗 보정을 하고, 겹치는 정점을 합쳐 연결 성분별 점 묶음을
# 구한다. split_large_surfaces면 큰 3차원 성분을 거의 같은 평면의 연결 면으로 더 나눈다.
def mesh_components(path, pivot, split_large_surfaces=False):
    vertices, faces = [], []

    # 정점과 면(1부터 시작, 음수는 뒤에서부터)을 0부터 시작하는 색인으로 읽는다.
    for line in path.read_text().splitlines():
        if line.startswith('v '):
            vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            face = []
            for token in line.split()[1:]:
                idx = int(token.split('/')[0])
                face.append(idx - 1 if idx > 0 else len(vertices) + idx)
            faces.append(face)
    vertices = np.array(vertices) * .001 - pivot

    # 내보낸 표면의 겹치는 정점을 합쳐 연결된 CAD 부품을 복원한다.
    unique, inverse = np.unique(np.round(vertices, 7), axis=0, return_inverse=True)
    edges = np.array([(inverse[f[0]], inverse[i]) for f in faces for i in f[1:]])
    graph = coo_matrix((np.ones(len(edges)), edges.T), shape=(len(unique), len(unique)))
    count, labels = connected_components(graph.tocsr(), directed=False)

    # 작은 성분은 그대로 쓰고, 큰 3차원 성분만 평면별로 나눈다.
    components = []
    for i in range(count):
        points = unique[labels == i]
        if not split_large_surfaces or not np.all(np.ptp(points, axis=0) > .2):
            components.append(points)
            continue

        # 연결된 적재대 프레임이 적재 공간을 가로지르는 하나의 꽉 찬 상자가 되면 안 된다.
        # 큰 3차원 부품은 상자를 맞추기 전에 연결된 거의 같은 평면의 면들로 나눈다.
        # 이 상자들도 보수적인 근사다.
        planes = {}
        for face in faces:
            indices = inverse[face]
            if labels[indices[0]] != i:
                continue
            pts = unique[indices]

            # 면 법선을 단위화하고 가장 큰 성분이 양수가 되게 부호를 맞춘 뒤 평면 키로 쓴다.
            normal = np.cross(pts[1] - pts[0], pts[2] - pts[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-14:
                raise ValueError(f'Degenerate rack face in {path}')
            normal /= norm
            if normal[np.argmax(np.abs(normal))] < 0:
                normal *= -1
            key = (*np.round(normal, 4), round(float(normal @ pts[0]), 4))
            planes.setdefault(key, []).append(indices)

        # 같은 평면의 면 묶음을 다시 연결 성분으로 나눈다.
        for group in planes.values():
            ids = np.unique(np.concatenate(group))
            edge = np.array([(np.searchsorted(ids, face[0]), np.searchsorted(ids, j))
                             for face in group for j in face[1:]])
            graph = coo_matrix((np.ones(len(edge)), edge.T), shape=(len(ids), len(ids)))
            n, sublabels = connected_components(graph.tocsr(), directed=False)
            components.extend(unique[ids[sublabels == j]] for j in range(n))
    return vertices, components


# 점 묶음을 모두 감싸는 상자를 축 정렬·PCA 방향 중 부피가 작은 쪽으로 만든다.
def box_for_points(points):
    # PCA 방향 상자가 축 정렬 상자보다 작을 때만 고른다.
    # 성분의 모든 정점(따라서 모든 삼각형)은 상자 안에 남는다.
    _, axes = np.linalg.eigh(np.cov(points.T))
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1

    # 두 후보 방향마다 감싸는 상자의 부피·중심·크기를 구한다.
    choices = []
    for rotation in (np.eye(3), axes):
        local = points @ rotation
        lo, hi = local.min(0), local.max(0)
        size = hi - lo + .002  # 면마다 1 mm 여유. 평면 장식에도 적용한다.
        center = rotation @ ((lo + hi) / 2)
        choices.append((float(np.prod(size)), center, size, rotation))

    # 부피가 작은 상자를 고르고, 모든 점이 들어가는지 확인한 뒤 rpy로 바꾼다.
    _, center, size, rotation = min(choices, key=lambda x: x[0])
    assert np.all(np.abs((points - center) @ rotation) <= size / 2 + 1e-10)
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', message='Gimbal lock detected.*')
        rpy = Rotation.from_matrix(rotation).as_euler('xyz').tolist()
    return {'shape': 'box', 'center': center.tolist(), 'size': size.tolist(), 'rpy': rpy}


# 다른 상자 안에 완전히 들어가는 중복 충돌 상자를 뺀다.
def remove_contained_boxes(boxes):
    kept = []
    signs = np.array(list(itertools.product((-1, 1), repeat=3)))
    for i, box in enumerate(boxes):
        rotation = Rotation.from_euler('xyz', box['rpy']).as_matrix()
        corners = np.array(box['center']) + (signs * np.array(box['size']) / 2) @ rotation.T

        # 더 크거나(같은 부피면 앞선) 다른 상자가 여덟 꼭짓점을 모두 감싸면 중복으로 본다.
        redundant = False
        for j, other in enumerate(boxes):
            if i == j:
                continue
            own_volume = np.prod(box['size'])
            other_volume = np.prod(other['size'])
            if (other_volume < own_volume - 1e-12
                    or (abs(other_volume-own_volume) < 1e-12 and j > i)):
                continue

            other_rotation = Rotation.from_euler('xyz', other['rpy']).as_matrix()
            if np.all(
                np.abs((corners - other['center']) @ other_rotation)
                <= np.array(other['size']) / 2 + 1e-10
            ):
                redundant = True
                break
        if not redundant:
            kept.append(box)
    return kept


# 상자 모양 가상 질량 부품 정의를 만든다.
def mass_box(mass, center, size, rpy=(0, 0, 0), label='housing'):
    return dict(shape='box', mass=mass, center=list(center), size=list(size),
                rpy=list(rpy), label=label)


# 원통(중공 가능) 모양 가상 질량 부품 정의를 만든다.
def mass_cylinder(mass, center, radius, length, rpy=(0, 0, 0), label='motor', inner=0):
    return dict(shape='cylinder', mass=mass, center=list(center), radius=radius,
                length=length, rpy=list(rpy), label=label, inner_radius=inner)


# 질량을 충돌 상자 부피에 비례해 나눠 하우징 질량 부품으로 만든다.
def distribute(mass, boxes):
    weights = np.array([np.prod(b['size']) for b in boxes])
    return [dict(b, mass=float(mass * w / weights.sum()), label='housing_volume_proxy')
            for b, w in zip(boxes, weights)]


# 링크별 가상 설계 질량 배분(모터·배터리·튜브·하우징)을 정한다. 실측·제조사 사양이 아니다.
def virtual_mass_parts(name, boxes):
    # 명시적 설계 가정이다. 모터 질량은 제조사 사양이 아니다.
    if name == 'base_link':
        return [
            mass_box(5.6, [0, 0, -.15], [.55, .35, .12], label='low_battery_ballast_budget')
        ] + distribute(2.4, boxes)
    if name == 'arm_mount_link':
        return [
            mass_box(1.8, [-.005, 0, .084], [.23, .29, .168], label='fixed_base_budget')
        ] + distribute(1.2, boxes)

    if name == 'column_link':
        # 짧아진 원본 튜브의 축 방향 길이. 질량 예산은 유지한다.
        length = json.loads((PARTS / 'column_split_manifest.json').read_text())['tube_length_m']
        return [mass_cylinder(1.6, [0, 0, .05], .08, .10),
                mass_cylinder(2.4, [0, 0, length/2], .1, length, inner=.095,
                              label='column_tube_budget')]

    # 회전 관절 링크는 축 방향 모터 원통과 하우징 부피 질량으로 나눈다.
    if name == 'arm_rotating_base_link':
        return [
            mass_cylinder(.6, [.0544427, 0, .10], .065, .06, [0, np.pi/2, 0])
        ] + distribute(.4, boxes)
    if name == 'upper_arm_link':
        return [
            mass_cylinder(1.6, [.04, 0, 0], .075, .06, [0, np.pi/2, 0])
        ] + distribute(2.4, boxes)
    if name == 'forearm_link':
        return [
            mass_cylinder(1.05, [.11, 0, 0], .075, .035, [0, np.pi/2, 0])
        ] + distribute(1.95, boxes)
    return [mass_cylinder(.45, [0, 0, -.09], .05, .08)] + distribute(.55, boxes)


# 질량 부품들을 합쳐 링크의 전체 질량, 무게중심, 무게중심 기준 관성 텐서를 계산한다.
def aggregate(parts):
    mass = sum(p['mass'] for p in parts)
    center = sum(p['mass'] * np.array(p['center']) for p in parts) / mass
    inertia = np.zeros((3, 3))

    # 부품별 자기 중심 관성을 회전한 뒤 평행축 정리로 전체 COM 기준에 더한다.
    for p in parts:
        m = p['mass']
        if p['shape'] == 'box':
            x, y, z = p['size']
            diagonal = m / 12 * np.array([y*y+z*z, x*x+z*z, x*x+y*y])
        else:
            rr = p['radius']**2 + p.get('inner_radius', 0)**2
            transverse = m / 12 * (3*rr + p['length']**2)
            diagonal = [transverse, transverse, .5*m*rr]

        rotation = Rotation.from_euler('xyz', p['rpy']).as_matrix()
        d = np.array(p['center']) - center
        inertia += (rotation @ np.diag(diagonal) @ rotation.T
                    + m * (np.dot(d, d)*np.eye(3)-np.outer(d, d)))
    return mass, center, inertia


# 로봇 하나의 충돌 근사 상자와 전체 질량 텐서를 계산해 physical_properties Xacro와 JSON을
# 같은 결과에서 생성한다.
def generate(
    links=LINKS,
    parts_dir=PARTS,
    output=OUT,
    report_name='physical_properties.json',
    macro_prefix='ax_physics_',
    surface_meshes=(),
):
    robot = ET.Element('robot')
    report = {
        'assumptions': (
            'Virtual mass budgets and volume-weighted box proxies, not measured hardware. '
            'Component boxes fill internal cavities; not convex decomposition.'
        ),
        'mesh_directory': str(parts_dir.relative_to(ROOT / 'src/ax_description')),
        'links': {},
    }

    for name, (mesh, pivot, budget) in links.items():
        source = parts_dir / (mesh + '.obj')
        split_surfaces = mesh in surface_meshes
        vertices, components = mesh_components(source, np.array(pivot), split_surfaces)

        # 선반의 연결 성분 수와 충돌 상자용 면 패치 수를 구분해 기록한다.
        raw_count = (
            len(mesh_components(source, np.array(pivot))[1])
            if split_surfaces else len(components)
        )
        boxes = remove_contained_boxes([box_for_points(c) for c in components])

        # AX_1의 명시적 가상 부품 배치와 AX_2의 독립 질량 예산을 구분한다.
        parts = (
            virtual_mass_parts(name, boxes)
            if macro_prefix == 'ax_physics_' else distribute(budget, boxes)
        )
        mass, center, inertia = aggregate(parts)
        assert abs(mass - budget) < 1e-10

        macro = ET.SubElement(robot, '{'+XACRO+'}macro', name=macro_prefix+name)
        for i, box in enumerate(boxes):
            collision = ET.SubElement(macro, 'collision', name=f'{name}_part_{i:02d}')
            ET.SubElement(
                collision, 'origin', xyz=numbers(box['center']), rpy=numbers(box['rpy']),
            )
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(geometry, 'box', size=numbers(box['size']))

        # 관성은 링크 COM 기준 전체 텐서이며 비대각 성분도 직렬화한다.
        inertial = ET.SubElement(macro, 'inertial')
        ET.SubElement(inertial, 'origin', xyz=numbers(center), rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value=f'{mass:.12g}')
        entries = (
            ('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2),
            ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2),
        )
        ET.SubElement(
            inertial, 'inertia',
            **{key: f'{inertia[i, j]:.12g}' for key, i, j in entries},
        )

        report['links'][name] = {
            'mesh': mesh + '.obj',
            'pivot_m': pivot,
            'mesh_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'mesh_bounds_link_m': [vertices.min(0).tolist(), vertices.max(0).tolist()],
            'mesh_connected_components': raw_count,

            'collision_fit_groups': len(components),
            'collision_count': len(boxes),
            'all_component_vertices_enclosed': True,
            'collision_box_volume_sum_m3': sum(float(np.prod(b['size'])) for b in boxes),
            'old_single_aabb_volume_m3': float(np.prod(np.ptp(vertices, axis=0))),

            'mass_kg': mass,
            'center_of_mass_m': center.tolist(),
            'inertia_kg_m2': inertia.tolist(),
            'mass_parts': parts,
            'collision_boxes': boxes,
        }

    # XML과 JSON은 같은 계산 결과에서 생성하며 수동 수정하지 않는다.
    ET.indent(robot, space='  ')
    output.write_text(
        '<?xml version="1.0"?>\n'
        '<!-- Generated by tools/generate_physical_properties.py; do not edit manually. -->\n'
        + ET.tostring(robot, encoding='unicode') + '\n'
    )
    destination = ROOT / 'src/ax_description/config' / report_name
    destination.write_text(json.dumps(report, indent=2) + '\n')
    print('Generated', output.relative_to(ROOT))
    for name, link in report['links'].items():
        print(
            name, 'collisions', link['collision_count'],
            'COM', np.round(link['center_of_mass_m'], 4),
        )


# AX_2의 가상 질량 예산을 따로 정하고, AX_2 메시에서는 형상 경계만 읽어 생성한다.
def generate_ax2():
    directory = PARTS.parent / 'AX_2'
    geometry = {}

    # 원본 CAD 좌표를 보존하고 계산에만 mm→m 변환을 적용한다.
    for path in sorted(directory.glob('*.obj')):
        vertices = np.array([
            [float(x) for x in line.split()[1:4]]
            for line in path.read_text().splitlines()
            if line.startswith('v ')
        ]) * .001
        lo, hi = vertices.min(0), vertices.max(0)
        geometry[path.stem] = dict(pivot=(lo+hi)/2, vertices=vertices, size=hi-lo)

    # AX_1과 독립된 가상 질량 예산이다. 실측 제품 질량이 아니다.
    budgets = (
        ('base_link', 'ax_body', 12.),
        ('shelf_link', 'ax_shelf', 5.),
        ('front_trim_link', 'ax_right', .05),
        ('top_lidar_housing_link', 'ax_top_lidar', .4),
    )
    links = {
        name: (mesh, geometry[mesh]['pivot'].tolist(), mass)
        for name, mesh, mass in budgets
    }

    # AX_2 전용 출력 파일 이름과 매크로 접두사로 생성하고, 선반은 평면별로 나눈다.
    generate(
        links,
        directory,
        OUT.with_name('ax_2_physical_properties.xacro'),
        'ax_2_physical_properties.json',
        'ax2_physics_',
        ('ax_shelf',),
    )

    # 메시 원점과 바퀴 치수를 Xacro에 전달하여 visual·관절·구동이 공유한다.
    robot = ET.Element('robot')
    for name, data in geometry.items():
        ET.SubElement(
            robot,
            '{'+XACRO+'}property',
            name=f'ax2_{name}_pivot',
            value='${' + repr(data['pivot'].tolist()) + '}',
        )

    # 바퀴 축은 CAD +Y다. X/Z 평면의 최대 반경과 Y 폭에 1 µm 여유를 둔다.
    wheels = [geometry[f'ax_tire_{i:02d}'] for i in range(1, 5)]
    radius = max(
        float(np.linalg.norm((w['vertices'] - w['pivot'])[:, [0, 2]], axis=1).max())
        for w in wheels
    ) + .000001
    width = max(float(w['size'][1]) for w in wheels) + .000001
    separation = float(wheels[1]['pivot'][1] - wheels[0]['pivot'][1])

    for name, value in (
        ('wheel_radius', radius),
        ('wheel_width', width),
        ('wheel_separation', separation),
        ('wheel_mass', .4),
    ):
        ET.SubElement(
            robot,
            '{'+XACRO+'}property',
            name='ax2_'+name,
            value=f'{value:.12g}',
        )

    ET.indent(robot, space='  ')
    OUT.with_name('ax_2_geometry.xacro').write_text(
        '<?xml version="1.0"?>\n<!-- Generated by tools/generate_physical_properties.py; '
        'CAD coordinates in meters. Masses are virtual assumptions. -->\n'
        + ET.tostring(robot, encoding='unicode') + '\n')


if __name__ == '__main__':
    generate()
    generate_ax2()
