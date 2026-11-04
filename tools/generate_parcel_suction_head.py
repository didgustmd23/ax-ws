#!/usr/bin/env python3
"""사용자 참고 사진을 바탕으로 만든 임시 다중 컵 헤드. 제품 CAD 복제나 실물 사양이 아니다."""
from pathlib import Path
import argparse
import math
import re
import struct
import yaml

ROOT = Path(__file__).resolve().parents[1]


def generate():
    settings = yaml.safe_load((ROOT / 'src/ax_description/config/epick.yaml').read_text(encoding='utf-8'))
    shape = settings['head_visual']
    out = ROOT / 'src/ax_description/meshes/tools'
    out.mkdir(parents=True, exist_ok=True)
    vertices, faces = [], []
    offset = settings['body_length_m'] / 2

    # 공구 기준 +Z가 흡착면 방향이다. 정점은 epick_body 원점에 맞춘다.
    def mesh(points, polygons, material):
        first = len(vertices)
        vertices.extend((x, y, z - offset) for x, y, z in points)
        faces.append('usemtl ' + material)
        faces.extend('f ' + ' '.join(str(first + i + 1) for i in polygon) for polygon in polygons)

    def housing(top_w, top_d, bottom_w, bottom_d, top, bottom, material):
        points = [(sx * w / 2, sy * d / 2, z) for w, d, z in
                  [(top_w, top_d, top), (bottom_w, bottom_d, bottom)]
                  for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
        mesh(points, [(3, 2, 1, 0), (4, 5, 6, 7), (0, 1, 5, 4),
                      (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)], material)

    def round_part(cx, cy, rings, material, segments=32):
        points = [(cx + r * math.cos(2 * math.pi * i / segments),
                   cy + r * math.sin(2 * math.pi * i / segments), z)
                  for z, r in rings for i in range(segments)]
        polygons = [tuple(reversed(range(segments))), tuple(range((len(rings)-1)*segments, len(rings)*segments))]
        polygons.extend((j*segments+i, j*segments+(i+1)%segments,
                         (j+1)*segments+(i+1)%segments, (j+1)*segments+i)
                        for j in range(len(rings)-1) for i in range(segments))
        mesh(points, polygons, material)

    w, d = shape['width_m'], shape['depth_m']
    top, bottom = shape['housing_top_m'], settings['body_length_m']
    # 검은 사다리꼴 하우징, 금속 플랜지, 중앙 목과 측면 프레임.
    housing(w*.72, d*.72, w, d, top, bottom, 'housing_black')
    round_part(0, 0, [(0, .028), (.012, .028)], 'metal')
    round_part(0, 0, [(.012, .017), (top-.008, .017)], 'housing_black')
    housing(w*.75, d*.75, w*.75, d*.75, top-.008, top-.004, 'frame_black')
    for side in (-1, 1):
        # 프레임 슬롯은 검은 판에 밝은 인셋을 넣어 모양을 표현한다.
        cx = side*w*.29
        for i in range(3):
            round_part(cx, (i-1)*d*.20, [(top-.0085, .006), (top-.008, .006)], 'metal')
    # 호스는 장착부에서 하우징까지 이어지는 가는 검은 관으로 표현한다.
    for i in range(18):
        t = i/17
        cx = w*.38*math.sin(math.pi*t/2)
        z = .006 + (top-.006)*t
        round_part(cx, d*.32, [(z, .003), (z+.004, .003)], 'hose')
    radius, height = settings['cup_radius_m'], settings['cup_height_m']
    for row in range(shape['rows']):
        for col in range(shape['columns']):
            x = (col-(shape['columns']-1)/2)*shape['pitch_m']
            y = (row-(shape['rows']-1)/2)*shape['pitch_m']
            # 초록색 주름관과 노란색 넓은 접촉 립. 흡착력은 구현하지 않는다.
            rings = [(bottom, radius*.55)]
            for i in range(1, 9):
                rings.append((bottom+height*.72*i/8, radius*(.92 if i%2 else .64)))
            round_part(x, y, rings, 'bellows_green')
            round_part(x, y, [(bottom+height*.72, radius*.65),
                             (bottom+height*.94, radius), (bottom+height, radius)], 'lip_yellow')
    # STL에는 별도 재질이 없어 Gazebo가 URDF의 단일 색상을 그대로 사용한다.
    # 면을 삼각형으로 나누며 형상·크기는 유지한다.
    triangles = []
    for face in faces:
        if not face.startswith('f '):
            continue
        indices = [int(i)-1 for i in face.split()[1:]]
        for i in range(1, len(indices)-1):
            points = [vertices[j] for j in (indices[0], indices[i], indices[i+1])]
            a = [points[1][j]-points[0][j] for j in range(3)]
            b = [points[2][j]-points[0][j] for j in range(3)]
            normal = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
            length = math.sqrt(sum(x*x for x in normal))
            normal = [x/length for x in normal] if length else [0., 0., 0.]
            triangles.append(struct.pack('<12fH', *normal, *(v for p in points for v in p), 0))
    (out / 'parcel_suction_head.stl').write_bytes(
        b'AX parcel suction head; metres; material from URDF'.ljust(80, b' ') +
        struct.pack('<I', len(triangles)) + b''.join(triangles))
    print(f'Generated {shape["columns"]} x {shape["rows"]} cup head: {w} x {d} m')


def match_physics():
    # 외형을 바꿔도 컵·ToF까지 포함한 말단 물리 특성을 집게 기본 자세에 맞춘다.
    import numpy as np
    import xacro
    import xml.etree.ElementTree as ET
    from arm_geometry import origin

    def parts(tool):
        robot = ET.fromstring(xacro.process_file(str(ROOT / 'src/ax_description/urdf/ax.urdf.xacro'),
            mappings={'gripper_model': tool, 'test_payload_mass': '0'}).toxml())
        frames = {'gripper_base_link': np.eye(4)}
        for _ in range(len(robot.findall('joint'))):
            for joint in robot.findall('joint'):
                parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
                if parent in frames:
                    frames[child] = frames[parent] @ origin(joint.find('origin'))
        result = []
        for link in robot.findall('link'):
            name, inertial = link.get('name'), link.find('inertial')
            if name not in frames or name == 'gripper_base_link' or inertial is None:
                continue
            transform = frames[name] @ origin(inertial.find('origin'))
            mass = float(inertial.find('mass').get('value'))
            v = inertial.find('inertia')
            inertia = np.array([[float(v.get('i'+a+b if a <= b else 'i'+b+a))
                                 for b in 'xyz'] for a in 'xyz'])
            rotation = transform[:3, :3]
            result.append((name, mass, transform[:3, 3], rotation @ inertia @ rotation.T))
        return result, frames

    def shift(mass, point):
        return mass * (point @ point * np.eye(3) - np.outer(point, point))

    reference, _ = parts('robotiq')
    current, frames = parts('epick')
    other = [row for row in current if row[0] != 'epick_body']
    mass = sum(m for _, m, _, _ in reference) - sum(m for _, m, _, _ in other)
    centre = (sum(m*p for _, m, p, _ in reference) - sum(m*p for _, m, p, _ in other)) / mass
    inertia = (sum(i+shift(m, p) for _, m, p, i in reference) -
               sum(i+shift(m, p) for _, m, p, i in other) - shift(mass, centre))
    t = frames['epick_body']
    centre = t[:3, :3].T @ (centre-t[:3, 3])
    inertia = t[:3, :3].T @ inertia @ t[:3, :3]
    values = np.linalg.eigvalsh(inertia)
    if mass <= 0 or values[0] <= 0 or values[-1] >= sum(values[:2]):
        raise ValueError('이 크기에서는 집게와 동일한 양의 관성을 만들 수 없습니다. 치수·질량 구성을 확인하세요.')
    path = ROOT / 'src/ax_description/config/epick.yaml'
    text = re.sub(r'(?m)^body_mass_kg: .*$', 'body_mass_kg: '+repr(mass), path.read_text(encoding='utf-8'))
    text = text.split('body_com_xyz:')[0] + yaml.safe_dump({
        'body_com_xyz': ' '.join(format(float(v), '.17g') for v in centre),
        'body_inertia': {key: float(inertia[a, b]) for key, a, b in
                        [('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2), ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2)]}}, sort_keys=False)
    path.write_text(text, encoding='utf-8')
    print('Matched Robotiq default-pose mass, centre and inertia; total mass:',
          sum(m for _, m, _, _ in reference))


def preview():
    # PNG는 공유·미리보기용이며 Gazebo에서 쓰는 STL·물리 설정을 변경하지 않는다.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import numpy as np
    import xml.etree.ElementTree as ET

    folder = ROOT / 'src/ax_description/meshes/tools'
    data = (folder / 'parcel_suction_head.stl').read_bytes()
    triangles = np.array([row[3:12] for row in struct.iter_unpack('<12fH', data[84:])]).reshape(-1, 3, 3)
    # 팔에 매달린 모습처럼 흡착면을 아래로 향하게 표시한다.
    triangles[:, :, 2] *= -1
    material = ET.parse(ROOT / 'src/ax_description/urdf/epick_adapter.xacro').find(
        ".//material[@name='parcel_head_dark_gray']/color")
    color = np.array(list(map(float, material.get('rgba').split()))[:3])
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    light = .55+.45*np.abs(normals @ np.array([.3, -.4, .866]))
    colors = color[None, :]*light[:, None]
    vertices = triangles.reshape(-1, 3)
    lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    centre, extent = (lower+upper)/2, (upper-lower)*1.15
    fig = plt.figure(figsize=(12, 6), facecolor='#f4f6f8')
    for index, (title, elevation, azimuth) in enumerate([
            ('Mounted view', 18, -55), ('Suction cup array', -35, -55)], 1):
        ax = fig.add_subplot(1, 2, index, projection='3d', facecolor='#f4f6f8')
        ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, linewidths=0))
        ax.set_xlim(centre[0]-extent[0]/2, centre[0]+extent[0]/2)
        ax.set_ylim(centre[1]-extent[1]/2, centre[1]+extent[1]/2)
        ax.set_zlim(centre[2]-extent[2]/2, centre[2]+extent[2]/2)
        ax.set_box_aspect(extent.copy())
        ax.view_init(elev=elevation, azim=azimuth)
        ax.set_axis_off()
        ax.set_title(title, color='#263747')
    fig.tight_layout()
    path = folder / 'parcel_suction_head_preview.png'
    fig.savefig(path, dpi=160, facecolor=fig.get_facecolor(), bbox_inches='tight', pad_inches=.2)
    plt.close(fig)
    print('Saved preview:', path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview-only', action='store_true', help='기존 STL에서 PNG만 생성')
    args = parser.parse_args()
    if args.preview_only:
        preview()
    else:
        generate()
        match_physics()
