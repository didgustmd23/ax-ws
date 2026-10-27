#!/usr/bin/env python3
"""AX 질량 특성을 검사하고, 수평 지면에서 표본 자세의 정적 무게중심 투영을 계산한다.

충돌 없는 자세, 파지, 추종, 동적 전도는 검증하지 않는다.
필요: source된 ROS 작업공간, numpy, scipy, matplotlib.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
ARM = [
    'column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint'
]
TARGETS = {
    'suction': ('suction_grasp_frame', [0, 0, .15]),
    'parallel_wide': ('wide_grasp_frame', [0, 0, 0]),
    'primitive': ('gripper_base_link', [.07, 0, 0]),
    'robotiq': ('robotiq_85_base_link', [0, 0, .17]),
}


# XML 요소의 공백 구분 숫자 속성(xyz·rpy 등)을 배열로 읽는다. 요소가 없으면 기본값을 쓴다.
def xyz(element, key, default='0 0 0'):
    return np.array([
        float(v)
        for v in (element.get(key, default) if element is not None else default).split()
    ])


# URDF <origin>의 rpy·xyz를 4×4 동차 변환 행렬로 바꾼다.
def origin(element):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', xyz(element, 'rpy')).as_matrix()
    transform[:3, 3] = xyz(element, 'xyz')
    return transform


# 팔 5축 자세 배열(N, 5)에 대해 base_link 기준 모든 링크의 변환(N, 4, 4)을 구한다.
# 팔 이외 관절(바퀴·손가락·Robotiq)은 0 위치로 둔다.
def fk(root, poses):
    count = len(poses)
    transforms = {'base_link': np.broadcast_to(np.eye(4), (count, 4, 4))}
    remaining = list(root.findall('joint'))

    # 부모 변환이 이미 구해진 관절부터 차례로 자식 링크 변환을 구한다.
    while remaining:
        progress = False
        for joint in remaining[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue

            motion = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
            name = joint.get('name')
            if name in ARM:
                axis = xyz(joint.find('axis'), 'xyz', '1 0 0')
                motion[:, :3, :3] = Rotation.from_rotvec(
                    poses[:, ARM.index(name), None] * axis
                ).as_matrix()

            # 팔 이외 관절은 q=0으로 둔다(Robotiq 열림, primitive 기본 위치, 바퀴).
            transforms[joint.find('child').get('link')] = (
                transforms[parent] @ origin(joint.find('origin')) @ motion
            )
            remaining.remove(joint)
            progress = True
        assert progress, 'URDF has a disconnected graph'
    return transforms


# 자세별 전체 질량과 무게중심을 계산한다. 관성의 물리적 유효성을 확인하고, 하중이 있으면
# 지정 프레임의 점에 붙인 질량으로 더한다.
def center_of_mass(root, transforms, payload_mass, payload_frame, payload_point):
    total = 0.
    weighted = np.zeros((len(next(iter(transforms.values()))), 3))

    # 링크마다 질량·관성 텐서를 확인하고 질량 가중 위치를 누적한다.
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').get('value'))
        assert mass > 0

        # 관성 텐서는 양의 정부호이고 삼각 부등식을 만족해야 한다.
        i = inertial.find('inertia')
        matrix = np.array([[float(i.get('ixx')), float(i.get('ixy')), float(i.get('ixz'))],
                           [float(i.get('ixy')), float(i.get('iyy')), float(i.get('iyz'))],
                           [float(i.get('ixz')), float(i.get('iyz')), float(i.get('izz'))]])
        eigen = np.linalg.eigvalsh(matrix)
        assert np.all(eigen > 0) and eigen[-1] <= sum(eigen[:2]) + 1e-9

        point = np.r_[xyz(inertial.find('origin'), 'xyz'), 1]
        weighted += mass * (transforms[link.get('name')] @ point)[:, :3]
        total += mass

    # 하중은 지정 프레임의 한 점에 붙은 질점으로 더한다.
    if payload_mass:
        weighted += payload_mass * (transforms[payload_frame] @ np.r_[payload_point, 1])[:, :3]
        total += payload_mass
    return total, weighted / total


# 공구별 URDF를 만들어 물리 특성 왕복 일치·충돌 상자 포함을 확인하고, 표본 자세의 정적 COM
# 바닥 투영과 지지 여유를 계산해 JSON·PNG로 저장한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=20000)
    parser.add_argument('--before-dir', type=Path)
    parser.add_argument('--output-dir', type=Path,
                        default=ROOT / 'docs/evidence/physical_properties')
    args = parser.parse_args()
    assert args.samples > 0

    # 설치된 물리 특성과 결과 보고서의 공통 가정을 준비한다.
    share = Path(get_package_share_directory('ax_description'))
    properties = json.loads((share / 'config/physical_properties.json').read_text())
    results = {
        'scope': 'Static level-ground COM only. Sampled joint poses NOT filtered for '
                 'self/ground collision or reachability. No dynamic or grasp validation.',
        'seed': 42,
        'random_samples': args.samples,
        'required_margin_m': .05,
        'payload_assumption': '2 kg rigidly attached at declared point; suction box COM '
                              '150 mm beyond cup plane. Tool grasp force not tested.',
        'models': {},
    }
    plots = {}

    with tempfile.TemporaryDirectory() as tmp:
        for model, (payload_frame, point) in TARGETS.items():
            # 공구별 URDF를 생성하고 구조를 검사한다.
            output = Path(tmp) / f'{model}.urdf'
            subprocess.run(
                ['xacro', str(share/'urdf/ax.urdf.xacro'), f'gripper_model:={model}',
                 '-o', str(output)],
                check=True,
            )
            subprocess.run(['check_urdf', str(output)], check=True, stdout=subprocess.DEVNULL)
            root = ET.parse(output).getroot()

            # 직렬화된 무게중심과 대각 성분뿐 아니라 전체 텐서를 왕복 검증한다.
            for name, prop in properties['links'].items():
                inertial = root.find(f"link[@name='{name}']/inertial")
                assert np.allclose(
                    xyz(inertial.find('origin'), 'xyz'), prop['center_of_mass_m'], atol=1e-10
                )
                assert np.isclose(float(inertial.find('mass').get('value')), prop['mass_kg'])
                collisions = root.findall(f"link[@name='{name}']/collision")
                assert len(collisions) == prop['collision_count']

                # 반올림하지 않은 모든 메시 정점이 설치된 충돌 상자 하나 이상에 들어가는지 확인한다.
                vertices = np.array([
                    [float(x) for x in line.split()[1:4]]
                    for line in (
                        share/properties['mesh_directory']/prop['mesh']
                    ).read_text().splitlines()
                    if line.startswith('v ')
                ])*.001 - prop['pivot_m']
                covered = np.zeros(len(vertices), dtype=bool)
                for collision in collisions:
                    transform = origin(collision.find('origin'))
                    size = xyz(collision.find('geometry/box'), 'size')
                    local = (vertices-transform[:3, 3]) @ transform[:3, :3]
                    covered |= np.all(np.abs(local) <= size/2 + 1e-9, axis=1)
                assert np.all(covered), (model, name, int((~covered).sum()))

            # 이전 URDF가 주어지면 관절과 visual이 바뀌지 않았는지 비교한다.
            if args.before_dir:
                before = ET.parse(args.before_dir / f'{model}.urdf').getroot()
                assert len(root.findall('link')) == len(before.findall('link'))
                for joint in root.findall('joint'):
                    old = before.find(f"joint[@name='{joint.get('name')}']")
                    assert ET.tostring(joint) == ET.tostring(old)

                for link in root.findall('link'):
                    old = before.find(f"link[@name='{link.get('name')}']")
                    assert [
                        ET.canonicalize(ET.tostring(v), strip_text=True)
                        for v in link.findall('visual')
                    ] == [
                        ET.canonicalize(ET.tostring(v), strip_text=True)
                        for v in old.findall('visual')
                    ]

            # 기본 자세·관절 한계 격자·난수 자세를 모아 정기구학을 계산한다.
            limits = np.array([
                [float(root.find(f"joint[@name='{name}']/limit").get(k))
                 for k in ('lower', 'upper')]
                for name in ARM
            ])
            random = np.random.default_rng(42).uniform(
                limits[:, 0], limits[:, 1], size=(args.samples, 5)
            )
            grid = np.array(list(itertools.product(
                *[np.linspace(lo, hi, 5) for lo, hi in limits]
            )))
            poses = np.vstack([np.zeros((1, 5)), grid, random])

            # 모든 자세의 링크 변환을 한 번에 구한다.
            transforms = fk(root, poses)

            # 바퀴 중심이 이루는 사각형을 지지 영역으로 쓴다.
            wheel_centers = np.array([
                transforms[name][0, :3, 3]
                for name in ['front_left_wheel', 'front_right_wheel',
                             'rear_left_wheel', 'rear_right_wheel']
            ])
            xmin, ymin = wheel_centers[:, :2].min(0)
            xmax, ymax = wheel_centers[:, :2].max(0)
            results['support_rectangle_base_link_m'] = [
                float(xmin), float(xmax), float(ymin), float(ymax)
            ]

            # 무부하·2 kg 하중별로 COM과 지지 여유를 계산한다.
            scenarios = {}
            for payload in (0., 2.):
                total, com = center_of_mass(root, transforms, payload, payload_frame, point)
                distances = np.column_stack([
                    com[:, 0]-xmin, xmax-com[:, 0], com[:, 1]-ymin, ymax-com[:, 1]
                ])
                margins = distances.min(1)
                worst = int(np.argmin(margins))
                ground_z = -wheel_centers[0, 2] + .19631

                # 기본 자세(첫 표본) 값과 전체 표본의 최악 여유를 기록한다.
                scenarios[str(payload)] = {
                    'total_mass_kg': total,
                    'nominal_com_base_link_m': com[0].tolist(),
                    'nominal_com_ground_height_m': float(com[0, 2]+ground_z),
                    'nominal_margin_m': float(margins[0]),
                    'nominal_restoring_moment_nearest_edge_Nm': float(total*9.81*margins[0]),
                    'sample_count': len(poses),
                    'minimum_sampled_margin_m': float(margins[worst]),
                    'outside_support_samples': int(np.sum(margins < 0)),
                    'below_50mm_samples': int(np.sum(margins < .05)),
                    'worst_joint_angles_rad': dict(zip(ARM, poses[worst].tolist())),
                }

                if payload == 2:
                    plots[model] = (com[:, :2], com[0, :2], xmin, xmax, ymin, ymax)

            # 공구별 결과를 모으고 요약을 출력한다.
            results['models'][model] = {
                'payload_frame': payload_frame,
                'payload_center_local_m': point,
                'scenarios_payload_kg': scenarios,
            }
            print(model, 'mass/2kg=', round(scenarios['2.0']['total_mass_kg'], 3),
                  'nominal margin=', round(scenarios['2.0']['nominal_margin_m'], 4),
                  'worst unfiltered margin=',
                  round(scenarios['2.0']['minimum_sampled_margin_m'], 4))

    # JSON 보고서를 저장한다.
    destination = args.output_dir
    destination.mkdir(parents=True, exist_ok=True)
    (destination/'static_stability.json').write_text(json.dumps(results, indent=2)+'\n')

    # 그림 저장용 matplotlib은 화면 없이 쓰도록 여기서 불러온다.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    # 공구별로 2 kg 하중 COM 투영과 지지 사각형을 그린다.
    fig, axes = plt.subplots(2, 2, figsize=(10, 10))
    for ax, (model, (points, nominal, xmin, xmax, ymin, ymax)) in zip(axes.flat, plots.items()):
        ax.scatter(points[::5, 0], points[::5, 1], s=1, alpha=.2,
                   label='Unfiltered joint samples')
        ax.add_patch(Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, fill=False, color='black',
                               label='Wheel-center support'))
        ax.add_patch(Rectangle((xmin+.05, ymin+.05), xmax-xmin-.1, ymax-ymin-.1, fill=False,
                               color='orange', linestyle='--', label='50 mm inset'))
        ax.plot(*nominal, 'r*', ms=10, label='q=0, 2 kg payload')
        ax.set(title=model, xlabel='base_link X (m)', ylabel='base_link Y (m)', aspect='equal')
        ax.grid(alpha=.2)

    # 범례·제목을 달고 PNG로 저장한다.
    axes.flat[0].legend(fontsize=7)
    fig.suptitle('AX static COM projections: 2 kg attached payload\n'
                 'Self/ground collisions and dynamic stability NOT evaluated')
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(destination/'com_projection.png', dpi=160)
    print('Saved report and plot to', destination)


if __name__ == '__main__':
    main()
