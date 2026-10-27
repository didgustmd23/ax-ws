#!/usr/bin/env python3
"""개발 자세와 관절 공간 경로에서 AX_1 팔 관절의 정적 중력 토크를 계산한다.

범위: 수평 지면, 강체 링크, 정지 상태. URDF의 공구 질량을 포함해, 외부 하중이 없을 때와
지정한 점에 2 kg 하중을 강체로 붙였다고 가정할 때 각 팔 관절이 중력에 맞서 자세를 유지하는 데
필요한 토크(-tau_gravity, N·m, URDF 관절축 기준 부호)를 보고한다. 가속 참고치(축 기준 관성 ×
계획 가속도, 정지에서 출발하는 단일 관절, 결합·코리올리·마찰·접촉 제외)는 따로 보고한다.
자기·지면 충돌, 도달성, 파지, 전도는 검사하지 않으며 모터 정격이 아니다.

필요: source된 ROS 작업공간(xacro), numpy, scipy. --output을 지정할 때만 파일을 쓴다.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_static_stability import ARM, center_of_mass, fk, origin, xyz  # noqa: E402

GRAVITY = np.array([0., 0., -9.81])   # base_link 기준 m/s^2, 차체 수평
PAYLOAD_KG = 2.0
# src/ax_description/config/arm_joint_limits.yaml의 계획 가속도(rad/s^2). 실행 중 강제되지 않는다.
PLANNED_ACCELERATION = dict(zip(ARM, (0.50, 0.50, 0.70, 1.00, 1.00)))

# 공구별 하중 중심: 기준 프레임, 기본 위치(m, 프레임 좌표), 공구 축 방향(손목에서 멀어지는
# 쪽) 단위 벡터, 축 직각 단위 벡터 2개. 기본 위치는 check_static_stability.py의 가정이다.
# primitive는 손가락 사이, suction은 흡착 접촉면 밖 150 mm의 상자 중심이다.
# EPick TCP(epick_tcp)는 아직 정해지지 않았다.
TOOLS = {
    'primitive': ('gripper_base_link', [0.07, 0, 0], [1, 0, 0], ([0, 1, 0], [0, 0, 1])),
    'suction': ('suction_grasp_frame', [0, 0, 0.15], [0, 0, 1], ([1, 0, 0], [0, 1, 0])),
}
AXIAL_OFFSETS_M = (-0.10, -0.05, 0.05, 0.10)
LATERAL_OFFSETS_M = (-0.05, 0.05)

# 개발 자세(rad, ARM 순서). P0~P3·POSE_A/B는 log/ax_verify/ctrl/gravity.py, L1/L2는
# record.py(effort 범위 안 자세), HANDOFF는 tools/verify_handoff.py의 동작 목표다.
WORK_POSES = {
    'P0_zero': [0, 0, 0, 0, 0],
    'P1_reach': [0, -1.0, 1.0, 0, 0],
    'P2_mid': [0, -0.6, 0.8, -0.2, 0],
    'P3_high': [0, -0.3, 0.6, -0.3, 0],
    'L1_in_limit': [0.4, 0.1, 0.2, -0.3, 0.3],
    'L2_in_limit': [-0.3, 0.0, 0.3, -0.3, -0.3],
    'POSE_A': [0.4, -0.2, 0.4, -0.2, 0.3],
    'POSE_B': [-0.3, 0, 0.3, -0.3, -0.3],
    'HANDOFF_target': [0.3, -0.4, 0.5, 0.3, 0.3],
}
SHOULDER_LOWER = -1.5708
PATH_SAMPLES = 51


# 관절이 움직이는 링크 집합(자식 링크와 모든 하위 링크)을 구한다.
def subtree(root, joint_name):
    children = {}
    for joint in root.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(
            joint.find('child').get('link'))
    start = root.find(f"joint[@name='{joint_name}']/child").get('link')
    result, stack = set(), [start]
    while stack:
        link = stack.pop()
        result.add(link)
        stack += children.get(link, [])
    return result


# 링크별 (질량, 링크 좌표의 무게중심, 링크 좌표축으로 나타낸 무게중심 기준 관성 텐서)를 읽는다.
def inertials(root):
    result = {}
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        i = inertial.find('inertia')
        tensor = np.array([[float(i.get('ixx')), float(i.get('ixy')), float(i.get('ixz'))],
                           [float(i.get('ixy')), float(i.get('iyy')), float(i.get('iyz'))],
                           [float(i.get('ixz')), float(i.get('iyz')), float(i.get('izz'))]])
        # 텐서는 inertial 좌표계 기준이므로 링크 좌표계로 변환한다.
        rotation = origin(inertial.find('origin'))[:3, :3]
        result[link.get('name')] = (float(inertial.find('mass').get('value')),
                                    xyz(inertial.find('origin'), 'xyz'),
                                    rotation @ tensor @ rotation.T)
    return result


# 팔 5축의 (유지 토크, 축 기준 관성)을 (N, 5) 배열로 계산한다.
# 유지 토크 = -(움직이는 질량마다 ((c - p) × m g) · a의 합): 관절이 축 a에 대해 정지 상태를
# 유지하려고 내야 하는 토크다. payload는 (질량, 프레임, 위치) 또는 None이다.
def joint_torques(root, poses, payload=None):
    poses = np.atleast_2d(np.asarray(poses, dtype=float))
    transforms = fk(root, poses)
    masses = inertials(root)
    holding = np.zeros((len(poses), len(ARM)))
    inertia = np.zeros_like(holding)

    for k, name in enumerate(ARM):
        joint = root.find(f"joint[@name='{name}']")
        child = transforms[joint.find('child').get('link')]
        # 관절 회전은 원점과 축을 바꾸지 않으므로 자식 프레임에서 둘 다 얻는다.
        p = child[:, :3, 3]
        a = np.einsum('nij,j->ni', child[:, :3, :3], xyz(joint.find('axis'), 'xyz', '1 0 0'))

        # 관절 하위 링크와(하위 프레임에 붙은 경우) 하중을 움직이는 물체로 모은다.
        bodies = [(m, transforms[link], c, tensor) for link, (m, c, tensor) in masses.items()
                  if link in subtree(root, name)]
        if payload and payload[1] in subtree(root, name):
            bodies.append((payload[0], transforms[payload[1]], np.asarray(payload[2], float),
                           np.zeros((3, 3))))

        # 물체마다 중력 토크를 빼서 유지 토크를, 회전 관성+평행축 항으로 축 관성을 누적한다.
        for m, frame, c, tensor in bodies:
            r = (frame @ np.r_[c, 1])[:, :3] - p
            holding[:, k] -= np.einsum('ni,ni->n', np.cross(r, m * GRAVITY), a)
            rotation = frame[:, :3, :3]
            local_axis = np.einsum('nji,nj->ni', rotation, a)
            perpendicular = r - np.einsum('ni,ni->n', r, a)[:, None] * a
            inertia[:, k] += (np.einsum('ni,ij,nj->n', local_axis, tensor, local_axis)
                              + m * np.einsum('ni,ni->n', perpendicular, perpendicular))
    return holding, inertia


# 관절 공간 직선 경로를 만든다(정지에서 JointTrajectory 점 하나를 보낼 때의 경로).
def interpolate(start, end, count=PATH_SAMPLES):
    s = np.linspace(0, 1, count)[:, None]
    return (1 - s) * np.asarray(start, float) + s * np.asarray(end, float)


# 작업 자세·접근 경로·하한 복귀 경로를 {그룹: [(이름, 자세 배열)]}로 만든다.
def scenario_poses():
    zero = WORK_POSES['P0_zero']
    groups = {'work': [(n, np.array([q], float)) for n, q in WORK_POSES.items()],
              'approach': [(f'P0_zero->{n}', interpolate(zero, q))
                           for n, q in WORK_POSES.items() if n != 'P0_zero'],
              'recovery': []}
    for n, q in WORK_POSES.items():
        sagged = list(q)
        sagged[1] = SHOULDER_LOWER          # 관찰된 상태: 어깨가 하한에 머묾
        groups['recovery'].append((f'{n}@shoulder_lower->{n}', interpolate(sagged, q)))
        groups['recovery'].append((f'{n}@shoulder_lower->P0_zero', interpolate(sagged, zero)))
    return groups


# 관절 한계 전체의 참고 격자를 만든다. 충돌·도달성으로 거르지 않는다.
def envelope_poses():
    grid = np.meshgrid(np.linspace(-1.5708, 1.5708, 13), np.linspace(-2.0, 2.0, 13),
                       np.linspace(-1.57, 1.57, 13), np.linspace(-np.pi, np.pi, 5),
                       indexing='ij')
    return np.column_stack([np.zeros(grid[0].size)] + [g.ravel() for g in grid])


# 하중 기본 위치와 민감도용 축 방향·축 직각 이동 위치를 만든다.
def payload_variants(tool_spec):
    frame, point, axial, laterals = tool_spec
    point, axial = np.asarray(point, float), np.asarray(axial, float)
    variants = {'nominal': point}
    for d in AXIAL_OFFSETS_M:
        variants[f'axial{d:+.2f}'] = point + d * axial
    for i, lateral in enumerate(laterals):
        for d in LATERAL_OFFSETS_M:
            variants[f'lateral{"uv"[i]}{d:+.2f}'] = point + d * np.asarray(lateral, float)
    return frame, variants


# 그룹별로 관절마다 유지 토크 절댓값의 최대와 그 경로 이름·자세를 찾는다.
def maxima(root, groups, payload=None):
    result = {}
    for group, items in groups.items():
        best = {name: {'abs_Nm': -1.0} for name in ARM}
        for label, poses in items:
            holding, _ = joint_torques(root, poses, payload)
            for k, name in enumerate(ARM):
                i = int(np.argmax(np.abs(holding[:, k])))
                if abs(holding[i, k]) > best[name]['abs_Nm']:
                    best[name] = {'abs_Nm': float(abs(holding[i, k])),
                                  'signed_Nm': float(holding[i, k]), 'where': label,
                                  'pose_rad': [round(float(v), 4) for v in poses[i]]}
        result[group] = best
    return result


# 공구 하나에 대해 무부하·2 kg·민감도·전체 범위 참고·가속 참고·작업 자세별 값을 모은다.
def analyse(root, tool):
    frame, variants = payload_variants(TOOLS[tool])
    groups = scenario_poses()
    envelope = envelope_poses()
    out = {'payload_frame': frame, 'payload_nominal_point_m': variants['nominal'].tolist(),
           'no_payload': maxima(root, groups),
           'payload_2kg_nominal': maxima(root, groups, (PAYLOAD_KG, frame, variants['nominal'])),
           'payload_2kg_sensitivity': {},
           'envelope_reference': {}, 'work_poses': {}, 'acceleration_reference': {}}

    # 하중 위치를 옮긴 경우마다 그룹 전체의 관절별 최대 유지 토크를 구한다.
    for key, point in variants.items():
        if key == 'nominal':
            continue
        result = maxima(root, groups, (PAYLOAD_KG, frame, point))
        out['payload_2kg_sensitivity'][key] = {
            name: max(result[g][name]['abs_Nm'] for g in result) for name in ARM}

    # 무부하·2 kg 하중에서 전체 범위 참고 토크와 경로 최대 축 관성의 가속 참고치를 구한다.
    for label, payload in (('no_payload', None),
                           ('payload_2kg_nominal', (PAYLOAD_KG, frame, variants['nominal']))):
        holding, inertia = joint_torques(root, envelope, payload)
        out['envelope_reference'][label] = {
            name: {'abs_Nm': float(np.abs(holding[:, k]).max()),
                   'pose_rad': envelope[int(np.argmax(np.abs(holding[:, k])))].round(4).tolist()}
            for k, name in enumerate(ARM)}

        all_poses = np.vstack([p for items in groups.values() for _, p in items])
        _, path_inertia = joint_torques(root, all_poses, payload)
        out['acceleration_reference'][label] = {
            name: {'max_axis_inertia_kgm2': float(path_inertia[:, k].max()),
                   'planned_acceleration_rad_s2': PLANNED_ACCELERATION[name],
                   'torque_Nm': float(path_inertia[:, k].max() * PLANNED_ACCELERATION[name])}
            for k, name in enumerate(ARM)}

    # 작업 자세별 유지 토크와 차체 COM 여유를 기록한다.
    for name, q in WORK_POSES.items():
        free, _ = joint_torques(root, [q])
        loaded, _ = joint_torques(root, [q], (PAYLOAD_KG, frame, variants['nominal']))
        out['work_poses'][name] = {'pose_rad': q, 'no_payload_Nm': free[0].round(3).tolist(),
                                   'payload_2kg_Nm': loaded[0].round(3).tolist(),
                                   'base_com_margin_m': {
                                       'no_payload': base_margin(root, q, 0., frame, [0, 0, 0]),
                                       'payload_2kg': base_margin(root, q, PAYLOAD_KG, frame,
                                                                  variants['nominal'])}}
    return out


# 전체 무게중심이 바퀴 중심 사각형 안쪽으로 얼마나 떨어져 있는지 계산한다(m, 밖이면 음수).
# check_static_stability.py와 같은 지지 모델이며 관절 토크와는 별도 문제다.
def base_margin(root, pose, payload_mass, frame, point):
    transforms = fk(root, np.array([pose], float))
    _, com = center_of_mass(root, transforms, payload_mass, frame, point)
    wheels = np.array([transforms[n][0, :3, 3] for n in (
        'front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel')])
    (xmin, ymin), (xmax, ymax) = wheels[:, :2].min(0), wheels[:, :2].max(0)
    x, y = com[0, :2]
    return round(float(min(x - xmin, xmax - x, y - ymin, ymax - y)), 4)


# URDF에 설정된 팔 5축의 effort 값을 읽는다.
def efforts(root):
    return {name: float(root.find(f"joint[@name='{name}']/limit").get('effort'))
            for name in ARM}


# 공구별 URDF를 만들어 계산하고 요약 표를 출력하며, --output이 있으면 JSON을 저장한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--tools', nargs='+', default=list(TOOLS), choices=list(TOOLS))
    parser.add_argument('--output', type=Path, help='JSON result file (default: none)')
    args = parser.parse_args()

    # ROS 패키지 경로는 실행할 때만 찾는다.
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('ax_description'))
    result = {'scope': __doc__.split('\n\n')[1].replace('\n', ' '),
              'joint_order': ARM, 'gravity_m_s2': GRAVITY.tolist(), 'payload_kg': PAYLOAD_KG,
              'work_poses_rad': WORK_POSES, 'tools': {}}

    # 공구별 URDF를 생성해 분석한다.
    with tempfile.TemporaryDirectory() as tmp:
        for tool in args.tools:
            urdf = Path(tmp) / f'{tool}.urdf'
            subprocess.run(['xacro', str(share / 'urdf/ax.urdf.xacro'),
                            f'gripper_model:={tool}', '-o', str(urdf)], check=True)
            root = ET.parse(urdf).getroot()
            result['tools'][tool] = dict(analyse(root, tool), effort_Nm=efforts(root))

    # 관절별 effort와 그룹별 최대 유지 토크·민감도·가속 참고치를 표로 출력한다.
    for tool, data in result['tools'].items():
        print(f'== {tool}: max |holding torque| N*m (work / approach / recovery)')
        for name in ARM:
            cells = []
            for label in ('no_payload', 'payload_2kg_nominal'):
                cells.append(' / '.join(f"{data[label][g][name]['abs_Nm']:.1f}"
                                        for g in ('work', 'approach', 'recovery')))
            sens = max(data['payload_2kg_sensitivity'][k][name]
                       for k in data['payload_2kg_sensitivity'])
            accel = data['acceleration_reference']['payload_2kg_nominal'][name]['torque_Nm']
            print(f'{name:24s} effort {data["effort_Nm"][name]:5.1f} | none {cells[0]:17s}'
                  f' | 2 kg {cells[1]:17s} | 2 kg offsets max {sens:5.1f}'
                  f' | accel ref {accel:4.1f}')

    if args.output:
        args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')


if __name__ == '__main__':
    main()
