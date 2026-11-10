#!/usr/bin/env python3
"""현재 URDF의 무적재·중립 자세 스펙을 모델별 JSON/Markdown으로 기록한다."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml
from arm_geometry import origin
from generate_parts_preview import panels, LABELS

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/ax_description'


def calculate(number):
    filename = 'ax.urdf.xacro' if number == 1 else 'ax_2.urdf.xacro'
    robot = ET.fromstring(xacro.process_file(str(PACKAGE/'urdf'/filename),
        mappings={'gripper_model': 'epick'}).toxml())
    frames = {'base_link': np.eye(4)}
    todo = list(robot.findall('joint'))
    while todo:
        progressed = False
        for joint in todo[:]:
            parent = joint.find('parent').get('link')
            if parent in frames:
                frames[joint.find('child').get('link')] = frames[parent] @ origin(joint.find('origin'))
                todo.remove(joint)
                progressed = True
        if not progressed:
            raise ValueError('Disconnected joint graph')
    pieces = []
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').get('value'))
        pose = frames[link.get('name')] @ origin(inertial.find('origin'))
        a = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
        tensor = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']], [a['ixz'], a['iyz'], a['izz']]])
        if np.linalg.eigvalsh(tensor).min() <= 0:
            raise ValueError('Nonpositive inertia: ' + link.get('name'))
        pieces.append(dict(link=link.get('name'), label=LABELS.get(link.get('name'), link.get('name')),
            mass_kg=mass, com_in_base_m=pose[:3, 3].tolist(),
            inertia_in_base=pose[:3, :3] @ tensor @ pose[:3, :3].T))
    total = sum(p['mass_kg'] for p in pieces)
    com = sum(p['mass_kg']*np.array(p['com_in_base_m']) for p in pieces)/total
    # 링크별 관성의 축을 맞춘 뒤 평행축 정리로 전체 무게중심 기준 관성을 합산한다.
    tensor = np.zeros((3, 3))
    for part in pieces:
        d = np.array(part['com_in_base_m'])-com
        tensor += part.pop('inertia_in_base') + part['mass_kg']*((d@d)*np.eye(3)-np.outer(d, d))
    visual_parts = panels(filename)
    points = np.concatenate([p[1].reshape(-1, 3) for p in visual_parts])
    visible = {p[0] for p in visual_parts}
    if visible - LABELS.keys():
        raise ValueError('Missing display labels: ' + repr(visible-LABELS.keys()))
    for name, _, _, sources in visual_parts:
        entry = next((p for p in pieces if p['link']==name), None)
        if entry is not None:
            entry['visual_sources'] = sources
    joints = []
    for joint in robot.findall('joint'):
        if joint.get('type')=='fixed':
            continue
        entry = dict(name=joint.get('name'), type=joint.get('type'))
        if joint.find('limit') is not None:
            entry['limits'] = {k: float(v) for k, v in joint.find('limit').attrib.items()}
        joints.append(entry)
    control = yaml.safe_load((ROOT/f'src/ax_control/config/ax{number}_controllers.yaml').read_text())['/**/diff_drive_controller']['ros__parameters']
    common = yaml.safe_load((ROOT/'src/ax_control/config/common_controllers.yaml').read_text())['/**/diff_drive_controller']['ros__parameters']
    wheels = [frames[f'{end}_{side}_wheel'][:3, 3] for end in ('front','rear') for side in ('left','right')]
    support = np.array(wheels)[:, :2]
    data = dict(model=f'AX_{number}', source_urdf=filename, pose='all movable joints at zero; tool=epick for AX_1; unloaded',
        basis='Virtual simulation assumptions, not measured hardware specifications or rated payload.',
        units=dict(length='m', mass='kg', inertia='kg*m^2', angle='rad'),
        total_mass_kg=total, com_in_base_m=com.tolist(), inertia_at_total_com_kg_m2=tensor.tolist(),
        visual_bounds_base_m=[points.min(0).tolist(), points.max(0).tolist()],
        visual_size_m=np.ptp(points, axis=0).tolist(),
        wheel_radius_m=control['wheel_radius'], wheel_separation_m=control['wheel_separation'],
        wheelbase_m=float(abs(wheels[0][0]-wheels[2][0])),
        wheel_separation_multiplier=control.get('wheel_separation_multiplier', 1.0),
        max_linear_velocity_m_s=common['linear.x.max_velocity'], max_angular_velocity_rad_s=common['angular.z.max_velocity'],
        max_linear_acceleration_m_s2=common['linear.x.max_acceleration'],
        unloaded_com_xy_inside_wheel_centers=bool(np.all(com[:2]>=support.min(0)) and np.all(com[:2]<=support.max(0))),
        rated_payload_kg=None, links=pieces, movable_joints=joints,
        sensors=[dict(name=s.get('name'), type=s.get('type'), rate_hz=float(s.findtext('update_rate')), topic=s.findtext('topic')) for s in robot.findall('.//sensor')])
    for sensor, entry in zip(robot.findall('.//sensor'), data['sensors']):
        camera = sensor.find('camera')
        if camera is not None:
            entry['horizontal_fov_rad'] = float(camera.findtext('horizontal_fov'))
            entry['image_size_px'] = [int(camera.findtext('image/width')),int(camera.findtext('image/height'))]
            entry['projection'] = 'custom lens intrinsics' if camera.find('lens') is not None else 'Gazebo automatic projection'
        lidar = sensor.find('lidar')
        if lidar is not None:
            entry['range_m'] = [float(lidar.findtext('range/min')),float(lidar.findtext('range/max'))]
            entry['horizontal_samples'] = int(lidar.findtext('scan/horizontal/samples'))
    if number == 2:
        cfg = yaml.safe_load((PACKAGE/'config/cargo_height.yaml').read_text())['/**']['ros__parameters']
        data['cargo'] = dict(deck_size_m=[cfg['deck_width_m'],cfg['deck_length_m']],
            max_height_m=cfg['max_height_m'], camera_height_above_deck_m=cfg['camera_height_m'],
            deck_origin_in_base_m=frames['cargo_link'][:3,3].tolist(),
            camera_origin_in_base_m=frames['cargo_rgbd_link'][:3,3].tolist(),
            tested_payloads_kg=[.5,1.,2.], rated_payload_kg=None)
    return data


def markdown(data):
    vector = lambda values: ', '.join(f'{v:.6f}' for v in values)
    lines = [f'# {data["model"]} 개발 모델 스펙', '',
        '현재 URDF에서 자동 계산한 **무적재·중립 자세** 스펙이다. AX_1은 다중 흡착판(epick) 장착 기준이다.',
        '질량·관성·속도는 시뮬레이션 가정이며 실측·제조사 정격이 아니다. 적재 정격 중량은 미확정이다.', '',
        '| 항목 | 값 |', '|---|---|',
        f'| 원본 모델 | `{data["source_urdf"]}` |',
        f'| 총질량(센서·공구 포함) | {data["total_mass_kg"]:.6f} kg |',
        f'| 전체 무게중심 XYZ (`base_link` 기준) | {vector(data["com_in_base_m"])} m |',
        f'| 외형 크기 XYZ (중립 자세) | {vector(data["visual_size_m"])} m |',
        f'| 바퀴 반경 / 좌우 중심 간격 / 앞뒤 중심 간격 | {data["wheel_radius_m"]:.9f} / {data["wheel_separation_m"]:.6f} / {data["wheelbase_m"]:.6f} m |',
        f'| 최대 선속도 / 각속도 | {data["max_linear_velocity_m_s"]} m/s / {data["max_angular_velocity_rad_s"]} rad/s |',
        f'| 최대 선가속도 | {data["max_linear_acceleration_m_s2"]} m/s² |',
        f'| 윤거 보정 계수 | {data["wheel_separation_multiplier"]} (기존 개발 시험값, 새 하중에서 재보정 필요) |',
        f'| 무적재 COM의 바퀴 중심 지지 영역 포함 | {data["unloaded_com_xy_inside_wheel_centers"]} |', '',
        '## 전체 무게중심 기준 관성 텐서 (kg·m²)', '', '```text']
    lines += [vector(row) for row in data['inertia_at_total_com_kg_m2']]
    lines += ['```', '', '자세·공구·적재물 위치가 바뀌면 전체 COM과 관성도 바뀐다. 위 수치는 무적재 중립 자세에만 해당한다.',
        '좌표는 바퀴 접지면이나 월드 원점이 아닌 `base_link` 기준이다.', '']
    if 'cargo' in data:
        c = data['cargo']
        lines += ['## 적재부', '',
            f'- 감시 적재면: {c["deck_size_m"]}m, 적재 높이 한계: {c["max_height_m"]}m.',
            f'- 적재판 상면 원점: ({vector(c["deck_origin_in_base_m"])})m.',
            f'- 카메라: 적재판 위 {c["camera_height_above_deck_m"]}m, 수직 하향, 수평 시야각 100도.',
            '- 카메라 지지 마스트는 아직 모델링하지 않은 가상 설치다. 추가 시 질량·COM 재계산이 필요하다.',
            '- 기존 시험 하중 0.5/1/2kg은 정격 중량이 아니다. 새 적재함에서 별도 동적 검증이 필요하다.',
            '- 옛 상부 LiDAR 외장은 새 CAD에 없어 활성 모델에서 제외했다. 전후 LiDAR 센서는 유지한다.', '']
    lines += ['## 부품별 질량·무게중심', '', '| 표시 이름 | URDF 링크 | 질량(kg) | COM XYZ(m, base_link 기준) |', '|---|---|---:|---|']
    for p in data['links']:
        lines.append(f'| {p["label"]} | `{p["link"]}` | {p["mass_kg"]:.6f} | {vector(p["com_in_base_m"])} |')
    lines += ['', '## 센서', '', '| 이름 | 종류 | 주기(Hz) | 토픽 |', '|---|---|---:|---|']
    for s in data['sensors']:
        lines.append(f'| `{s["name"]}` | {s["type"]} | {s["rate_hz"]:g} | `{s["topic"]}` |')
    lines += ['', '센서 토픽의 robot ID는 실행 인자에 따라 바뀐다. 다중 로봇에서는 각 namespace로 분리한다.', '',
        '## 갱신', '', '`python3 tools/generate_robot_specs.py`로 두 로봇의 문서와 JSON을 함께 갱신한다.',
        '기계 판독용 상세 관절 제한과 부품 데이터는 `src/ax_description/config/ax1_spec.json` 및 `ax2_spec.json`에 있다.', '']
    return '\n'.join(lines)


if __name__ == '__main__':
    for number in (1, 2):
        data = calculate(number)
        (PACKAGE/f'config/ax{number}_spec.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
        (ROOT/f'docs/guides/AX_{number}_SPEC.md').write_text(markdown(data),encoding='utf-8')
        print(data['model'], 'mass', data['total_mass_kg'], 'COM',data['com_in_base_m'])
