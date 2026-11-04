#!/usr/bin/env python3
"""현재 URDF의 실제 visual에서 로봇별 부품 모음 PNG와 상대경로 안내를 생성한다."""
from pathlib import Path
import math
import struct
import os
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from arm_geometry import origin
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import font_manager, ft2font

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/ax_description'
OUT = PACKAGE / 'meshes'
LABELS = {
    'base_link': '차체', 'front_left_wheel': '왼쪽 앞바퀴', 'front_right_wheel': '오른쪽 앞바퀴',
    'rear_left_wheel': '왼쪽 뒷바퀴', 'rear_right_wheel': '오른쪽 뒷바퀴',
    'arm_mount_link': '팔 고정 받침', 'column_link': '회전 기둥', 'arm_rotating_base_link': '윗팔 연결 받침',
    'upper_arm_link': '윗팔', 'forearm_link': '아래팔', 'wrist_link': '손목',
    'gripper_base_link': '공구 연결부', 'epick_body': '다중 흡착판 · 12컵',
    'shelf_link': '적재함', 'front_trim_link': '보조 부품(현재 바퀴 모양 메시)',
    'top_lidar_housing_link': '상부 LiDAR 하우징(외형)', 'cargo_contact_link': '적재함 접촉 센서 영역',
    'front_lidar_link': '전방 LiDAR', 'rear_lidar_link': '후방 LiDAR',
    'front_rgbd_link': '전방 RGB-D 카메라', 'front_rgb_link': '전방 RGB 카메라',
    'arm_rgbd_link': '손목 RGB-D 카메라', 'arm_rgb_link': '팔 RGB 카메라',
    'imu_link': '자세·가속도 센서', 'epick_tof_link': '흡착판 거리 센서',
}


def korean_font():
    # 개인 폴더 경로를 저장하지 않고 시스템 글꼴을 찾아 이미지 한글 표시에 사용한다.
    candidates = font_manager.findSystemFonts()
    if os.name == 'nt':
        windows = os.environ.get('WINDIR') or os.environ.get('SystemRoot')
        if windows:
            candidates.insert(0, str(Path(windows)/'Fonts/malgun.ttf'))
    elif os.environ.get('WSL_DISTRO_NAME'):
        try:
            windows = subprocess.check_output(['cmd.exe', '/c', 'echo', '%WINDIR%'], text=True,
                encoding='utf-8', errors='replace', stderr=subprocess.DEVNULL).strip().splitlines()[-1]
            folder = subprocess.check_output(['wslpath', '-u', windows], text=True,
                encoding='utf-8', errors='replace', stderr=subprocess.DEVNULL).strip()
            candidates.insert(0, str(Path(folder)/'Fonts/malgun.ttf'))
        except (OSError, subprocess.SubprocessError):
            pass
    for path in candidates:
        try:
            if ord('가') in ft2font.FT2Font(path).get_charmap():
                return font_manager.FontProperties(fname=path)
        except (OSError, RuntimeError):
            pass
    return None


def load_mesh(path):
    if path.suffix.lower() == '.stl':
        data = path.read_bytes()
        faces = np.array([row[3:12] for row in struct.iter_unpack('<12fH', data[84:])]).reshape(-1, 3, 3)
        return faces, np.tile([.3, .33, .36], (len(faces), 1))
    vertices, faces, colors, palette = [], [], [], {}
    color = [.45, .5, .55]
    for line in path.read_text().splitlines():
        row = line.split()
        if not row:
            continue
        if row[0] == 'mtllib':
            for name in row[1:]:
                material = path.parent / name
                if not material.exists():
                    continue
                for entry in material.read_text(errors='replace').splitlines():
                    tokens = entry.split()
                    if tokens and tokens[0] == 'newmtl':
                        key = tokens[1]
                    if tokens and tokens[0] == 'Kd':
                        palette[key] = list(map(float, tokens[1:4]))
        elif row[0] == 'usemtl':
            color = palette.get(row[1], [.45, .5, .55])
        elif row[0] == 'v':
            vertices.append(list(map(float, row[1:4])))
        elif row[0] == 'f':
            indices = [int(token.split('/')[0]) for token in row[1:]]
            indices = [i-1 if i > 0 else len(vertices)+i for i in indices]
            for i in range(1, len(indices)-1):
                faces.append([vertices[j] for j in (indices[0], indices[i], indices[i+1])])
                colors.append(color)
    return np.array(faces), np.clip(np.array(colors), 0, 1)


def primitive(geometry):
    box = geometry.find('box')
    if box is not None:
        half = np.fromstring(box.get('size'), sep=' ')/2
        vertices = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])*half
        quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        return np.array([vertices[list(t)] for a, b, c, d in quads for t in [(a, b, c), (a, c, d)]])
    cylinder = geometry.find('cylinder')
    sphere = geometry.find('sphere')
    radius = float((cylinder if cylinder is not None else sphere).get('radius'))
    length = float(cylinder.get('length')) if cylinder is not None else 2*radius
    # 센서 도형 미리보기. 구는 위도 고리로 만든다.
    rings = [(-length/2, radius), (length/2, radius)] if cylinder is not None else [
        (radius*math.cos(t), radius*math.sin(t)) for t in np.linspace(.001, math.pi-.001, 10)]
    points = np.array([[[r*math.cos(a), r*math.sin(a), z] for a in np.linspace(0, 2*math.pi, 17)[:-1]] for z, r in rings])
    faces = []
    for i in range(len(rings)-1):
        for j in range(16):
            a, b, c, d = points[i, j], points[i, (j+1)%16], points[i+1, (j+1)%16], points[i+1, j]
            faces.extend([[a, b, c], [a, c, d]])
    for ring in (points[0], points[-1]):
        centre = ring.mean(axis=0)
        faces.extend([[centre, ring[j], ring[(j+1)%16]] for j in range(16)])
    return np.array(faces)


def panels(model):
    robot = ET.fromstring(xacro.process_file(str(PACKAGE/'urdf'/model), mappings={'gripper_model': 'epick'}).toxml())
    frames = {'base_link': np.eye(4)}
    for _ in range(len(robot.findall('joint'))):
        for joint in robot.findall('joint'):
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent in frames:
                frames[child] = frames[parent] @ origin(joint.find('origin'))
    result = []
    for link in robot.findall('link'):
        chunks, shades, sources = [], [], []
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            mesh = geometry.find('mesh')
            if mesh is not None:
                package, relative = mesh.get('filename')[10:].split('/', 1)
                path = Path(get_package_share_directory(package))/relative
                triangles, colors = load_mesh(path)
                triangles *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                sources.append(relative.removeprefix('meshes/') if package == 'ax_description' else mesh.get('filename'))
            else:
                triangles = primitive(geometry)
                colors = np.tile([.35, .45, .55], (len(triangles), 1))
                sources.append('기본 도형: URDF visual')
            color = visual.find('material/color')
            if color is not None:
                colors[:] = np.fromstring(color.get('rgba'), sep=' ')[:3]
            transform = frames[link.get('name')] @ origin(visual.find('origin'))
            triangles = triangles @ transform[:3, :3].T + transform[:3, 3]
            chunks.append(triangles)
            shades.append(colors)
        if chunks:
            result.append((link.get('name'), np.concatenate(chunks), np.concatenate(shades), sources))
    return result


def draw(parts, number):
    font = korean_font()
    all_triangles = np.concatenate([p[1] for p in parts])
    all_colors = np.concatenate([p[2] for p in parts])
    items = [('Neutral assembly', all_triangles, all_colors, [])]+parts
    rows = math.ceil(len(items)/4)
    fig = plt.figure(figsize=(16, rows*3.4), facecolor='#f4f6f8')
    for index, (name, triangles, colors, _) in enumerate(items, 1):
        ax = fig.add_subplot(rows, 4, index, projection='3d', facecolor='#f4f6f8')
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        colors = colors*(.65+.35*np.abs(normals @ np.array([.3, -.4, .866])))[:, None]
        ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, linewidths=0))
        lower, upper = triangles.reshape(-1, 3).min(axis=0), triangles.reshape(-1, 3).max(axis=0)
        centre, size = (lower+upper)/2, max(np.max(upper-lower)*1.12, .001)
        ax.set_xlim(centre[0]-size/2, centre[0]+size/2)
        ax.set_ylim(centre[1]-size/2, centre[1]+size/2)
        ax.set_zlim(centre[2]-size/2, centre[2]+size/2)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=18, azim=-55)
        ax.set_axis_off()
        title = ('전체 조립 · 기본 자세' if name == 'Neutral assembly' else LABELS.get(name, name)+'\n'+name) if font else name
        ax.set_title(title, fontsize=10, color='#263747', pad=0, fontproperties=font)
    title = f'로봇{number} · 실제 부품 형상 · 각 부품은 개별 확대' if font else f'AX_{number} | actual visual geometry | panels individually scaled'
    fig.suptitle(title, fontsize=17, fontproperties=font)
    fig.tight_layout(rect=(0, 0, 1, .97))
    fig.savefig(OUT/f'robot_{number}_parts.png', dpi=130, bbox_inches='tight', pad_inches=.2)
    plt.close(fig)


def main():
    lines = ['# 로봇 부품 미리보기', '',
             '실제 URDF의 visual 형상에서 생성한 부품 모음입니다. PNG를 클릭하거나 이 README에서 **Ctrl+Shift+V**를 누르면 VS Code에서 볼 수 있습니다. GitHub에서도 표시됩니다.', '',
             '각 부품은 보기 좋게 개별 확대했습니다. 그림 사이의 크기는 실물 비율이 아니며, 전체 조립 모습은 기본 관절 자세입니다. 재질과 조명 표현은 Gazebo와 다를 수 있습니다.', '']
    for number, model in [(1, 'ax.urdf.xacro'), (2, 'ax_2.urdf.xacro')]:
        parts = panels(model)
        draw(parts, number)
        lines += [f'## 로봇{number}', '', f'![로봇{number} 전체 조립과 부품별 미리보기](robot_{number}_parts.png)', '',
                  f'[원본 모델 설정](../urdf/{model})', '', '| 부품 설명 | 실제 링크 이름 | 형상 파일·설정 |', '|---|---|---|']
        for name, _, _, sources in parts:
            links = ['['+s.split('/')[-1]+']('+s+')' if not s.startswith(('기본', 'package:')) else s for s in sources]
            lines.append(f'| {LABELS.get(name, name)} | `{name}` | '+', '.join(links)+' |')
        lines.append('')
        print(f'Generated robot_{number}_parts.png: {len(parts)} visible parts')
    lines += ['## 공구·좌표 기준', '',
              '로봇1 그림은 다중 흡착판 · 12컵을 장착한 모습입니다. 손가락 집게 · Robotiq 2F-85는 교체 공구이며 동시에 장착하지 않습니다.',
              '[흡착판 상세 미리보기](tools/README.md) · [교체 집게 모델](../../robotiq_description/README.md)', '',
              '`epick_tcp`, `epick_tip`, `tool_tof_link`, 카메라의 `*_optical_frame`처럼 visual이 없는 좌표 기준은 독립 부품 이미지가 없습니다. 흡착컵 배열 외형은 `epick_body`의 STL에 포함됩니다.', '',
              '| 사용자 표시 이름 | 실행 선택값 | 링크·좌표 기준 |', '|---|---|---|',
              '| 다중 흡착판 · 12컵 | `tool:=epick` | `epick_body`(전체 외형), `epick_suction_cup`(컵 배열 충돌·질량), `epick_tcp`(파지 기준점) |',
              '| 손가락 집게 · Robotiq 2F-85 | `tool:=robotiq` | `robotiq_85_*` 링크 |', '',
              '`epick`은 기존 실행·센서·TCP 연결을 유지하기 위한 호환 이름입니다. 현재 다중 흡착판이 실제 EPick 제품이라는 뜻은 아닙니다.', '',
              '## 갱신 방법', '', '모델·센서·공구 형상을 변경하면 Ubuntu에서 ROS 환경과 빌드된 작업공간을 source한 뒤 저장소 루트에서 실행합니다. NumPy·SciPy·Matplotlib·Xacro·PyYAML이 필요합니다.', '',
              '```bash', 'python3 tools/generate_parts_preview.py', '```', '',
              '이 도구는 미리보기 PNG 두 장과 이 안내만 갱신하며 로봇 모델·질량·관절 설정은 변경하지 않습니다.', '']
    (OUT/'README.md').write_text('\n'.join(lines), encoding='utf-8')


if __name__ == '__main__':
    main()
