#!/usr/bin/env python3
# 공구 5종과 로봇 2종의 설치 자산 경로를 검사한다. 다른 물리 PC 실행을 대신하는 검사는 아니다.
"""Check installed resources for every selectable tool without PC-specific paths."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
share = Path(get_package_share_directory('ax_description'))
count = 0
for tool in ('epick', 'suction', 'primitive', 'robotiq', 'parallel_wide'):
    for name in ('ax.urdf.xacro', 'ax_2.urdf.xacro'):
        model = ET.fromstring(xacro.process_file(str(share / 'urdf' / name), mappings={
            'gripper_model': tool, 'ros2_control_params': ''}).toxml())
        for mesh in model.findall('.//mesh'):
            uri = mesh.get('filename')
            assert uri.startswith('package://'), uri
            package, relative = uri[len('package://'):].split('/', 1)
            target = Path(get_package_share_directory(package)) / relative
            assert target.is_file(), target
            count += 1
for folder in ('src', 'tools', 'maps'):
    for file in (ROOT / folder).rglob('*'):
        if not file.is_file() or file.suffix not in ('.py', '.sh', '.xacro', '.urdf', '.xml', '.yaml', '.world'):
            continue
        text = file.read_text(encoding='utf-8')
        # Reject user-home paths and filesystem URIs.
        for prefix in ('/' + 'home/', '/' + 'mnt/', '/' + 'Users/', 'file' + '://'):
            assert prefix not in text, str(file.relative_to(ROOT)) + ': machine-specific path'
        assert not re.search(r'(?<![A-Za-z])[A-Za-z]:[\\/]', text), str(file.relative_to(ROOT))
worlds = list((share / 'worlds').rglob('*.world'))
assert {p.relative_to(share / 'worlds').as_posix() for p in worlds} == {'planned/ax_warehouse.world', 'planned/ax_warehouse_plain.world'}, worlds
for path in worlds:
    ET.parse(path)
import yaml
map_path = ROOT / 'maps/warehouse_map.yaml'
image = yaml.safe_load(map_path.read_text())['image']
assert not Path(image).is_absolute() and (map_path.parent / image).is_file()
print(f'Portable package resources: {count} mesh references across 10 model/tool combinations; unified world and map OK')
