#!/usr/bin/env python3
"""주행 시험 전용 빈 바닥을 메모리·로그에 생성한다. 선택 가능한 맵 자산은 아니다."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def empty_world():
    current = ET.parse(ROOT/'src/ax_description/worlds/planned/ax_warehouse.world').getroot().find('world')
    sdf = ET.Element('sdf',version='1.8')
    world = ET.SubElement(sdf,'world',name='ax_test')
    for item in current:
        if item.tag in ('plugin','physics','gravity','light','scene'):
            world.append(deepcopy(item))
    ground = ET.SubElement(world,'model',name='ground_plane')
    ET.SubElement(ground,'static').text='true'
    link = ET.SubElement(ground,'link',name='ground')
    for tag in ('collision','visual'):
        shape = ET.SubElement(link,tag,name='ground_'+tag)
        plane = ET.SubElement(ET.SubElement(shape,'geometry'),'plane')
        ET.SubElement(plane,'normal').text='0 0 1'
        ET.SubElement(plane,'size').text='30 30'
    return sdf


def write_empty_world(folder):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    path=folder/'empty_test_fixture.sdf'
    root=empty_world();ET.indent(root,space='  ')
    path.write_text(ET.tostring(root,encoding='unicode'),encoding='utf-8')
    return str(path.resolve())
