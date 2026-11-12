#!/usr/bin/env python3
"""두 창고 월드(planned/ax_warehouse.world)의 SDF 형상에서 B3 영역 파일을 만든다(개발용, 미승인).

사용: python3 tools/generate_b3_regions.py [--check]
- 출력: src/ax_environment/config/ax_warehouse_regions.json. --check는 쓰지 않고 현재 파일과 같은지만 확인한다(다르면 종료 코드 1).
- 통로: corridor_<n>_wall_north·south의 안쪽 면(y)과 문 벽(wall_*_door_section)의 바깥 면(x) 사이.
- 접근 구역: 각 통로 양 입구 바깥에 통로 폭과 같은 폭, 깊이 APPROACH_DEPTH_M(개발값)으로 둔다.
- 작업대: source_pickup_zone·destination_unload_zone 모델의 바닥 표시 상자.
- 대기 지점: 개발용 제안 사각형. 접근 구역·통로·작업대·충돌 형상과 떨어져 있어야 한다(시험에서 확인).
- world→map: 채택 지도 revision 1의 T_world_map 초기값 (−6.8, 0, 0)(map 원점 = robot_2 출발 자세). map = world − (−6.8, 0).
"""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / 'src/ax_description/worlds/planned/ax_warehouse.world'
OUTPUT = ROOT / 'src/ax_environment/config/ax_warehouse_regions.json'
MAP_README = 'src/ax_localization/maps/ax_warehouse/README.md'

# 채택 지도 revision 1 README의 T_world_map 초기값(world 기준 map 원점, yaw 0).
T_WORLD_MAP = (-6.8, 0.0, 0.0)
MAP_REVISION = 1

# 접근 구역 깊이(개발값): AX_1 이동 자세 외곽 길이 1.60 m(a1_validation.yaml footprint)와 비슷하게 잡아
# 입구 바로 앞에서 통로를 향한 로봇 한 대가 들어가는 깊이로 둔다. 운영 합의값이 아니다.
APPROACH_DEPTH_M = 1.5

# 대기 지점(개발 제안, world 좌표 [xmin, ymin, xmax, ymax]). 접근 구역에서 0.8 m 이상, 랙·작업대에서 떨어진 빈 바닥.
WAITING_WORLD = {
    'wait_src': [-9.0, 0.5, -7.0, 2.5],
    'wait_dst': [6.5, 2.5, 8.5, 4.5],
}

# 작업대 ID와 월드 바닥 표시 모델
STATIONS = {'station_src': 'source_pickup_zone', 'station_dst': 'destination_unload_zone'}


# SDF 모델의 (x, y) 위치와 상자 크기(x, y)를 읽는다. 회전이 있는 모델은 지원하지 않는다.
def model_boxes(world=WORLD):
    root = ET.parse(world).getroot()
    boxes = {}

    for model in root.iter('model'):
        pose = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        size = model.find('.//geometry/box/size')

        if size is None:
            continue

        if any(abs(v) > 1e-9 for v in pose[3:6]):
            raise ValueError(f'회전한 모델은 지원하지 않음: {model.get("name")}')

        sx, sy, _ = [float(v) for v in size.text.split()]
        has_collision = model.find('.//collision') is not None
        boxes[model.get('name')] = {'x': pose[0], 'y': pose[1], 'sx': sx, 'sy': sy, 'collision': has_collision}

    return boxes


# 상자 하나의 축 정렬 경계 [xmin, ymin, xmax, ymax]
def box_bounds(box):
    return [box['x'] - box['sx'] / 2, box['y'] - box['sy'] / 2, box['x'] + box['sx'] / 2, box['y'] + box['sy'] / 2]


# world 경계를 map 경계로 옮긴다(yaw 0 평행 이동만).
def world_to_map(bounds):
    if abs(T_WORLD_MAP[2]) > 1e-12:
        raise ValueError('회전한 T_world_map은 지원하지 않음')

    ox, oy = T_WORLD_MAP[0], T_WORLD_MAP[1]
    return [bounds[0] - ox, bounds[1] - oy, bounds[2] - ox, bounds[3] - oy]


# 소수 6자리 반올림(파일 비교를 안정적으로 하기 위함)
def rounded(values):
    return [round(v + 0.0, 6) for v in values]


# 통로 n의 world 경계: 남·북 벽 안쪽 면과 서·동 문 벽 바깥 면
def corridor_world(boxes, number):
    north = box_bounds(boxes[f'corridor_{number}_wall_north'])
    south = box_bounds(boxes[f'corridor_{number}_wall_south'])
    west = box_bounds(boxes['wall_source_door_section_1'])
    east = box_bounds(boxes['wall_destination_door_section_1'])
    return [west[0], south[3], east[2], north[1]]


# 영역 하나를 world·map 경계와 함께 만든다.
def region(region_id, kind, world_bounds, basis, **extra):
    item = {'id': region_id, 'kind': kind, 'bounds': rounded(world_to_map(world_bounds)),
            'world_bounds': rounded(world_bounds), 'basis': basis}
    item.update(extra)
    return item


# 전체 영역 파일 내용을 만든다.
def build(world=WORLD):
    boxes = model_boxes(world)
    regions = []

    # 1) 통로 3개
    corridors = {}
    for number in (1, 2, 3):
        bounds = corridor_world(boxes, number)
        corridors[number] = bounds
        regions.append(region(
            f'corridor_{number}', 'corridor', bounds,
            f'SDF corridor_{number}_wall_north/south 안쪽 면(y)과 wall_source/destination_door_section 바깥 면(x) 사이'))

    # 2) 각 통로 양 입구의 접근 구역(서쪽 입구는 +X, 동쪽 입구는 −X로 진입)
    for number, (x0, y0, x1, y1) in corridors.items():
        regions.append(region(
            f'corridor_{number}_approach_west', 'approach', [x0 - APPROACH_DEPTH_M, y0, x0, y1],
            f'corridor_{number} 서쪽 입구 바깥, 통로 폭, 깊이 {APPROACH_DEPTH_M} m(개발값: AX_1 이동 외곽 길이 수준)',
            corridor_id=f'corridor_{number}', entry_heading_rad=0.0))
        regions.append(region(
            f'corridor_{number}_approach_east', 'approach', [x1, y0, x1 + APPROACH_DEPTH_M, y1],
            f'corridor_{number} 동쪽 입구 바깥, 통로 폭, 깊이 {APPROACH_DEPTH_M} m(개발값: AX_1 이동 외곽 길이 수준)',
            corridor_id=f'corridor_{number}', entry_heading_rad=round(math.pi, 6)))

    # 3) 작업대(월드 바닥 표시 구역)
    for station_id, model in STATIONS.items():
        regions.append(region(station_id, 'workstation', box_bounds(boxes[model]),
                              f'SDF {model} 바닥 표시 상자(충돌 없음). 접근 조건·용량은 D2 확인 전'))

    # 4) 대기 지점(개발 제안)
    for waiting_id, bounds in WAITING_WORLD.items():
        regions.append(region(waiting_id, 'waiting', bounds,
                              'B3 개발 제안 빈 바닥. 접근 구역·통로·작업대·충돌 형상과 떨어짐(시험 확인). 안전 주차 승인 아님'))

    return {
        'schema_version': 1,
        'world': 'planned/ax_warehouse.world',
        'world_variants': ['planned/ax_warehouse.world', 'planned/ax_warehouse_plain.world'],
        'frame_id': 'map',
        'map_revision': MAP_REVISION,
        'approved': False,
        'reason': 'DEVELOPMENT_ONLY; B3 개발용 좌표. 통로·접근·대기 경계와 로봇 외곽 팀 합의(B3/D1/D3/A4/D2) 전',
        'source': 'tools/generate_b3_regions.py',
        'transform': {
            'T_world_map': list(T_WORLD_MAP),
            'rule': 'map = world − (T_world_map.x, T_world_map.y), yaw 0',
            'basis': f'{MAP_README} 채택 지도 revision 1 T_world_map 초기값(map 원점 = robot_2 출발 자세). '
                     '참조 지도 최적 정합(−6.822, +0.006, +0.0003 rad)과의 차 약 0.02 m는 반영하지 않음',
        },
        'approach_depth_m': APPROACH_DEPTH_M,
        'regions': regions,
    }


# 파일을 쓰거나(--check면) 현재 파일과 비교한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--check', action='store_true', help='현재 파일과 같은지만 확인')
    arguments = parser.parse_args()

    text = json.dumps(build(), indent=2, ensure_ascii=False) + '\n'

    if arguments.check:
        same = OUTPUT.is_file() and OUTPUT.read_text(encoding='utf-8') == text
        print('같음' if same else f'다름: {OUTPUT}')
        sys.exit(0 if same else 1)

    OUTPUT.write_text(text, encoding='utf-8')
    print(f'작성: {OUTPUT}')


if __name__ == '__main__':
    main()
