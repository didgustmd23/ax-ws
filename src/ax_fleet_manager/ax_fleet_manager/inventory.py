"""D2 W5 등록 지점 카탈로그와 재고 snapshot 필드(순수, ROS 무의존).

- 지점 메타(용도·용량·허용 물체 종류·최대 하중·pose·역할별 정차 자세·대기 지점)는 D2 개발 설정 파일로
  받는다. 등록 지점 원장(D2/D4 소유 미정)의 값이 아니며 수치는 개발 가정값이다.
- 지점 상태(AVAILABLE·RESERVED·OCCUPIED·INACTIVE)는 재고 항목과 활성 예약에서 파생한다.
"""
import json
import math
from dataclasses import dataclass
from pathlib import Path

from . import group_model as gm

# RegisteredLocation.state(공개 값 그대로)
LOCATION_UNKNOWN = 0
LOCATION_AVAILABLE = 1
LOCATION_RESERVED = 2
LOCATION_OCCUPIED = 3
LOCATION_INACTIVE = 4

PURPOSES = ('RECEIPT', 'STORAGE', 'STATION', 'HANDOFF', 'WAIT', 'SHIPPING')
LOCATION_KEYS = ('location_revision', 'purpose', 'zone_id', 'capacity', 'allowed_object_classes',
                 'max_load_kg', 'active', 'pose', 'stop_poses', 'wait_location_id')
POSE_KEYS = ('x', 'y', 'yaw')
CATALOG_KEYS = ('catalog_id', 'map_revision', 'locations')
MAX_LOCATIONS = 100
MAX_CATALOG_BYTES = 256 * 1024

# 새 사유 코드(D2 내부, ROS 연결 시 계약 D-13 반영 필요)
LOCATION_FULL = 'LOCATION_FULL'
OBJECT_CLASS_NOT_ALLOWED = 'OBJECT_CLASS_NOT_ALLOWED'
INVENTORY_MISMATCH = 'INVENTORY_MISMATCH'
LOCATION_INACTIVE_CODE = 'LOCATION_INACTIVE'


@dataclass(frozen=True)
class LocationCatalog:
    """등록 지점 카탈로그. locations는 {location_id: 검증된 사전}."""

    catalog_id: str
    map_revision: int
    locations: dict

    # registry용 {지점 ID: revision}
    def revisions(self):
        return {k: v['location_revision'] for k, v in self.locations.items()}

    # 지점 정보(없으면 None)
    def get(self, location_id):
        return self.locations.get(location_id)


# 유한한 map 기준 자세 사전 검사
def _check_pose(name, pose):
    if not isinstance(pose, dict) or set(pose) != set(POSE_KEYS):
        raise gm.SpecError('INVALID_SCHEMA', f'{name} 키는 {POSE_KEYS}')

    for key in POSE_KEYS:
        value = pose[key]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise gm.SpecError('INVALID_NUMBER', f'{name}.{key}')

    return {k: float(pose[k]) for k in POSE_KEYS}


# 지점 사전 하나 검증
def _parse_location(location_id, data):
    gm._check_id('location_id', location_id)
    if not isinstance(data, dict) or set(data) != set(LOCATION_KEYS):
        raise gm.SpecError('INVALID_SCHEMA', f'{location_id} 키는 {LOCATION_KEYS}')

    gm._check_uint('location_revision', data['location_revision'])

    if data['purpose'] not in PURPOSES:
        raise gm.SpecError('INVALID_SCHEMA', f'{location_id}.purpose')

    gm._check_id('zone_id', data['zone_id'])
    if type(data['capacity']) is not int or not 0 <= data['capacity'] <= 1000:
        raise gm.SpecError('INVALID_NUMBER', f'{location_id}.capacity')

    classes = data['allowed_object_classes']
    if not isinstance(classes, list) or not all(type(c) is str and c for c in classes):
        raise gm.SpecError('INVALID_SCHEMA', f'{location_id}.allowed_object_classes')

    load = data['max_load_kg']
    if type(load) not in (int, float) or not math.isfinite(load) or load < 0:
        raise gm.SpecError('INVALID_NUMBER', f'{location_id}.max_load_kg')

    if type(data['active']) is not bool:
        raise gm.SpecError('INVALID_SCHEMA', f'{location_id}.active')

    stops = data['stop_poses']
    if not isinstance(stops, dict) or not set(stops) <= set(gm.ROLES):
        raise gm.SpecError('INVALID_SCHEMA', f'{location_id}.stop_poses 키는 역할 이름')

    if type(data['wait_location_id']) is not str:
        raise gm.SpecError('INVALID_SCHEMA', f'{location_id}.wait_location_id')

    parsed = dict(data)
    parsed['allowed_object_classes'] = list(classes)
    parsed['max_load_kg'] = float(load)
    parsed['pose'] = _check_pose(f'{location_id}.pose', data['pose'])
    parsed['stop_poses'] = {role: _check_pose(f'{location_id}.stop_poses.{role}', pose)
                            for role, pose in stops.items()}

    return parsed


# 카탈로그 사전 검증
def parse_catalog(data):
    if not isinstance(data, dict) or set(data) != set(CATALOG_KEYS):
        raise gm.SpecError('INVALID_SCHEMA', f'카탈로그 키는 {CATALOG_KEYS}')

    gm._check_id('catalog_id', data['catalog_id'])
    gm._check_uint('map_revision', data['map_revision'])

    locations = data['locations']
    if not isinstance(locations, dict) or not 0 < len(locations) <= MAX_LOCATIONS:
        raise gm.SpecError('INVALID_SCHEMA', f'locations는 1~{MAX_LOCATIONS}개 사전')

    parsed = {k: _parse_location(k, v) for k, v in locations.items()}

    for location_id, info in parsed.items():
        wait = info['wait_location_id']
        if wait and wait not in parsed:
            raise gm.SpecError('UNKNOWN_LOCATION', f'{location_id}.wait_location_id={wait}')

    return LocationCatalog(data['catalog_id'], data['map_revision'], parsed)


# 카탈로그 JSON 파일 읽기(크기 상한)
def load_catalog(path):
    data_path = Path(path)
    if data_path.stat().st_size > MAX_CATALOG_BYTES:
        raise gm.SpecError('INVALID_CONFIG', f'카탈로그 크기 상한 초과: {MAX_CATALOG_BYTES}')

    try:
        data = json.loads(data_path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise gm.SpecError('INVALID_CONFIG', f'카탈로그 읽기 실패: {error}') from error

    return parse_catalog(data)


# 물체 종류가 지점에 허용되는지("*"는 제한 없음, 빈 목록은 모두 불가)
def class_allowed(info, object_class):
    classes = info['allowed_object_classes']

    return '*' in classes or (object_class != '' and object_class in classes)


# 지점 파생 상태: 사용 중지 → INACTIVE, 물품으로 가득 → OCCUPIED, 활성 예약 → RESERVED, 그 밖 AVAILABLE
def location_state(info, items, active_reservations):
    if not info['active']:
        return LOCATION_INACTIVE

    if items >= info['capacity'] and info['capacity'] > 0:
        return LOCATION_OCCUPIED

    if active_reservations:
        return LOCATION_RESERVED

    return LOCATION_AVAILABLE


# InventorySnapshot 필드(locations·items). pose는 map 기준 (x, y, yaw) 사전으로 둔다
def inventory_snapshot_fields(store, catalog):
    reservations = store.reservations()
    locations = []

    for location_id, info in sorted(catalog.locations.items()):
        items, active = store.location_load(location_id)
        current = [r['reservation_id'] for r in reservations
                   if r['location_id'] == location_id and r['state'] == 'ACTIVE']
        locations.append({
            'location_id': location_id,
            'location_revision': info['location_revision'],
            'map_revision': catalog.map_revision,
            'purpose': info['purpose'],
            'zone_id': info['zone_id'],
            'capacity': info['capacity'],
            'allowed_object_classes': list(info['allowed_object_classes']),
            'max_load_kg': info['max_load_kg'],
            'state': location_state(info, items, active),
            'reservation_id': current[-1] if current else '',
            'pose': dict(info['pose']),
            'wait_location_id': info['wait_location_id'],
            'stop_poses': {k: dict(v) for k, v in info['stop_poses'].items()},
        })

    items = [{
        'object_id': e['object_id'], 'revision': e['revision'],
        'receipt_event_id': e['receipt_event_id'], 'location_id': e['location_id'],
        'carrier_robot_id': e['carrier_robot_id'],
        'assignment_generation': e['assignment_generation'], 'state': e['state'],
        'isolated': e['isolated'],
    } for e in store.inventory_entries()]

    return {'revision': store.inventory_revision(), 'map_revision': catalog.map_revision,
            'locations': locations, 'items': items}
