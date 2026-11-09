"""B3의 영역 장부와 보수적인 경계 판정. 센서·통로 허가를 대신하지 않는다."""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path


def checked_bounds(value):
    """사각형을 [왼쪽 x, 아래 y, 오른쪽 x, 위 y] 순서로 검사한다."""
    if (not isinstance(value, (list, tuple)) or len(value) != 4
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in value)):
        raise ValueError('경계에는 유한한 숫자 4개가 필요합니다.')
    left, bottom, right, top = value
    if left >= right or bottom >= top:
        raise ValueError('경계의 왼쪽·아래 값은 오른쪽·위 값보다 작아야 합니다.')
    return tuple(value)


def load_regions(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if (data.get('schema_version') != 1 or not isinstance(data.get('approved'), bool)
            or not data.get('frame_id') or not data.get('world')
            or type(data.get('map_revision')) is not int or data['map_revision'] < 0):
        raise ValueError('영역 파일의 버전·좌표계·지도 revision·승인 표시를 확인하세요.')
    if not isinstance(data.get('regions'), list) or not data['regions']:
        raise ValueError('영역 목록이 필요합니다.')
    ids = set()
    for region in data['regions']:
        name = region.get('id')
        if (not isinstance(name, str) or not name or name in ids
                or region.get('kind') not in ('corridor', 'workstation', 'waiting')):
            raise ValueError('영역 ID 중복·누락 또는 잘못된 종류입니다.')
        ids.add(name)
        if region.get('bounds') is not None:
            checked_bounds(region['bounds'])
        elif data['approved']:
            raise ValueError('좌표가 없는 영역을 승인할 수 없습니다.')
    if data['approved'] and (data['frame_id'] != 'map' or data['map_revision'] < 1):
        raise ValueError('운영 승인은 채택된 map 좌표·revision이 필요합니다.')
    return data


@dataclass(frozen=True)
class Observation:
    state: str
    reason: str
    position_valid: bool = False


def observe_corridor(catalog, corridor_id, footprint_bounds, *, frame_id,
                     map_revision, position_valid, outline_complete, age_s,
                     max_age_s=1.0, uncertainty_m=0.0):
    """전체 외곽이 경계에서 떨어졌을 때만 OUTSIDE를 반환한다.

    footprint_bounds는 차체 중심점이 아니라 접힌 팔·적재물까지 포함한 외곽의
    축 정렬 포위 사각형이어야 한다. 로봇 1·2 모두 동일한 판정 규칙을 사용한다.
    """
    unknown = lambda reason: Observation('UNKNOWN', reason)
    if not catalog.get('approved'):
        return unknown('REGIONS_NOT_APPROVED')
    if frame_id != catalog['frame_id'] or map_revision != catalog['map_revision']:
        return unknown('FRAME_OR_MAP_MISMATCH')
    if (not position_valid or not outline_complete or not math.isfinite(age_s)
            or not math.isfinite(max_age_s) or max_age_s <= 0 or not 0 <= age_s <= max_age_s):
        return unknown('POSITION_OR_OUTLINE_INVALID')
    if not math.isfinite(uncertainty_m) or uncertainty_m < 0:
        return unknown('UNCERTAINTY_INVALID')
    region = next((r for r in catalog['regions'] if r['id'] == corridor_id
                   and r['kind'] == 'corridor'), None)
    if region is None or region.get('bounds') is None:
        return unknown('CORRIDOR_UNDEFINED')
    try:
        left, bottom, right, top = checked_bounds(region['bounds'])
        x0, y0, x1, y1 = checked_bounds(footprint_bounds)
    except ValueError:
        return unknown('GEOMETRY_INVALID')
    # 위치 오차만큼 외곽을 넓혀 '아직 안에 있을 가능성'을 보수적으로 남긴다.
    x0 -= uncertainty_m
    y0 -= uncertainty_m
    x1 += uncertainty_m
    y1 += uncertainty_m
    outside = x1 < left or x0 > right or y1 < bottom or y0 > top
    return Observation('OUTSIDE' if outside else 'INSIDE', 'GEOMETRY_CHECKED', True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--require-approved', action='store_true')
    args = parser.parse_args()
    if args.config is None:
        from ament_index_python.packages import get_package_share_directory
        args.config = Path(get_package_share_directory('ax_environment')) / 'config/aws_warehouse_regions.json'
    try:
        catalog = load_regions(args.config)
    except (ValueError, OSError) as error:
        parser.exit(2, f'영역 파일 오류: {error}\n')
    print(f"좌표계: {catalog['frame_id']} / 지도 revision: {catalog['map_revision']}")
    print(f"운영 승인: {catalog['approved']} / 사유: {catalog.get('reason', '')}")
    for region in catalog['regions']:
        print(f"{region['id']} ({region['kind']}): {region.get('bounds') or '좌표 미확정'}")
    if args.require_approved and not catalog['approved']:
        parser.exit(2, '미승인 영역이므로 운영 연결을 차단합니다.\n')


if __name__ == '__main__':
    main()
