"""B3의 영역 장부와 보수적인 경계 판정(축 정렬 근사 observe_corridor, 회전 외곽 observe_robot). 센서·통로 허가를 대신하지 않는다."""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path


# 영역 종류. approach는 통로 입구 바로 바깥의 접근 구역이다(corridor_id·entry_heading_rad 필요).
KINDS = ('corridor', 'approach', 'workstation', 'waiting')


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


# 영역 파일을 읽어 버전·좌표계·승인·영역 종류·접근 구역 참조를 검사하고 내용을 돌려준다.
def load_regions(path):
    # 파일이 존재해도 운영 승인된 것은 아닙니다. 좌표·버전·경계 형식을 먼저 검사합니다.
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
                or region.get('kind') not in KINDS):
            raise ValueError('영역 ID 중복·누락 또는 잘못된 종류입니다.')
        ids.add(name)
        if region.get('bounds') is not None:
            checked_bounds(region['bounds'])
        elif data['approved']:
            raise ValueError('좌표가 없는 영역을 승인할 수 없습니다.')

    # 접근 구역은 같은 파일의 통로를 가리키고 진입 방향(rad)을 가져야 한다.
    corridors = {r['id'] for r in data['regions'] if r['kind'] == 'corridor'}
    for region in data['regions']:
        if region['kind'] != 'approach':
            continue
        heading = region.get('entry_heading_rad')
        if region.get('corridor_id') not in corridors:
            raise ValueError(f"접근 구역 {region['id']}의 corridor_id가 이 파일의 통로가 아닙니다.")
        if (isinstance(heading, bool) or not isinstance(heading, (int, float))
                or not math.isfinite(heading) or abs(heading) > math.pi + 1e-6):
            raise ValueError(f"접근 구역 {region['id']}의 entry_heading_rad는 −π~π의 유한한 값이어야 합니다.")
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


# 판정 상수(개발값). 진입 방향 허용 각도: 접근 구역에서 차체 +X가 통로 진입 방향에서 이 각도 안이면 APPROACHING.
# 60°는 문 앞에서 회전하며 정렬하는 중간 자세도 포함하도록 잡은 값이며 운영 합의값이 아니다.
HEADING_TOLERANCE_RAD = math.pi / 3
# 접촉 판정 허용 오차(m). 경계에 닿는 것도 겹침으로 본다.
TOUCH_EPS_M = 1e-9


def outline_polygon(outline_bounds, pose, inflate_m=0.0):
    """차체 좌표 외곽 사각형을 map 자세(x, y, yaw)로 옮긴 꼭짓점 4개를 돌려준다.

    inflate_m만큼 차체 좌표에서 사방으로 넓힌 뒤 회전한다(원 반지름 inflate_m의 민코프스키 합을 포함하는 보수적 근사).
    """
    x0, y0, x1, y1 = checked_bounds(outline_bounds)
    if not math.isfinite(inflate_m) or inflate_m < 0:
        raise ValueError('외곽 확장 값은 0 이상의 유한한 값이어야 합니다.')

    x0, y0, x1, y1 = x0 - inflate_m, y0 - inflate_m, x1 + inflate_m, y1 + inflate_m
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    return [(px + c * u - s * v, py + s * u + c * v) for u, v in corners]


def bounds_polygon(bounds):
    """축 정렬 경계 [xmin, ymin, xmax, ymax]를 꼭짓점 4개로 바꾼다."""
    x0, y0, x1, y1 = checked_bounds(bounds)
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def polygons_overlap(a, b):
    # 중심점이 아니라 외곽끼리 비교합니다. 경계에 닿아 있으면 완전 이탈로 처리하지 않습니다.
    """볼록 다각형 두 개가 겹치거나 닿는지(분리축 정리). 닿기만 해도 True다."""
    for polygon in (a, b):
        for i in range(len(polygon)):
            (x0, y0), (x1, y1) = polygon[i], polygon[(i + 1) % len(polygon)]
            nx, ny = y1 - y0, x0 - x1

            projection_a = [nx * x + ny * y for x, y in a]
            projection_b = [nx * x + ny * y for x, y in b]

            scale = math.hypot(nx, ny)
            if (max(projection_a) < min(projection_b) - TOUCH_EPS_M * scale
                    or max(projection_b) < min(projection_a) - TOUCH_EPS_M * scale):
                return False

    return True


def angle_difference(a, b):
    """두 각도 차의 절댓값(0~π)"""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def observe_robot(catalog, outline_bounds, pose, *, frame_id, map_revision, position_valid,
                  outline_complete, age_s, max_age_s=1.0, uncertainty_m=0.0, yaw_uncertainty_rad=0.0,
                  heading_tolerance_rad=HEADING_TOLERANCE_RAD, future_tolerance_s=0.0, degraded=False):
    """로봇 1대의 모든 통로 관측을 {corridor_id: Observation}으로 돌려준다.

    - outline_bounds: 차체(base_link) 좌표 외곽 [xmin, ymin, xmax, ymax]. 접힌 팔·적재물을 포함한 값이어야 한다.
    - pose: map 좌표 (x, y, yaw). 외곽을 yaw로 회전한 다각형으로 판정한다(축 정렬 근사로 넓히지 않음).
    - 위치 불확실성: uncertainty_m + yaw_uncertainty_rad × (외곽 최대 반지름)만큼 외곽을 넓힌다.
    - 통로와 겹치면 INSIDE. 아니면 그 통로의 접근 구역과 겹치고 차체 방향이 진입 방향에서
      heading_tolerance_rad + yaw_uncertainty_rad 안이면 APPROACHING. 아니면 OUTSIDE.
    - 판정 불가(미승인·좌표계/revision 불일치·위치 무효·만료·외곽 미확인·형상 오류)는 모든 통로 UNKNOWN.
    - age_s는 위치 표본 나이(s). future_tolerance_s까지의 음수(미래 stamp)는 허용한다.
    - degraded=True(위치 신뢰도 저하): INSIDE·APPROACHING을 확정하지 않는다. 넓힌 외곽(호출자가 여유를 더한 uncertainty_m)이
      그 통로와 그 통로의 접근 구역 어디와도 겹치지 않으면 OUTSIDE(position_valid=true), 겹치면 UNKNOWN(DEGRADED_NEAR)이다.
    """
    corridors = [r for r in catalog.get('regions', []) if r.get('kind') == 'corridor']

    def everywhere(reason):
        return {r['id']: Observation('UNKNOWN', reason) for r in corridors}

    # 1) 공통 판정 불가 조건
    if not catalog.get('approved'):
        return everywhere('REGIONS_NOT_APPROVED')
    if frame_id != catalog['frame_id'] or map_revision != catalog['map_revision']:
        return everywhere('FRAME_OR_MAP_MISMATCH')
    numbers = (age_s, max_age_s, uncertainty_m, yaw_uncertainty_rad, heading_tolerance_rad, future_tolerance_s, *pose)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in numbers):
        return everywhere('POSITION_OR_OUTLINE_INVALID')
    if (not position_valid or not outline_complete or max_age_s <= 0 or future_tolerance_s < 0
            or not -future_tolerance_s <= age_s <= max_age_s):
        return everywhere('POSITION_OR_OUTLINE_INVALID')
    if uncertainty_m < 0 or yaw_uncertainty_rad < 0 or not 0 < heading_tolerance_rad < math.pi:
        return everywhere('UNCERTAINTY_INVALID')

    # 2) 불확실성만큼 넓힌 회전 외곽
    try:
        x0, y0, x1, y1 = checked_bounds(outline_bounds)
        radius = max(math.hypot(x, y) for x in (x0, x1) for y in (y0, y1))
        footprint = outline_polygon(outline_bounds, pose, uncertainty_m + yaw_uncertainty_rad * radius)
    except ValueError:
        return everywhere('GEOMETRY_INVALID')

    # 3) 통로별 판정
    # OUTSIDE는 위치 관측일 뿐입니다. 실제 통로 허가·해제는 별도 D3 판단을 거쳐야 합니다.
    result = {}
    for corridor in corridors:
        if corridor.get('bounds') is None:
            result[corridor['id']] = Observation('UNKNOWN', 'CORRIDOR_UNDEFINED')
            continue

        zones = [z for z in catalog['regions'] if z.get('kind') == 'approach' and z.get('corridor_id') == corridor['id']
                 and z.get('bounds') is not None]

        # 위치 신뢰도 저하: 통로·접근 구역과 떨어졌을 때만 OUTSIDE, 아니면 UNKNOWN(점유로 취급)
        if degraded:
            near = any(polygons_overlap(footprint, bounds_polygon(r['bounds'])) for r in [corridor, *zones])
            result[corridor['id']] = (Observation('UNKNOWN', 'DEGRADED_NEAR') if near
                                      else Observation('OUTSIDE', 'DEGRADED_CLEAR', True))
            continue

        if polygons_overlap(footprint, bounds_polygon(corridor['bounds'])):
            result[corridor['id']] = Observation('INSIDE', 'GEOMETRY_CHECKED', True)
            continue

        approaching = False
        for zone in zones:
            facing = angle_difference(pose[2], zone['entry_heading_rad']) <= heading_tolerance_rad + yaw_uncertainty_rad
            if facing and polygons_overlap(footprint, bounds_polygon(zone['bounds'])):
                approaching = True
                break

        result[corridor['id']] = Observation('APPROACHING' if approaching else 'OUTSIDE', 'GEOMETRY_CHECKED', True)

    return result


# 명령행: 영역 파일을 검사해 요약을 출력하고, --require-approved면 미승인을 종료 코드 2로 막는다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--require-approved', action='store_true')
    args = parser.parse_args()
    if args.config is None:
        from ament_index_python.packages import get_package_share_directory
        args.config = Path(get_package_share_directory('ax_environment')) / 'config/ax_warehouse_regions.json'
    try:
        catalog = load_regions(args.config)
    except (ValueError, OSError) as error:
        parser.exit(2, f'영역 파일 오류: {error}\n')
    print(f"좌표계: {catalog['frame_id']} / 지도 revision: {catalog['map_revision']}")
    print(f"운영 승인: {catalog['approved']} / 사유: {catalog.get('reason', '')}")
    for region in catalog['regions']:
        extra = f" → {region['corridor_id']}, 진입 {region['entry_heading_rad']:+.3f} rad" if region['kind'] == 'approach' else ''
        print(f"{region['id']} ({region['kind']}): {region.get('bounds') or '좌표 미확정'}{extra}")
    if args.require_approved and not catalog['approved']:
        parser.exit(2, '미승인 영역이므로 운영 연결을 차단합니다.\n')


if __name__ == '__main__':
    main()
