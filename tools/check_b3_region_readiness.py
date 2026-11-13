#!/usr/bin/env python3
"""B3 승인 전 정적 근거를 점검합니다. 설정 변경·실제 TF 검증·운영 승인은 하지 않습니다."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/ax_environment'))
from ax_environment.outline import model_outline
from ax_environment.regions import load_regions, observe_robot
import generate_b3_regions as generator


def boundary_cases(catalog, outline, model):
    """실제 판정 함수로 두 입구의 접근·접촉·진입·완전 이탈 경계를 계산합니다."""
    fixture = deepcopy(catalog)
    fixture['approved'] = True  # 계산용 사본만 활성화하며 원본 파일은 쓰지 않습니다.
    cases = []
    for corridor in fixture['regions']:
        if corridor['kind'] != 'corridor':
            continue
        x0, y0, x1, y1 = corridor['bounds']
        y = (y0 + y1) / 2
        for yaw, side in ((0.0, 'west'), (math.pi, 'east')):
            # 중심점이 아닌 로봇 전체 외곽의 앞·뒤 끝으로 진입 접촉과 완전 이탈을 계산합니다.
            low, high = (outline[0], outline[2]) if yaw == 0 else (-outline[2], -outline[0])
            if side == 'west':
                points = [('outside', x0 - high - catalog['approach_depth_m'] - 1, 'OUTSIDE'),
                          ('approach', x0 - high - 0.02, 'APPROACHING'),
                          ('entry_contact', x0 - high, 'INSIDE'),
                          ('entered', x0 - high + 0.02, 'INSIDE'),
                          ('exit_overlap', x1 - low - 0.02, 'INSIDE'),
                          ('full_exit', x1 - low + 0.02, 'OUTSIDE')]
            else:
                points = [('outside', x1 - low + catalog['approach_depth_m'] + 1, 'OUTSIDE'),
                          ('approach', x1 - low + 0.02, 'APPROACHING'),
                          ('entry_contact', x1 - low, 'INSIDE'),
                          ('entered', x1 - low - 0.02, 'INSIDE'),
                          ('exit_overlap', x0 - high + 0.02, 'INSIDE'),
                          ('full_exit', x0 - high - 0.02, 'OUTSIDE')]
            for phase, x, expected in points:
                result = observe_robot(
                    fixture, outline, (x, y, yaw), frame_id=fixture['frame_id'],
                    map_revision=fixture['map_revision'], position_valid=True,
                    outline_complete=True, age_s=0.0)[corridor['id']]
                cases.append(dict(model=model, corridor=corridor['id'], side=side, phase=phase,
                                  pose=[x, y, yaw], expected=expected, actual=result.state,
                                  passed=result.state == expected and result.position_valid))
    return cases


def audit():
    """저장된 지도·월드·외곽 근거를 대조하고 실제 관측이 필요한 항목을 분리합니다."""
    regions_path = ROOT / 'src/ax_environment/config/ax_warehouse_regions.json'
    catalog = load_regions(regions_path)
    map_dir = ROOT / 'src/ax_localization/maps/ax_warehouse'
    info = yaml.safe_load((map_dir / 'map_info.yaml').read_text())
    applicable = yaml.safe_load((map_dir / 'applicable_worlds.yaml').read_text())
    checks = [dict(name='generated_regions_match_sdf', passed=catalog == generator.build()),
              dict(name='map_frame_revision', passed=(
                  catalog['frame_id'] == info['map']['frame']
                  and catalog['map_revision'] == info['map_revision']
                  == applicable['map_revision']))]
    for filename in ('map.yaml', 'map.pgm'):
        # 지도 이름과 revision이 같아도 실제 파일 내용이 바뀌었으면 탐지합니다.
        checks.append(dict(name='map_hash:' + filename, passed=(hashlib.sha256(
            (map_dir / filename).read_bytes()).hexdigest() == info['files'][filename])))
    transform = catalog['transform']['T_world_map']
    # 이 값은 저장된 검증 기준과의 일치만 확인합니다. 실제 TF 측정이나 운영 승인이 아닙니다.
    initial = info['T_world_map']['initial']
    checks.append(dict(name='reference_transform_matches_map_metadata', passed=all(
        math.isclose(value, initial[key], abs_tol=1e-6) for value, key in zip(
            transform, ('x_m', 'y_m', 'yaw_rad')))))
    for world in applicable['applicable_worlds']:
        path = ROOT / 'src/ax_description/worlds' / world['launch_name']
        checks.append(dict(name='world_hash:' + world['launch_name'], passed=(
            path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == world['sha256'])))
    outlines, cases = {}, []
    for model in ('AX_1', 'AX_2'):
        outline, basis = model_outline(model, ROOT / 'src/ax_description/config')
        outlines[model] = dict(bounds=outline, basis=basis)
        cases.extend(boundary_cases(catalog, outline, model))
    # 정적 통과와 운영 승인 가능 여부를 분리해 계산 시험을 실제 관측 검증으로 오해하지 않게 합니다.
    return dict(
        region_file=str(regions_path.relative_to(ROOT)), approved=catalog['approved'],
        static_passed=all(row['passed'] for row in checks + cases),
        ready_for_approval=False, checks=checks, outlines=outlines, boundary_cases=cases,
        configured_world_map_transform=catalog['transform'],
        transform_usage=info['T_world_map']['use'],
        pending=[
            'B2: 실제 채택 지도 해시·map_revision·world-map 정합과 잔여 오차 확인',
            'B2/B3: 같은 ROS 시계의 실제 map→odom→base TF 연속성·지연·위치 신뢰성 검증',
            'A1/A2/A4: 실제 접힌 팔·적재물 외곽 및 오차 여유 확인',
            'B3/D1/D3/A4: 두 로봇·양방향 진입/완전 이탈 실제 관측과 경계·대기 지점 합의',
            'D2: 작업대·등록 지점 ID와 용량·접근 조건 확인',
        ],
        limitations='정적 형상 계산만 검증. T_world_map 초기값은 B2 문서상 검증 전용이며 '
                    '운영 좌표 정합 근거로 사용할 수 없음. 실제 지도 픽셀 정합·TF·센서·주행을 검증하지 않음. '
                    '관절 임의 자세·돌출 적재물·운영 불확실성 여유를 보장하지 않음.')


def main():
    """정적 검증 보고서를 출력하며 승인 파일은 변경하지 않습니다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    arguments = parser.parse_args()
    report = audit()
    if arguments.output:
        # 승인 설정은 쓰지 않고 별도 보고서만 저장합니다.
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(static_passed=report['static_passed'],
                         boundary_cases=len(report['boundary_cases']),
                         approved=report['approved'], ready_for_approval=False), ensure_ascii=False))
    return 0 if report['static_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
