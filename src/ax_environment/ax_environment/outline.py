"""로봇 모델별 전체 외곽(base_link 좌표 축 정렬 사각형)을 생성 자료에서 계산한다.

- AX_1: ax1_spec.json visual_bounds_base_m(관절 0 자세, epick)과 a1_validation.yaml mobility.robot_1.footprint_xy_m
  (이동 자세 travel_pose_rad, 센서 포함 collision AABB + 약 20 mm)의 합집합. 이동 중 팔이 이동 자세 또는 0 자세라고 가정한다.
- AX_2: ax2_spec.json visual_bounds_base_m과 mobility.robot_2.footprint_xy_m(적재함·전후 센서 + 약 30 mm)의 합집합.
  적재물은 cargo_bounds_xy_m(갑판 안 ±0.34 m) 안에 있다고 가정하며, 이 범위는 외곽 안에 들어간다. 갑판 밖으로 나온 적재물은 포함하지 않는다.
- 임의 작업 자세(팔을 뻗은 상태)는 포함하지 않는다. 추가 여유는 margin_m으로 넓힌다.
"""
import json
import math
from pathlib import Path

import yaml

# 모델 이름 → (spec 파일, a1_validation mobility 키)
MODELS = {'AX_1': ('ax1_spec.json', 'robot_1'), 'AX_2': ('ax2_spec.json', 'robot_2')}


# 사각형 여러 개의 합집합 포위 사각형 [xmin, ymin, xmax, ymax]
def union(boxes):
    return [min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes)]


# 모델 외곽을 계산해 (경계, 근거 문자열)을 돌려준다. config_dir는 ax_description의 config 폴더다.
def model_outline(model, config_dir, margin_m=0.0):
    # 로봇 번호 대신 AX_1/AX_2 모델을 명시합니다. 정해진 팔 자세·적재 범위 가정 안의 외곽입니다.
    if model not in MODELS:
        raise ValueError(f'알 수 없는 로봇 모델: {model!r} (AX_1 또는 AX_2)')
    if not math.isfinite(margin_m) or margin_m < 0:
        raise ValueError('외곽 여유는 0 이상의 유한한 값이어야 합니다.')

    spec_name, mobility_key = MODELS[model]
    config_dir = Path(config_dir)

    # 1) visual_bounds_base_m: [[xmin, ymin, zmin], [xmax, ymax, zmax]]
    spec = json.loads((config_dir / spec_name).read_text(encoding='utf-8'))
    low, high = spec['visual_bounds_base_m']
    visual = [low[0], low[1], high[0], high[1]]

    # 2) 이동 자세 footprint 다각형의 포위 사각형
    validation = yaml.safe_load((config_dir / 'a1_validation.yaml').read_text(encoding='utf-8'))
    points = validation['mobility'][mobility_key]['footprint_xy_m']
    footprint = [min(p[0] for p in points), min(p[1] for p in points), max(p[0] for p in points), max(p[1] for p in points)]

    boxes = [visual, footprint]

    # 3) AX_2 적재물 허용 범위(갑판 기준)도 합친다(이미 외곽 안이면 변화 없음).
    if model == 'AX_2':
        bounds = validation['mobility'][mobility_key].get('cargo_bounds_xy_m')
        if bounds:
            boxes.append([bounds[0], bounds[2], bounds[1], bounds[3]])

    outline = union(boxes)
    outline = [outline[0] - margin_m, outline[1] - margin_m, outline[2] + margin_m, outline[3] + margin_m]
    basis = (f'{spec_name} visual_bounds_base_m ∪ a1_validation.yaml mobility.{mobility_key}.footprint_xy_m'
             f'{" ∪ cargo_bounds_xy_m" if model == "AX_2" else ""} + 여유 {margin_m} m')
    return [round(v, 6) for v in outline], basis
