#!/usr/bin/env python3
"""창고 질감 이미지를 절차적으로 만든다(외부 이미지 없음).

질감은 기존 재질 색(diffuse·ambient)에 곱해지는 밝기 변조 이미지로 설계했다.
평균 밝기를 0.85~0.95 부근에 두어 기존 색감을 크게 바꾸지 않는다.

- 콘크리트 바닥: 얼룩·골재 반점·미세 입자·줄눈(약 6 m)·잔금·바퀴 자국
- 벽 패널: 세로 패널 이음매(1 m)·고정 볼트·하단 오염띠·긁힘·구역 표지판
- 선반 기둥: 50 mm 간격 구멍(랙 업라이트 슬롯)·도장 마모
- 선반판: 합판·강판 입자, 이음매, 사용 마모
- 운반 상자: 골판지 결·크기별 라벨(AX-180 등), 샘플 상자: 대비가 낮은 결

사용: python3 tools/gen_textures.py <월드 경로> <빈 출력 폴더> [--seed N] [--ppm 픽셀/m] [옵션]
출력: <출력 폴더>/*.png|*.jpg, <출력 폴더>/textures_manifest.json(대상 모델·크기·시드·해시)
옵션 기본값은 PNG 초판(v1) 설정이다. 저장소의 질감 월드(worlds/planned/textures)는 v2_slim 설정으로 만든다.
  --ppm 220 --upright-ppm 512 --deck-ppm 256 --box-px 512 --sample-px 256 --format jpg --jpg-quality 92
출력 폴더에 같은 이름의 공유 질감이 이미 있으면 다시 쓰지 않으므로 빈 폴더를 쓴다.
"""
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np


# 해상도 상한(px). 큰 면도 이 크기를 넘지 않게 해 GPU 메모리를 제한한다.
MAX_SIDE = 4096


# 여러 크기의 부드러운 잡음을 합쳐 0~1 범위의 얼룩 장을 만든다.
def fractal_noise(rng, height, width, cell_sizes, weights):
    field = np.zeros((height, width), np.float32)

    for cell, weight in zip(cell_sizes, weights):
        small_h = max(2, int(round(height / cell)) + 2)
        small_w = max(2, int(round(width / cell)) + 2)
        small = rng.random((small_h, small_w)).astype(np.float32)
        field += weight * cv2.resize(small, (width, height), interpolation=cv2.INTER_CUBIC)

    field -= field.min()
    return field / max(float(field.max()), 1e-6)


# 회색조 밝기 배열(0~1)을 약간의 색조와 함께 8bit RGB로 바꾼다.
def to_rgb(value, tint=(1.0, 1.0, 1.0)):
    value = np.clip(value, 0.0, 1.0)
    channels = [np.clip(value * tint[i], 0.0, 1.0) for i in (2, 1, 0)]
    return (np.dstack(channels) * 255.0 + 0.5).astype(np.uint8)


# 콘크리트 바닥 질감. ppm은 m당 픽셀 수, 크기는 바닥 윗면(x, y) m다.
def concrete(rng, size_x, size_y, ppm):
    width, height = scaled_size(size_x, size_y, ppm)
    px_per_m = width / size_x

    # 넓은 얼룩(1~3 m)과 중간 무늬(10~40 cm)
    mottling = fractal_noise(
        rng, height, width,
        cell_sizes=[px_per_m * 3.0, px_per_m * 1.0, px_per_m * 0.3, px_per_m * 0.1],
        weights=[0.45, 0.30, 0.15, 0.10],
    )
    value = 0.80 + 0.12 * mottling

    # 미세 입자(1 px)와 골재 반점(2~4 px)
    value += rng.normal(0.0, 0.025, (height, width)).astype(np.float32)
    spots = rng.random((height, width)) < 0.004
    spot_layer = np.zeros((height, width), np.float32)
    spot_layer[spots] = rng.uniform(-0.10, 0.06, int(spots.sum()))
    value += cv2.GaussianBlur(spot_layer, (0, 0), 1.0) * 2.5

    # 기름·물 얼룩: 부드러운 타원형 어두운 영역
    stain = np.zeros((height, width), np.float32)
    for _ in range(int(size_x * size_y / 12) + 2):
        center = (int(rng.uniform(0, width)), int(rng.uniform(0, height)))
        axes = (int(rng.uniform(0.2, 1.0) * px_per_m), int(rng.uniform(0.1, 0.6) * px_per_m))
        cv2.ellipse(stain, center, axes, float(rng.uniform(0, 180)), 0, 360, float(rng.uniform(0.04, 0.10)), -1)
    value -= cv2.GaussianBlur(stain, (0, 0), px_per_m * 0.12)

    # 바퀴 자국: 완만한 곡선의 흐린 줄
    tracks = np.zeros((height, width), np.float32)
    for _ in range(int(size_x * size_y / 25) + 1):
        start = np.array([rng.uniform(0, width), rng.uniform(0, height)])
        heading = rng.uniform(0, 2 * np.pi)
        points = []
        for _ in range(60):
            heading += rng.normal(0, 0.03)
            start = start + px_per_m * 0.1 * np.array([np.cos(heading), np.sin(heading)])
            points.append(start.copy())
        cv2.polylines(tracks, [np.int32(points)], False, float(rng.uniform(0.03, 0.06)), max(2, int(px_per_m * 0.12)))
    value -= cv2.GaussianBlur(tracks, (0, 0), px_per_m * 0.03)

    # 줄눈(절단 이음): 약 6 m 간격, 폭 약 8 mm의 어두운 선
    joint = np.zeros((height, width), np.float32)
    joint_width = max(1, int(round(px_per_m * 0.008)))
    for axis_len_m, vertical in ((size_x, True), (size_y, False)):
        count = max(1, int(round(axis_len_m / 6.0)))
        for k in range(1, count):
            pos = int(round(k * (width if vertical else height) / count))
            if vertical:
                cv2.line(joint, (pos, 0), (pos, height - 1), 0.30, joint_width)
            else:
                cv2.line(joint, (0, pos), (width - 1, pos), 0.30, joint_width)
    value -= cv2.GaussianBlur(joint, (0, 0), 0.7)

    # 가는 잔금: 짧은 무작위 보행선
    cracks = np.zeros((height, width), np.float32)
    for _ in range(int(size_x * size_y / 20) + 2):
        point = np.array([rng.uniform(0, width), rng.uniform(0, height)])
        heading = rng.uniform(0, 2 * np.pi)
        points = [point.copy()]
        for _ in range(int(rng.uniform(20, 80))):
            heading += rng.normal(0, 0.35)
            point = point + 3.0 * np.array([np.cos(heading), np.sin(heading)])
            points.append(point.copy())
        cv2.polylines(cracks, [np.int32(points)], False, 0.12, 1)
    value -= cracks

    return to_rgb(value, tint=(1.0, 0.99, 0.97))


# 표지판 글자를 그린다. 구역 번호처럼 실제 창고에서 벽에 붙이는 형태다.
def draw_sign(canvas, center_x, center_y, px_per_m, text, rng):
    sign_w = int(px_per_m * 0.60)
    sign_h = int(px_per_m * 0.32)
    x0, y0 = int(center_x - sign_w / 2), int(center_y - sign_h / 2)

    # 바탕은 약간 바랜 흰색, 테두리는 얇은 회색
    cv2.rectangle(canvas, (x0, y0), (x0 + sign_w, y0 + sign_h), float(rng.uniform(0.95, 1.0)), -1)
    cv2.rectangle(canvas, (x0, y0), (x0 + sign_w, y0 + sign_h), 0.55, max(1, int(px_per_m * 0.008)))

    # 글자 크기를 표지판 높이에 맞춘다(Hershey 글꼴, 외부 글꼴 파일 없음)
    scale = sign_h / 40.0
    thickness = max(1, int(px_per_m * 0.02))
    (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    if text_w > sign_w * 0.9:
        scale *= sign_w * 0.9 / text_w
        (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    origin = (int(center_x - text_w / 2), int(center_y + text_h / 2))
    cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, 0.18, thickness, cv2.LINE_AA)


# 벽 패널 질감. length_m은 벽의 긴 수평 길이, height_m은 높이, zone은 표지판 접두어다.
def wall_panel(rng, length_m, height_m, ppm, zone, first_number=1):
    width, height = scaled_size(length_m, height_m, ppm)
    px_per_m = width / length_m

    # 도장면의 아주 약한 얼룩과 미세 입자
    value = 0.90 + 0.05 * fractal_noise(
        rng, height, width, cell_sizes=[px_per_m * 1.5, px_per_m * 0.3], weights=[0.7, 0.3])
    value += rng.normal(0.0, 0.012, (height, width)).astype(np.float32)

    # 세로 패널 이음매(약 1 m 간격): 어두운 홈과 옆의 밝은 모서리
    panel_count = max(1, int(round(length_m / 1.0)))
    seam = max(1, int(round(px_per_m * 0.006)))
    for k in range(1, panel_count):
        x = int(round(k * width / panel_count))
        cv2.line(value, (x, 0), (x, height - 1), 0.55, seam)
        cv2.line(value, (x + seam + 1, 0), (x + seam + 1, height - 1), 0.97, 1)
        # 이음매를 따라 고정 볼트(약 0.6 m 간격)
        for y_m in np.arange(0.3, height_m, 0.6):
            y = int(height - y_m * px_per_m)
            cv2.circle(value, (x + 3 * seam, y), max(1, int(px_per_m * 0.006)), 0.62, -1)

    # 가로 보강선(높이 1.2 m) 한 줄: 낮은 대비
    y_girt = int(height - 1.2 * px_per_m)
    cv2.line(value, (0, y_girt), (width - 1, y_girt), 0.80, max(1, int(px_per_m * 0.01)))

    # 하단 오염띠(바닥에서 0~0.35 m로 갈수록 진해짐)
    rows = np.arange(height, dtype=np.float32)
    height_from_floor = (height - rows) / px_per_m
    grime = np.clip(1.0 - height_from_floor / 0.35, 0.0, 1.0) ** 1.5
    value -= (0.10 * grime)[:, None] * (0.7 + 0.3 * fractal_noise(
        rng, height, width, cell_sizes=[px_per_m * 0.4], weights=[1.0]))

    # 지게차·대차 긁힘: 낮은 높이의 짧은 수평선
    for _ in range(int(length_m * 1.5)):
        x = int(rng.uniform(0, width))
        y = int(height - rng.uniform(0.1, 0.6) * px_per_m)
        length = int(rng.uniform(0.05, 0.4) * px_per_m)
        cv2.line(value, (x, y), (x + length, y + int(rng.normal(0, 2))), float(rng.uniform(0.55, 0.75)), 1)

    # 구역 표지판: 약 4.5 m 간격, 높이 1.8 m. 긴 벽에만 둔다.
    signs = []
    if length_m >= 3.0:
        count = max(1, int(length_m // 4.5))
        for k in range(count):
            center_x = (k + 0.5) * width / count + rng.uniform(-0.2, 0.2) * px_per_m
            text = '%s-%02d' % (zone, first_number + k)
            draw_sign(value, center_x, height - 1.8 * px_per_m, px_per_m, text, rng)
            signs.append(text)

    return to_rgb(value, tint=(1.0, 1.0, 1.0)), signs


# 랙 기둥(업라이트) 질감: 50 mm 간격 슬롯과 도장 마모. 크기는 폭 0.08 m, 높이 2 m 기준이다.
def rack_upright(rng, width_m, height_m, px_per_m=1024.0):
    width = max(32, int(round(width_m * px_per_m)))
    height = int(round(height_m * px_per_m))

    value = 0.92 + 0.04 * fractal_noise(rng, height, width, cell_sizes=[64, 16], weights=[0.6, 0.4])
    value += rng.normal(0.0, 0.01, (height, width)).astype(np.float32)

    # 슬롯: 폭 10 mm, 높이 25 mm, 50 mm 간격(어두운 구멍)
    slot_w, slot_h, pitch = int(0.010 * px_per_m), int(0.025 * px_per_m), int(0.050 * px_per_m)
    for y in range(pitch // 2, height - slot_h, pitch):
        x0 = width // 2 - slot_w // 2
        cv2.rectangle(value, (x0, y), (x0 + slot_w, y + slot_h), 0.28, -1)

    # 양쪽 모서리 접힘선과 아래쪽 도장 마모
    cv2.line(value, (2, 0), (2, height - 1), 0.78, 1)
    cv2.line(value, (width - 3, 0), (width - 3, height - 1), 0.78, 1)
    for _ in range(40):
        y = int(height - rng.uniform(0.0, 0.5) * px_per_m)
        cv2.circle(value, (int(rng.uniform(0, width)), y), int(rng.uniform(1, 4)), float(rng.uniform(1.0, 1.15)), -1)

    return to_rgb(value)


# 선반판 질감: 합판 입자와 판 이음매, 사용 마모. 크기는 윗면(x, y) m다.
def shelf_deck(rng, size_x, size_y, ppm):
    width, height = scaled_size(size_x, size_y, ppm)
    px_per_m = width / size_x

    value = 0.86 + 0.08 * fractal_noise(
        rng, height, width, cell_sizes=[px_per_m * 0.5, px_per_m * 0.05, px_per_m * 0.015],
        weights=[0.4, 0.35, 0.25])
    value += rng.normal(0.0, 0.03, (height, width)).astype(np.float32)

    # 판 이음매(약 0.9 m 간격)
    count = max(1, int(round(size_x / 0.9)))
    for k in range(1, count):
        x = int(round(k * width / count))
        cv2.line(value, (x, 0), (x, height - 1), 0.55, max(1, int(px_per_m * 0.004)))

    # 상자 끌림 자국: 앞뒤 방향의 흐린 줄
    for _ in range(12):
        x = int(rng.uniform(0, width))
        cv2.line(value, (x, 0), (x + int(rng.normal(0, 8)), height - 1), float(rng.uniform(0.75, 0.85)), 1)

    return to_rgb(value, tint=(1.0, 0.98, 0.95))


# 골판지 결: 한 방향의 가는 골(약 4 mm 간격) 그림자와 종이 섬유 얼룩. 밝기 변조 배열(평균 약 0.93)을 반환한다.
def corrugated_base(rng, size_px, flute_px, contrast):
    value = 0.92 + 0.05 * fractal_noise(rng, size_px, size_px, cell_sizes=[size_px / 3, size_px / 12, 6],
                                        weights=[0.5, 0.3, 0.2])
    value += rng.normal(0.0, 0.012, (size_px, size_px)).astype(np.float32)
    columns = np.arange(size_px, dtype=np.float32)
    value += (contrast * np.sin(2.0 * np.pi * columns / flute_px))[None, :]
    return value


# 운반 상자 질감(크기별 1장). 골판지 결, 가운데 포장 테이프, 송장 라벨(바코드·번호), 모서리 눌림을 넣는다.
# 상자 면마다 UV 0~1이라 모든 면에 같은 그림이 붙는다(실물은 라벨이 한 면에만 있음).
def cargo_box(rng, size_m, label_text, size_px=768):
    px_per_m = size_px / max(size_m)
    value = corrugated_base(rng, size_px, flute_px=max(3.0, 0.004 * px_per_m), contrast=0.012)

    # 포장 테이프: 폭 48 mm 띠, 약간 밝고 가장자리가 보이는 반투명 테이프
    tape_w = int(0.048 * px_per_m)
    y0 = size_px // 2 - tape_w // 2
    value[y0:y0 + tape_w, :] = value[y0:y0 + tape_w, :] * 0.6 + 0.42
    cv2.line(value, (0, y0), (size_px - 1, y0), 0.80, 1)
    cv2.line(value, (0, y0 + tape_w), (size_px - 1, y0 + tape_w), 0.80, 1)

    # 송장 라벨: 100×70 mm 흰 종이, 바코드와 번호 글자
    label_w, label_h = int(0.100 * px_per_m), int(0.070 * px_per_m)
    # 라벨은 면의 오른쪽 위, 가장자리에서 약 8 mm 안쪽에 둔다(작은 상자에서도 면 안에 들어가게).
    margin = int(0.008 * px_per_m)
    lx, ly = size_px - label_w - margin, margin
    cv2.rectangle(value, (lx, ly), (lx + label_w, ly + label_h), 1.0, -1)
    cv2.rectangle(value, (lx, ly), (lx + label_w, ly + label_h), 0.75, 1)
    bar_x, bar_y, bar_h = lx + label_w // 12, ly + label_h // 2, label_h // 3
    while bar_x < lx + label_w * 11 // 12:
        bar = int(rng.integers(1, 4))
        if rng.random() < 0.55:
            cv2.rectangle(value, (bar_x, bar_y), (bar_x + bar - 1, bar_y + bar_h), 0.15, -1)
        bar_x += bar + 1
    # 글자는 라벨 폭의 80 % 안에 들어가게 크기를 줄인다.
    scale = label_h / 90.0
    text_w = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, scale, max(1, int(scale * 2)))[0][0]
    if text_w > label_w * 0.8:
        scale *= label_w * 0.8 / text_w
    cv2.putText(value, label_text, (lx + label_w // 12, ly + label_h * 2 // 5), cv2.FONT_HERSHEY_SIMPLEX,
                scale, 0.2, max(1, int(scale * 2)), cv2.LINE_AA)
    for k in range(2):
        yy = ly + label_h * (5 + k) // 20 + label_h // 2 + bar_h
        cv2.line(value, (lx + label_w // 12, min(yy, ly + label_h - 3)), (lx + label_w // 2, min(yy, ly + label_h - 3)), 0.45, 1)

    # 모서리 눌림·닳음: 가장자리 근처의 약간 어두운 얼룩
    edge = np.zeros((size_px, size_px), np.float32)
    for _ in range(10):
        side = int(rng.integers(0, 4))
        t = int(rng.uniform(0, size_px))
        center = [(t, 0), (t, size_px - 1), (0, t), (size_px - 1, t)][side]
        cv2.circle(edge, center, int(rng.uniform(8, 30)), float(rng.uniform(0.04, 0.08)), -1)
    value -= cv2.GaussianBlur(edge, (0, 0), 6)

    return to_rgb(value, tint=(1.0, 1.0, 1.0))


# 분류 샘플 상자 질감(전체 공유 1장): 색을 바꾸지 않는 중립 회색조 골판지 결만 넣는다(평균 약 0.93, 낮은 대비).
def sample_grain(rng, size_px=512):
    value = corrugated_base(rng, size_px, flute_px=4.0, contrast=0.010)
    return to_rgb(value)


# 실제 크기(m)와 m당 픽셀로 이미지 크기를 정하고 큰 쪽을 MAX_SIDE로 제한한다.
def scaled_size(size_u, size_v, ppm):
    scale = min(ppm, MAX_SIDE / max(size_u, size_v))
    return int(round(size_u * scale)), int(round(size_v * scale))


# 월드의 대상 모델 목록과 상자 크기를 읽는다.
def target_models(world_path):
    world = ET.parse(world_path).getroot().find('world')
    targets = []

    for model in world.findall('model'):
        name = model.get('name')
        size = [float(v) for v in model.find('link/visual/geometry/box/size').text.split()]

        if name.startswith('floor_'):
            kind = 'concrete'
        elif name.startswith('wall_') or (name.startswith('corridor_') and '_wall_' in name):
            kind = 'wall'
        elif (name.startswith('rack_') or name.startswith('destination_rack_')) and '_leg_' in name:
            kind = 'rack_upright'
        elif (name.startswith('rack_') or name.startswith('destination_rack_')) and '_shelf_' in name:
            kind = 'shelf_deck'
        elif name.startswith('cargo_box_'):
            kind = 'cargo_box'
        elif name.startswith('sample_'):
            kind = 'sample'
        else:
            continue
        targets.append({'model': name, 'kind': kind, 'size_m': size})

    return targets


# 벽 이름에서 표지판 구역 접두어를 정한다(서쪽 창고 A, 동쪽 창고 B, 통로 C). 번호는 접두어별로 이어진다.
def zone_prefix(name):
    if name.startswith('corridor_'):
        return 'C' + name.split('_')[1]
    if 'source' in name:
        return 'A' + {'north': 'N', 'south': 'S', 'outer': 'W'}.get(name.split('_')[2], 'D')
    return 'B' + {'north': 'N', 'south': 'S', 'outer': 'E'}.get(name.split('_')[2], 'D')


# 대상별 질감을 만들고 manifest를 기록한다. 같은 종류의 랙 부품은 질감 하나를 공유한다.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('world')
    parser.add_argument('out')
    parser.add_argument('--seed', type=int, default=20261112)
    parser.add_argument('--ppm', type=float, default=220.0, help='바닥·벽 m당 픽셀')
    # 용량 축소판(v2_slim)용 옵션. 기본값은 v1과 같아 v1 재생성 결과가 바뀌지 않는다.
    parser.add_argument('--upright-ppm', type=float, default=1024.0, help='랙 기둥 m당 픽셀')
    parser.add_argument('--deck-ppm', type=float, default=700.0, help='선반판 m당 픽셀')
    parser.add_argument('--box-px', type=int, default=768, help='운반 상자 질감 한 변 픽셀')
    parser.add_argument('--sample-px', type=int, default=512, help='샘플 상자 질감 한 변 픽셀')
    parser.add_argument('--format', choices=['png', 'jpg'], default='png')
    parser.add_argument('--jpg-quality', type=int, default=85)
    args = parser.parse_args()
    ext = args.format
    write_params = [cv2.IMWRITE_JPEG_QUALITY, args.jpg_quality] if ext == 'jpg' else []

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    targets = target_models(args.world)
    shared = {}
    sign_numbers = {}
    records = []

    cargo_sizes = []
    env_index = -1

    for target in targets:
        name, kind, size = target['model'], target['kind'], target['size_m']
        # 시드 순번은 1차 대상(바닥·벽·랙·선반)만 센다. 상자 대상을 추가해도 1차 질감이 바뀌지 않게 한다.
        if kind not in ('cargo_box', 'sample'):
            env_index += 1
        index = env_index
        # 대상마다 시드를 고정 파생해 재생성 결과가 같게 한다.
        rng = np.random.default_rng([args.seed, index])
        extra = {}

        if kind == 'concrete':
            image = concrete(rng, size[0], size[1], args.ppm)
            filename = 'floor_%s.%s' % (name, ext)
        elif kind == 'wall':
            length = max(size[0], size[1])
            # 같은 접두어 안에서 번호가 겹치지 않게 이어서 매긴다(표지판 중복으로 인한 혼동 방지).
            prefix = zone_prefix(name)
            image, signs = wall_panel(rng, length, size[2], args.ppm, prefix, sign_numbers.get(prefix, 1))
            sign_numbers[prefix] = sign_numbers.get(prefix, 1) + len(signs)
            filename = 'wall_%s.%s' % (name, ext)
            extra['signs'] = signs
        elif kind == 'rack_upright':
            filename = 'rack_upright.' + ext
            if filename not in shared:
                shared[filename] = rack_upright(np.random.default_rng([args.seed, 9001]), size[0], size[2],
                                                args.upright_ppm)
            image = shared[filename]
        elif kind == 'cargo_box':
            # 같은 크기는 같은 질감·같은 라벨 번호(크기 순번으로 시드·번호를 정한다).
            size_key = 'x'.join('%d' % round(v * 1000) for v in size)
            if size_key not in cargo_sizes:
                cargo_sizes.append(size_key)
            size_index = cargo_sizes.index(size_key)
            filename = 'cargo_box_%s.%s' % (size_key, ext)
            extra['label'] = 'AX-%s' % size_key.split('x')[0]
            if filename not in shared:
                shared[filename] = cargo_box(np.random.default_rng([args.seed, 9100 + size_index]), size,
                                             extra['label'], args.box_px)
            image = shared[filename]
        elif kind == 'sample':
            filename = 'sample_grain.' + ext
            if filename not in shared:
                shared[filename] = sample_grain(np.random.default_rng([args.seed, 9200]), args.sample_px)
            image = shared[filename]
        else:
            filename = 'shelf_deck.' + ext
            if filename not in shared:
                shared[filename] = shelf_deck(np.random.default_rng([args.seed, 9002]), size[0], size[1], args.deck_ppm)
            image = shared[filename]

        path = out / filename
        if not path.exists() or filename not in shared:
            cv2.imwrite(str(path), image, write_params)
        records.append({**target, 'texture': filename, 'pixels_wh': [image.shape[1], image.shape[0]], **extra})

    files = sorted({record['texture'] for record in records})
    manifest = {
        'generator': Path(__file__).name,
        'seed': args.seed,
        'ppm_target': args.ppm,
        'max_side_px': MAX_SIDE,
        'upright_ppm': args.upright_ppm,
        'deck_ppm': args.deck_ppm,
        'box_px': args.box_px,
        'sample_px': args.sample_px,
        'format': ext,
        'jpg_quality': args.jpg_quality if ext == 'jpg' else None,
        'targets': records,
        'files': {
            name: {
                'bytes': (out / name).stat().st_size,
                'sha256': hashlib.sha256((out / name).read_bytes()).hexdigest(),
            }
            for name in files
        },
    }
    (out / 'textures_manifest.json').write_text(json.dumps(manifest, indent=1, ensure_ascii=False))
    print('targets', len(records), 'files', len(files), file=sys.stderr)


if __name__ == '__main__':
    main()
