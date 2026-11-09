"""B2 지도 작성·검증 공용 계산. ROS·Gazebo 없이 단위 시험한다.

- 2D 강체 변환(SE(2)), 자세 보간
- LiDAR 스캔을 점유·빈 공간·미관측으로 누적하는 점유 격자
- Nav2 형식 지도(YAML + PGM) 읽기·쓰기
- 두 지도의 정렬(ICP)과 양방향 형상 비교 지표

참조 지도는 Gazebo 실제 자세로 만든 검증 자료이며, 이 모듈의 결과를 SLAM이나 주행 입력으로 쓰지 않는다.
"""

import hashlib
import math
from pathlib import Path

import numpy as np
import yaml

# 점유 격자 값. Nav2 OccupancyGrid와 같은 의미다.
UNKNOWN = -1
FREE = 0
OCCUPIED = 100

# Nav2 trinary PGM 화소 값
PGM_OCCUPIED = 0
PGM_FREE = 254
PGM_UNKNOWN = 205


# 각도를 [-pi, pi)로 감는다.
def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# 사원수(x, y, z, w)에서 yaw(rad)를 구한다.
def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# (x, y, yaw)를 3×3 동차 변환으로 만든다.
def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)

    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


# 3×3 동차 변환을 (x, y, yaw)로 되돌린다.
def se2_params(transform):
    return float(transform[0, 2]), float(transform[1, 2]), math.atan2(transform[1, 0], transform[0, 0])


# 점 목록(N×2)에 변환을 적용한다.
def transform_points(transform, points):
    points = np.asarray(points, float).reshape(-1, 2)

    return points @ transform[:2, :2].T + transform[:2, 2]


# 시각 t의 자세를 앞뒤 표본으로 선형 보간한다. 범위 밖이거나 표본 간격이 max_gap보다 크면 None.
# times는 오름차순, poses는 N×3(x, y, yaw)이다.
def interpolate_pose(times, poses, t, max_gap):
    times = np.asarray(times, float)

    if len(times) == 0 or t < times[0] or t > times[-1]:
        return None

    index = int(np.searchsorted(times, t))

    if index < len(times) and times[index] == t:
        return tuple(float(v) for v in poses[index])

    before, after = index - 1, index

    if times[after] - times[before] > max_gap:
        return None

    ratio = (t - times[before]) / (times[after] - times[before])
    start, end = poses[before], poses[after]

    # yaw는 짧은 쪽으로 보간한다.
    yaw = start[2] + ratio * wrap_angle(end[2] - start[2])

    return (
        float(start[0] + ratio * (end[0] - start[0])),
        float(start[1] + ratio * (end[1] - start[1])),
        float(wrap_angle(yaw)),
    )


# 스캔 한 개를 빔별 (각도, 거리, 장애물 여부, 빈 공간 광선 여부)로 나눈다.
# - range_min~range_max 안의 유한값: 장애물. 그 앞은 빈 공간이다.
# - +inf 또는 range_max 초과: 장애물 없음. range_max까지 빈 공간이다.
# - NaN, -inf, range_min 미만: 판단 불가로 버린다(B1이 너무 가까움을 -inf로 표시).
def scan_rays(ranges, angle_min, angle_increment, range_min, range_max):
    ranges = np.asarray(ranges, float)
    angles = angle_min + angle_increment * np.arange(len(ranges))

    hit = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    beyond = (ranges == np.inf) | (np.isfinite(ranges) & (ranges > range_max))

    distances = np.where(hit, ranges, range_max)
    use = hit | beyond

    return angles[use], distances[use], hit[use]


# 격자 기하. origin은 (0, 0) 셀의 왼쪽 아래 모서리 좌표(m)이며 행은 +y, 열은 +x 방향이다.
class GridGeometry:
    def __init__(self, origin_x, origin_y, resolution, width, height):
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)

    # 점(N×2)의 셀 (행, 열) 정수 좌표
    def cells(self, points):
        points = np.asarray(points, float).reshape(-1, 2)
        columns = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(int)
        rows = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(int)

        return rows, columns

    # 셀이 격자 안에 있는지
    def inside(self, rows, columns):
        return (rows >= 0) & (rows < self.height) & (columns >= 0) & (columns < self.width)

    # 모든 셀 중심 좌표(높이×너비×2)
    def centers(self):
        xs = self.origin_x + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(self.height) + 0.5) * self.resolution

        return np.stack(np.meshgrid(xs, ys), axis=-1)

    # 메타데이터 기록용 사전
    def as_dict(self):
        return {
            "origin": [self.origin_x, self.origin_y],
            "resolution": self.resolution,
            "width": self.width,
            "height": self.height,
        }


# 스캔을 셀별 장애물·통과 횟수로 누적한다. 광선이 지나간 셀은 통과, 끝 셀은 장애물로 센다.
class OccupancyAccumulator:
    def __init__(self, geometry):
        self.geometry = geometry
        self.hits = np.zeros((geometry.height, geometry.width), np.int32)
        self.passes = np.zeros((geometry.height, geometry.width), np.int32)
        self.scans = 0

    # 센서 자세 T_frame_sensor로 스캔 한 개를 더한다.
    def add_scan(self, sensor_transform, angles, distances, hit):
        if len(angles) == 0:
            return

        geometry = self.geometry
        step = geometry.resolution * 0.5
        origin = sensor_transform[:2, 2]
        directions = np.stack([np.cos(angles), np.sin(angles)], axis=1) @ sensor_transform[:2, :2].T

        # 광선마다 끝점 바로 앞까지 반 셀 간격으로 표본을 찍어 통과 셀을 만든다.
        samples = []

        for direction, distance in zip(directions, distances):
            count = max(int((distance - geometry.resolution) / step), 0)

            if count:
                lengths = np.arange(count) * step
                samples.append(origin + lengths[:, None] * direction)

        if samples:
            self._count(np.concatenate(samples), self.passes)

        if np.any(hit):
            endpoints = origin + distances[hit, None] * directions[hit]
            self._count(endpoints, self.hits)

        self.scans += 1

    # 점이 속한 셀을 스캔당 한 번만 센다.
    def _count(self, points, counter):
        rows, columns = self.geometry.cells(points)
        inside = self.geometry.inside(rows, columns)
        linear = np.unique(rows[inside] * self.geometry.width + columns[inside])

        counter.flat[linear] += 1

    # 누적 횟수로 점유·빈 공간·미관측을 정한다.
    # 장애물 비율(hits / (hits + passes))이 occupied_ratio 이상이면 점유, 관측이 있으면 빈 공간, 없으면 미관측이다.
    def classify(self, occupied_ratio=0.25, min_hits=1):
        observed = self.hits + self.passes
        ratio = np.divide(self.hits, observed, out=np.zeros(observed.shape), where=observed > 0)

        grid = np.full(observed.shape, UNKNOWN, np.int8)
        grid[observed > 0] = FREE
        grid[(self.hits >= min_hits) & (ratio >= occupied_ratio)] = OCCUPIED

        return grid


# 파일의 SHA-256
def file_sha256(path):
    digest = hashlib.sha256()

    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)

    return digest.hexdigest()


# 이진 PGM(P5)을 읽는다.
def read_pgm(path):
    data = Path(path).read_bytes()
    fields = []
    position = 0

    # 헤더: P5, 너비, 높이, 최댓값. 주석(#)은 건너뛴다.
    while len(fields) < 4:
        while data[position:position + 1].isspace():
            position += 1

        if data[position:position + 1] == b"#":
            position = data.index(b"\n", position) + 1
            continue

        end = position

        while not data[end:end + 1].isspace():
            end += 1

        fields.append(data[position:end])
        position = end

    if fields[0] != b"P5" or int(fields[3]) > 255:
        raise ValueError(f"{path}: 8비트 P5 PGM만 지원")

    width, height = int(fields[1]), int(fields[2])
    pixels = np.frombuffer(data, np.uint8, width * height, position + 1)

    return pixels.reshape(height, width)


# 이진 PGM(P5)을 쓴다.
def write_pgm(path, pixels):
    pixels = np.asarray(pixels, np.uint8)
    header = f"P5\n{pixels.shape[1]} {pixels.shape[0]}\n255\n".encode()

    Path(path).write_bytes(header + pixels.tobytes())


# Nav2 지도 YAML과 PGM을 읽어 (격자, 기하, yaw)를 돌려준다. 격자 행은 +y 방향이다.
def read_map(yaml_path):
    yaml_path = Path(yaml_path)
    info = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    pixels = read_pgm(yaml_path.parent / info["image"])

    # 화소 → 점유 확률. negate=0이면 어두울수록 점유다.
    value = pixels.astype(float) / 255.0
    probability = value if info.get("negate", 0) else 1.0 - value

    grid = np.full(pixels.shape, UNKNOWN, np.int8)
    grid[probability > info["occupied_thresh"]] = OCCUPIED
    grid[probability < info["free_thresh"]] = FREE

    # PGM 첫 행은 지도 위쪽(+y 끝)이므로 뒤집는다.
    grid = grid[::-1].copy()
    origin = info["origin"]
    geometry = GridGeometry(origin[0], origin[1], info["resolution"], grid.shape[1], grid.shape[0])

    return grid, geometry, float(origin[2] if len(origin) > 2 else 0.0)


# 격자를 Nav2 trinary 지도(YAML + PGM)로 쓴다.
def write_map(yaml_path, grid, geometry, image_name=None):
    yaml_path = Path(yaml_path)
    image_name = image_name or yaml_path.with_suffix(".pgm").name

    pixels = np.full(grid.shape, PGM_UNKNOWN, np.uint8)
    pixels[grid == FREE] = PGM_FREE
    pixels[grid == OCCUPIED] = PGM_OCCUPIED
    write_pgm(yaml_path.parent / image_name, pixels[::-1])

    info = {
        "image": image_name,
        "mode": "trinary",
        "resolution": geometry.resolution,
        "origin": [geometry.origin_x, geometry.origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        # 미관측 205는 p=0.196이므로 그보다 작게 둬야 빈 공간으로 읽히지 않는다.
        "free_thresh": 0.196,
    }
    yaml_path.write_text(yaml.safe_dump(info, sort_keys=False), encoding="utf-8")


# 실제 사용한 robot_description(URDF 문자열)에서 바퀴 접촉 마찰과 LiDAR 설정을 뽑는다.
# 값이 없는 항목은 None으로 두며 임의 기본값을 채우지 않는다.
def describe_model(urdf_text):
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf_text)
    text = lambda element, path: (element.findtext(path) or "").strip() or None
    number = lambda value: float(value) if value is not None else None

    wheels, lidars = {}, {}

    for gazebo in root.iter("gazebo"):
        reference = gazebo.get("reference")

        # 바퀴 접촉: <gazebo reference>의 mu1·mu2·fdir1과 collision 마찰 블록의 slip1·slip2
        if gazebo.find("mu1") is not None or gazebo.find("mu2") is not None:
            ode = "collision/surface/friction/ode/"
            wheels[reference] = {
                "mu1": number(text(gazebo, "mu1")),
                "mu2": number(text(gazebo, "mu2")),
                "fdir1": text(gazebo, "fdir1"),
                "slip1": number(text(gazebo, ode + "slip1")),
                "slip2": number(text(gazebo, ode + "slip2")),
            }

        for sensor in gazebo.iter("sensor"):
            if sensor.get("type") not in ("gpu_lidar", "lidar"):
                continue

            scan = "lidar/scan/horizontal/"
            lidars[sensor.get("name")] = {
                "link": reference,
                "frame": text(sensor, "gz_frame_id"),
                "update_rate_hz": number(text(sensor, "update_rate")),
                "samples": number(text(sensor, scan + "samples")),
                "min_angle_rad": number(text(sensor, scan + "min_angle")),
                "max_angle_rad": number(text(sensor, scan + "max_angle")),
                "range_min_m": number(text(sensor, "lidar/range/min")),
                "range_max_m": number(text(sensor, "lidar/range/max")),
                "noise_stddev_m": number(text(sensor, "lidar/noise/stddev")),
            }

    return {"robot_name": root.get("name"), "wheel_contacts": wheels, "lidars": lidars}


# 저장 지도의 좌표계·원점·해상도·크기와 관측(빈 공간·점유)·미관측 셀 수·면적을 정리한다.
def map_summary(yaml_path):
    yaml_path = Path(yaml_path)
    info = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    grid, geometry, yaw = read_map(yaml_path)
    cell_area = geometry.resolution ** 2

    counts = {"free": int((grid == FREE).sum()), "occupied": int((grid == OCCUPIED).sum()),
              "unknown": int((grid == UNKNOWN).sum())}
    observed = grid != UNKNOWN
    summary = {
        "frame": "map",
        "origin": [float(value) for value in info["origin"]],
        "origin_note": "map.yaml origin = 이미지 왼쪽 아래 셀 모서리의 map 좌표(x, y, yaw)",
        "resolution_m": geometry.resolution,
        "size_cells": [geometry.width, geometry.height],
        "size_m": [geometry.width * geometry.resolution, geometry.height * geometry.resolution],
        "thresholds": {"occupied": info["occupied_thresh"], "free": info["free_thresh"], "mode": info.get("mode")},
        "cells": counts,
        "area_m2": {name: value * cell_area for name, value in counts.items()},
        "observed_ratio": float(observed.mean()) if grid.size else None,
        "observed_bounds_map_m": None,
    }

    # 관측 셀을 감싸는 map 좌표 범위 [x_min, y_min, x_max, y_max]. 행 0이 이미지 아래(y 최소)라고 본다.
    if observed.any():
        rows, columns = np.nonzero(observed)
        # numpy 정수 인덱스 계산 결과(numpy.float64)는 yaml.safe_dump가 쓰지 못하므로 float로 바꾼다.
        summary["observed_bounds_map_m"] = [float(value) for value in (
            geometry.origin_x + columns.min() * geometry.resolution, geometry.origin_y + rows.min() * geometry.resolution,
            geometry.origin_x + (columns.max() + 1) * geometry.resolution, geometry.origin_y + (rows.max() + 1) * geometry.resolution,
        )]

    return summary


# 점유 셀 중심 좌표(N×2)
def occupied_points(grid, geometry):
    rows, columns = np.nonzero(grid == OCCUPIED)

    return np.stack([
        geometry.origin_x + (columns + 0.5) * geometry.resolution,
        geometry.origin_y + (rows + 0.5) * geometry.resolution,
    ], axis=1)


# 점 집합 source를 target에 맞추는 2D 강체 변환을 ICP로 찾는다.
# 대응점 거리가 중앙값의 outlier_factor배(최소 min_inlier_m)를 넘으면 버려 가림·미관측 차이의 영향을 줄인다.
# 고정 비율로 자르면 짧은 벽 같은 정보가 큰 대응점이 늘 빠져 수렴이 멈추므로 쓰지 않는다.
def align_icp(source, target, initial, iterations=100, max_distance=0.5, outlier_factor=3.0, min_inlier_m=0.1,
              tolerance=1e-7):
    from scipy.spatial import cKDTree

    source = np.asarray(source, float)
    target = np.asarray(target, float)

    if len(source) < 3 or len(target) < 3:
        raise ValueError("정렬할 점유 셀이 부족함")

    tree = cKDTree(target)
    transform = np.array(initial, float)
    error = math.inf

    for _ in range(iterations):
        moved = transform_points(transform, source)
        distance, index = tree.query(moved, distance_upper_bound=max_distance)

        # 거리 제한 안의 대응점 중 중앙값 기준 이상치를 버린다.
        valid = np.isfinite(distance)

        if valid.sum() < 3:
            raise ValueError("정렬 대응점이 부족함")

        limit = max(outlier_factor * float(np.median(distance[valid])), min_inlier_m)
        keep = valid & (distance <= limit)

        # 대응점 쌍으로 최소제곱 회전·이동(Kabsch)을 구한다.
        a, b = moved[keep], target[index[keep]]
        mean_a, mean_b = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a - mean_a).T @ (b - mean_b))
        rotation = (u @ vt).T

        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = (u @ vt).T

        step = np.eye(3)
        step[:2, :2] = rotation
        step[:2, 2] = mean_b - rotation @ mean_a
        transform = step @ transform

        new_error = float(np.sqrt(np.mean(np.sum((a @ rotation.T + step[:2, 2] - b) ** 2, axis=1))))

        if abs(error - new_error) < tolerance:
            error = new_error
            break

        error = new_error

    return transform, {"rmse_m": error, "pairs": int(keep.sum()), "source_points": int(len(source))}


# 다른 격자를 reference 기하로 옮긴다. transform은 T_reference_other이고 최근접 셀 값을 쓴다.
def resample(grid, geometry, reference_geometry, transform):
    centers = reference_geometry.centers().reshape(-1, 2)
    points = transform_points(np.linalg.inv(transform), centers)

    rows, columns = geometry.cells(points)
    inside = geometry.inside(rows, columns)

    values = np.full(len(points), UNKNOWN, np.int8)
    values[inside] = grid[rows[inside], columns[inside]]

    return values.reshape(reference_geometry.height, reference_geometry.width)


# 마스크 셀에서 가장 가까운 target 셀까지 거리(m) 통계. target이 비면 None.
def distance_stats(mask, target, resolution):
    import cv2

    if not target.any() or not mask.any():
        return None

    # target 셀이 0인 영상의 거리 변환 → 각 셀에서 가장 가까운 target까지 화소 거리
    field = cv2.distanceTransform(np.where(target, 0, 255).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    values = field[mask] * resolution

    return {
        "mean_m": float(values.mean()),
        "p95_m": float(np.percentile(values, 95)),
        "max_m": float(values.max()),
        "cells": int(mask.sum()),
    }


# 정사각형 이웃(±cells)으로 마스크를 넓힌다.
def dilate(mask, cells):
    import cv2

    kernel = np.ones((2 * cells + 1, 2 * cells + 1), np.uint8)

    return cv2.dilate(mask.astype(np.uint8), kernel) > 0


# 참조 지도와 비교 지도(reference 기하로 옮긴 값)의 양방향 형상 지표를 계산한다.
# 한쪽 점유 셀은 상대 지도가 관측한 영역(±tolerance 셀 포함) 안에 있을 때만 비교 대상으로 센다.
# 상대 점유는 관측 마스크 없이 찾는다. 벽이 1셀 밀리면 그 셀이 상대 지도에서 미관측일 수 있기 때문이다.
def compare_grids(reference, compared, resolution, tolerance_cells=1):
    known = (reference != UNKNOWN) & (compared != UNKNOWN)
    reference_all = reference == OCCUPIED
    compared_all = compared == OCCUPIED
    reference_occupied = reference_all & dilate(compared != UNKNOWN, tolerance_cells)
    compared_occupied = compared_all & dilate(reference != UNKNOWN, tolerance_cells)

    # 장애물 검출 비율: 비교 대상 참조 점유 셀 중 비교 지도 점유가 ±tolerance 셀 안에 있는 비율
    detected = reference_occupied & dilate(compared_all, tolerance_cells)
    matched = compared_occupied & dilate(reference_all, tolerance_cells)
    cell_area = resolution * resolution

    return {
        "tolerance_cells": tolerance_cells,
        "obstacle_detection_ratio": float(detected.sum() / reference_occupied.sum()) if reference_occupied.any() else None,
        "reverse_match_ratio": float(matched.sum() / compared_occupied.sum()) if compared_occupied.any() else None,
        "reference_occupied_cells": int(reference_occupied.sum()),
        "compared_occupied_cells": int(compared_occupied.sum()),
        "wall_error_reference_to_compared": distance_stats(reference_occupied, compared_all, resolution),
        "wall_error_compared_to_reference": distance_stats(compared_occupied, reference_all, resolution),
        "coverage": {
            "comparable_area_m2": float(known.sum() * cell_area),
            "reference_only_area_m2": float(((reference != UNKNOWN) & (compared == UNKNOWN)).sum() * cell_area),
            "compared_only_area_m2": float(((compared != UNKNOWN) & (reference == UNKNOWN)).sum() * cell_area),
            "unobserved_by_both_area_m2": float(((reference == UNKNOWN) & (compared == UNKNOWN)).sum() * cell_area),
        },
    }
