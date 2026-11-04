#!/usr/bin/env python3
"""C1 학습용 RGB-D 영상과 YOLO 라벨 수집. 정답 pose는 학습 라벨에만 사용한다.

실행 중인 ax_test의 test_box와 전용 임시 카메라만 사용한다. 로봇/상자는
이동하지 않고 카메라는 종료 시 제거한다. 기본 월드·로봇 설정은 변경하지 않는다.
"""
import argparse
from collections import OrderedDict
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V

ROOT = Path(__file__).resolve().parents[1]


def request(action, kind, value):
    result = subprocess.run(['gz', 'service', '-s', '/world/ax_test/' + action,
        '--reqtype', kind, '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', value],
        capture_output=True, text=True, timeout=8)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError(result.stderr or result.stdout or action + ' failed')


def camera_rotation(position, target):
    forward = np.asarray(target) - position
    forward /= np.linalg.norm(forward)
    left = np.cross([0., 0., 1.], forward)
    left /= np.linalg.norm(left)
    return np.column_stack((forward, left, np.cross(forward, left)))


def box_label(depth, position, rotation, box_position, box_rotation, focal):
    """깊이에 보이는 표면만 상자 좌표로 변환해 가려진 영역을 라벨에서 제외한다."""
    h, w = depth.shape
    depth = np.where(np.isfinite(depth) & (depth > 0), depth, np.nan)
    right = (np.arange(w) - w / 2)[None, :] * depth / focal
    down = (np.arange(h) - h / 2)[:, None] * depth / focal
    points = np.stack((depth, -right, -down), axis=-1)
    local = (points @ rotation.T + position - box_position) @ box_rotation
    mask = np.isfinite(local).all(axis=-1) & (np.abs(local) <= .101).all(axis=-1)
    rows, cols = np.where(mask)
    if len(rows) < 64:
        return None
    x0, x1, y0, y1 = int(cols.min()), int(cols.max()) + 1, int(rows.min()), int(rows.max()) + 1
    if x1 - x0 < 10 or y1 - y0 < 10:
        return None
    return ((x0 + x1) / (2 * w), (y0 + y1) / (2 * h), (x1 - x0) / w, (y1 - y0) / h)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'vision_data/box_v1')
    parser.add_argument('--train', type=int, default=80)
    parser.add_argument('--val', type=int, default=20)
    parser.add_argument('--test', type=int, default=20)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    if min(args.train, args.val, args.test) < 5:
        parser.error('Each split needs at least 5 samples')
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('Output is not empty; select a new dataset name')
    if not os.environ.get('GZ_PARTITION'):
        # 바로가기 세션이면 이 작업공간의 실행 환경에서 partition을 가져온다.
        # 일반 ros2 launch는 기본 partition을 그대로 사용한다.
        from desktop_session import discover
        sessions = discover(ROOT)['launch']
        if len(sessions) > 1:
            parser.error('Multiple sessions found; set GZ_PARTITION explicitly')
        if sessions and sessions[0]['env'].get('GZ_PARTITION'):
            os.environ['GZ_PARTITION'] = sessions[0]['env']['GZ_PARTITION']
    node, lock = Node(), threading.Lock()
    frames, objects = OrderedDict(), {}
    name = 'c1_camera_' + uuid.uuid4().hex[:10]
    topic = '/ax/c1_collection/' + name

    def pose(message):
        for item in message.pose:
            if item.name == 'test_box':
                with lock:
                    objects['box'] = item

    def image(message, part):
        key = (message.header.stamp.sec, message.header.stamp.nsec)
        with lock:
            frames.setdefault(key, {})[part] = message
            while len(frames) > 12:
                frames.popitem(last=False)

    if not node.subscribe(Pose_V, '/world/ax_test/dynamic_pose/info', pose):
        raise RuntimeError('Cannot subscribe to world poses; check GZ_PARTITION')
    deadline = time.monotonic() + 10
    while not objects and time.monotonic() < deadline:
        time.sleep(.05)
    if not objects:
        raise RuntimeError('test_box not found in running ax_test; check GZ_PARTITION')
    focal = 320 / math.tan(math.pi / 6)
    sdf = f'''<sdf version="1.9"><model name="{name}"><static>true</static><link name="camera">
      <sensor name="rgbd" type="rgbd_camera"><topic>{topic}</topic><always_on>true</always_on>
      <update_rate>15</update_rate><camera><horizontal_fov>{math.pi/3}</horizontal_fov>
      <image><width>640</width><height>480</height><format>R8G8B8</format></image>
      <clip><near>0.05</near><far>10</far></clip><depth_camera><clip><near>0.05</near><far>5</far></clip></depth_camera>
      <lens><type>gnomonical</type><intrinsics><fx>{focal}</fx><fy>{focal}</fy><cx>320</cx><cy>240</cy><s>0</s></intrinsics></lens>
      </camera></sensor></link></model></sdf>'''
    request('create', 'gz.msgs.EntityFactory', 'sdf: ' + json.dumps(sdf))
    try:
        if not node.subscribe(Image, topic + '/image', lambda msg: image(msg, 'rgb')):
            raise RuntimeError('Cannot subscribe to collection RGB image')
        if not node.subscribe(Image, topic + '/depth_image', lambda msg: image(msg, 'depth')):
            raise RuntimeError('Cannot subscribe to collection depth image')
        for split_index, split in enumerate(('train', 'val', 'test')):
            rng = np.random.default_rng(args.seed + split_index)
            for part in ('images', 'labels'):
                (args.output / part / split).mkdir(parents=True, exist_ok=True)
            wanted, saved, tries = getattr(args, split), 0, 0
            while saved < wanted:
                tries += 1
                if tries > wanted * 5:
                    raise RuntimeError('Too many invisible/occluded samples; inspect the scene')
                # 평가 시점 영역은 학습에서 제외해 인접 프레임이 양쪽에 섞이지 않게 한다.
                if split == 'train':
                    angle = rng.uniform(-180, 180)
                    if 30 <= angle <= 70 or 120 <= angle <= 160:
                        continue
                else:
                    angle = rng.uniform(40, 60) if split == 'val' else rng.uniform(130, 150)
                with lock:
                    box = objects['box']
                center = np.array([box.position.x, box.position.y, box.position.z])
                box_rotation = Rotation.from_quat([box.orientation.x, box.orientation.y, box.orientation.z, box.orientation.w]).as_matrix()
                radius = rng.uniform(.65, 2.2)
                theta = math.radians(angle)
                position = center + [radius * math.cos(theta), radius * math.sin(theta), rng.uniform(.3, 1.1)]
                negative = saved % 5 == 0
                target = center + ([0, 0, -1.6] if negative else rng.uniform(-.06, .06, size=3))
                rotation = camera_rotation(position, target)
                q = Rotation.from_matrix(rotation).as_quat()
                request('set_pose', 'gz.msgs.Pose', f'name: "{name}" position: {{x: {position[0]} y: {position[1]} z: {position[2]}}} '
                        + f'orientation: {{x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]}}}')
                with lock:
                    frames.clear()
                deadline = time.monotonic() + 12
                pair = None
                while time.monotonic() < deadline:
                    with lock:
                        ready = [item for item in frames.values() if 'rgb' in item and 'depth' in item]
                        if len(ready) >= 3:
                            pair = ready[-1]
                    if pair:
                        break
                    time.sleep(.02)
                if pair is None:
                    raise RuntimeError('RGB-D collection timed out')
                rgb, depth = pair['rgb'], pair['depth']
                pixels = np.ndarray((rgb.height, rgb.width, 3), dtype='u1', buffer=rgb.data, strides=(rgb.step, 3, 1)).copy()
                distances = np.ndarray((depth.height, depth.width), dtype='<f4', buffer=depth.data, strides=(depth.step, 4))
                label = box_label(distances, position, rotation, center, box_rotation, focal)
                if (negative and label is not None) or (not negative and label is None):
                    continue
                stem = f'{split}_{saved:05d}'
                if not cv2.imwrite(str(args.output / 'images' / split / (stem + '.jpg')), cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError('Image write failed')
                (args.output / 'labels' / split / (stem + '.txt')).write_text(
                    '' if label is None else '0 ' + ' '.join(f'{value:.8f}' for value in label) + '\n')
                saved += 1
                print(f'{split}: {saved}/{wanted}', flush=True)
        (args.output / 'data.yaml').write_text(yaml.safe_dump({
            'train': 'images/train', 'val': 'images/val', 'test': 'images/test', 'names': {0: 'box'}}, sort_keys=False))
        print('Dataset ready:', args.output)
    finally:
        request('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')
        print('Collection camera removed', flush=True)


if __name__ == '__main__':
    main()
