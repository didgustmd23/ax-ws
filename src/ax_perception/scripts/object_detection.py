#!/usr/bin/env python3
"""C1 영상 기반 YOLO 검출. Gazebo 정답·물체 pose를 읽지 않는다.

Detection2DArray는 픽셀 관측이다. 물리 물체 ID·3D 위치·입고 완료를 선언하지 않는다.
"""
import argparse
from copy import deepcopy
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_processing import checked_image, stamp


def bgr_image(message):
    checked_image(message)
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}[message.encoding]
    data = np.ndarray((message.height, message.width, channels), dtype='u1', buffer=message.data,
                      strides=(message.step, channels, 1)).copy()
    code = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
            'bgra8': cv2.COLOR_BGRA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
    if message.encoding == 'mono8':
        data = data[:, :, 0]
    return cv2.cvtColor(data, code[message.encoding]) if message.encoding in code else data


def detection_message(header, boxes, scores, classes, names):
    result = Detection2DArray(header=deepcopy(header))
    for (x, y, width, height), score, category in zip(boxes, scores, classes):
        item = Detection2D(header=deepcopy(header))
        item.bbox.center.position.x, item.bbox.center.position.y = float(x), float(y)
        item.bbox.size_x, item.bbox.size_y = float(width), float(height)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(names[int(category)])
        hypothesis.hypothesis.score = float(score)
        item.results.append(hypothesis)
        # id는 비워 둔다. 클래스가 같다는 사실만으로 같은 물체라고 추정하지 않는다.
        result.detections.append(item)
    return result


class ObjectDetection(Node):
    def __init__(self, weights):
        super().__init__('object_detection')
        from ultralytics import YOLO, settings
        import torch
        settings.update({key: False for key in ('sync', 'clearml', 'comet', 'dvc', 'mlflow', 'raytune', 'tensorboard', 'wandb')})
        torch.set_num_threads(2)
        self.model = YOLO(str(weights.resolve()))
        robot = self.get_namespace().rstrip('/').split('/')[-1]
        defaults = {'input_topic': 'sensors/front_rgb/processed/image_raw',
                    'camera_frame': robot + '/front_rgb_optical_frame',
                    'confidence': .25, 'max_age_s': .5, 'max_rate_hz': 2., 'imgsz': 320}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        if not (0 < self.cfg['confidence'] <= 1 and self.cfg['max_age_s'] > 0 and self.cfg['max_rate_hz'] > 0 and self.cfg['imgsz'] >= 32):
            raise ValueError('Invalid detector parameters')
        # 수신 시작 전에 predictor 생성·모델 복사·워밍업을 끝낸다.
        # 첫 영상 처리 중 초기화가 길어져 결과가 만료되는 일을 피한다.
        self.model.predict(source=np.zeros((self.cfg['imgsz'], self.cfg['imgsz'], 3), dtype=np.uint8),
                           imgsz=self.cfg['imgsz'], conf=self.cfg['confidence'], device='cpu', verbose=False)
        self.lock, self.stop_event = threading.Lock(), threading.Event()
        self.latest, self.last_stamp, self.clock_previous = None, -1., 0.
        self.received, self.inferred, self.reason, self.latency_ms = 0., 0., 'No input', 0.
        self.output = self.create_publisher(Detection2DArray, 'perception/detections', SENSOR)
        self.health = self.create_publisher(DiagnosticArray, 'perception/detection_health', 10)
        self.create_subscription(Image, self.cfg['input_topic'], self.receive, SENSOR)
        self.create_timer(.5, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.worker = threading.Thread(target=self.inference, daemon=True)
        self.worker.start()

    def receive(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        measured = stamp(message)
        with self.lock:
            if now < self.clock_previous:
                self.last_stamp, self.latest = -1., None
            self.clock_previous = now
            self.received = time.monotonic()
            if now <= 0 or measured <= 0 or measured <= self.last_stamp or not (-.05 <= now - measured <= self.cfg['max_age_s']):
                self.reason = 'Rejected image timestamp'
                return
            if message.header.frame_id != self.cfg['camera_frame']:
                self.reason = 'Rejected image frame'
                return
            try:
                checked_image(message)
            except ValueError as error:
                self.reason = str(error)
                return
            self.last_stamp, self.latest = measured, message

    def inference(self):
        import torch
        # 백그라운드 추론 스레드에도 CPU 스레드 수를 명시한다.
        torch.set_num_threads(2)
        while not self.stop_event.is_set():
            start = time.monotonic()
            with self.lock:
                message, self.latest = self.latest, None
            if message is not None:
                try:
                    results = self.model.predict(source=bgr_image(message), imgsz=self.cfg['imgsz'],
                        conf=self.cfg['confidence'], device='cpu', verbose=False)[0]
                    age = self.get_clock().now().nanoseconds * 1e-9 - stamp(message)
                    if -.05 <= age <= self.cfg['max_age_s'] and not self.stop_event.is_set():
                        boxes = results.boxes
                        self.output.publish(detection_message(message.header, boxes.xywh.cpu().numpy(),
                            boxes.conf.cpu().numpy(), boxes.cls.cpu().numpy(), results.names))
                        with self.lock:
                            self.inferred, self.reason = time.monotonic(), 'Inference ready'
                            self.latency_ms = (time.monotonic() - start) * 1000
                    else:
                        with self.lock:
                            self.reason = 'Inference result expired'
                except Exception as error:
                    with self.lock:
                        self.reason = 'Inference error: ' + str(error)
            self.stop_event.wait(max(.01, 1 / self.cfg['max_rate_hz'] - (time.monotonic() - start)))

    def report(self):
        with self.lock:
            now, reason = time.monotonic(), self.reason
            stale = now - self.received > 2 or now - self.inferred > 2
            status = DiagnosticStatus(name=self.get_namespace() + '/C1', hardware_id=self.get_namespace(),
                level=DiagnosticStatus.STALE if stale else DiagnosticStatus.OK if reason == 'Inference ready' else DiagnosticStatus.ERROR,
                message=reason, values=[KeyValue(key='inference_wall_ms', value=str(self.latency_ms))])
        message = DiagnosticArray(status=[status]);message.header.stamp = self.get_clock().now().to_msg()
        self.health.publish(message)

    def close(self):
        self.stop_event.set()
        self.worker.join(timeout=10)
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', type=Path, required=True)
    args = parser.parse_args(remove_ros_args()[1:])
    if not args.weights.is_file():
        parser.error('Trained weights not found; run tools/train_box_detector.py first')
    os.environ['YOLO_CONFIG_DIR'] = str(args.weights.resolve().parent / '.config')
    rclpy.init()
    node = ObjectDetection(args.weights)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
