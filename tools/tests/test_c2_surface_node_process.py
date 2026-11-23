"""설치본 C2 입력 노드를 ros2 run으로 띄워 실제 DDS 메시지·시작 거부·종료 정리를 검사한다.

[검사 범위] AX_C2_NODE_PREFIX(시험용 격리 install 폴더)를 source해 실행한 설치본만 검사한다. 공유 install을 쓰지 않는다.
/clock·RGB-D 부품·Detection2DArray는 이 시험이 합성해 발행한다. 실제 C1 추론·B1·Gazebo 검증이 아니다.
설정값은 시험값이다(운영 승인값 아님).

실행 예(저장소 루트)
  source /opt/ros/jazzy/setup.bash
  AX_C2_NODE_PREFIX=<격리 install 절대 경로> python3 -m pytest -q tools/tests/test_c2_surface_node_process.py
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/tests'))
sys.path.insert(0, str(ROOT / 'src/ax_perception/scripts'))

import ros_test_domain  # noqa: E402

PREFIX = os.environ.get('AX_C2_NODE_PREFIX', '')
NS = 1_000_000_000
NAMESPACE = '/c2test/robot_1'
# 대용량 영상 전송용 기존 Fast DDS 프로필(B1 launch와 같은 파일, 소스 경로에서 읽기만 함)
PROFILE = ROOT / 'src/ax_description/config/fastdds_large_data.xml'


def setUpModule():
    global DOMAIN, DOMAIN_LOCK, rclpy, support, Clock, Time

    if not PREFIX or not (Path(PREFIX) / 'setup.bash').is_file():
        raise unittest.SkipTest('AX_C2_NODE_PREFIX(격리 install 폴더)가 지정되지 않음: 설치본 실행 검사 생략')

    DOMAIN, DOMAIN_LOCK = ros_test_domain.reserve_domain(range(96, 100))
    os.environ.update(ROS_DOMAIN_ID=str(DOMAIN), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(PROFILE))

    import rclpy
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock
    import test_c2_surface_node as support

    support.load_ros()
    rclpy.init()


def tearDownModule():
    rclpy.shutdown()
    ros_test_domain.release(DOMAIN_LOCK)


# 시험 설정 파일(YAML의 부분집합인 JSON). drop에 든 이름은 빼서 시작 거부를 시험한다
def write_parameters(path, record, extra=None, drop=()):
    values = dict(support.BASE_PARAMETERS, record_path=record, use_sim_time=True)
    values.update({'estimation.' + key: value for key, value in support.TEST_SETTINGS.__dict__.items()})
    values.update(extra or {})
    values = {name: value for name, value in values.items() if name not in drop}
    Path(path).write_text(json.dumps({'/**': {'ros__parameters': values}}, indent=1, ensure_ascii=False))

    return path


# 설치본 노드 실행(시험 install만 source, 저장소 소스 경로를 import 경로에 넣지 않음)
def launch(parameters):
    command = ('source /opt/ros/jazzy/setup.bash && source "$AX_C2_NODE_PREFIX/setup.bash" && '
               'echo "prefix=$(ros2 pkg prefix ax_perception)" && '
               f'exec ros2 run ax_perception c2_surface_node.py --ros-args -r __ns:={NAMESPACE} '
               f'--params-file "{parameters}"')
    environment = dict(os.environ, AX_C2_OWNER='test_c2_surface_node_process')
    environment.pop('PYTHONPATH', None)

    return subprocess.Popen(['bash', '-c', command], cwd=tempfile.gettempdir(), env=environment,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


# 기록 줄 읽기
def rows(path):
    if not Path(path).is_file():
        return []

    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


class InstalledNode(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.record = os.path.join(self.folder.name, 'record.jsonl')
        self.parameters = os.path.join(self.folder.name, 'parameters.yaml')

    # 프로세스 그룹을 끝까지 정리한다(시험 실패 시에도)
    def stop(self, process, sig=signal.SIGINT, wait_s=10.0):
        if process.poll() is None:
            os.killpg(process.pid, sig)

        try:
            return process.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()

            return None

    def test_invalid_settings_exit_with_reason(self):
        process = launch(write_parameters(self.parameters, self.record, drop=('detection_topic',)))
        output, _ = process.communicate(timeout=30)
        self.assertEqual(process.returncode, 2, output)
        self.assertIn('CONFIG_INVALID: detection_topic', output)
        self.assertFalse(Path(self.record).exists())

    # 시험 발행자(같은 namespace)를 만들고 노드 시작 기록·구독 연결·노드 쪽 실제 수신을 확인한다.
    # 발행자 쪽 구독 수(get_subscription_count)는 노드 쪽 매칭 완료를 뜻하지 않는다. 매칭 직후 첫 영상이
    # 노드에 닿지 않는 경우가 실측돼(best effort라 재전송 없음), 시계 시작 전 탐침으로 네 입력의 수신을 확인한다.
    def connect(self, process):
        from rclpy.qos import qos_profile_sensor_data as SENSOR

        self.addCleanup(self.stop, process, signal.SIGKILL, 5.0)
        node = rclpy.create_node('c2_process_test', namespace=NAMESPACE)
        self.addCleanup(node.destroy_node)
        clock = node.create_publisher(Clock, '/clock', 10)
        publishers = {part: node.create_publisher(kind, f'sensors/front_rgbd/processed/{part}', SENSOR)
                      for part, kind in (('image', support.Image), ('depth_image', support.Image),
                                         ('camera_info', support.CameraInfo))}
        publishers['detections'] = node.create_publisher(support.Detection2DArray, support.TOPIC, SENSOR)
        end = time.monotonic() + 30

        while time.monotonic() < end and not (rows(self.record) and all(
                publisher.get_subscription_count() >= 1 for publisher in publishers.values())):
            rclpy.spin_once(node, timeout_sec=0.05)

        unmatched = [topic for topic, publisher in publishers.items() if publisher.get_subscription_count() < 1]
        self.assertTrue(rows(self.record), '30 s 안에 노드 start 기록 없음')
        self.assertEqual(unmatched, [], '30 s 안에 노드 구독과 매칭되지 않은 시험 발행자')

        self.wait_node_receives(node, publishers)
        # 탐침 기록(시계 시작 전 거부)은 시험 판정에서 뺀다
        self.ready_row = len(rows(self.record))

        # sim 시각(정수 ns)을 발행하고 메시지를 보낸다. confirm_clock이면 노드 시계가 그 시각에 닿은 것을
        # 기록에서 확인한 뒤 보낸다(/clock과 영상은 다른 토픽이라 도착 순서가 보장되지 않음)
        def send(stamp_ns, *messages, confirm_clock=False):
            clock.publish(Clock(clock=Time(sec=stamp_ns // NS, nanosec=stamp_ns % NS)))

            if confirm_clock:
                self.wait_node_clock(node, clock, stamp_ns)

            for topic, message in messages:
                publishers[topic].publish(message)

            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0.02)

        return send

    # 노드가 네 입력을 실제로 받았는지 state 기록의 received_* 카운터로 확인한다(상한 10 s, 넘으면 실패).
    # 탐침은 시계 시작 전(sim 0)에 보내므로 노드가 CLOCK_NOT_STARTED로 버리고 조립·연결 상태는 바뀌지 않는다.
    # 시험 판정용 메시지를 다시 보내는 것이 아니라 준비 확인용 탐침만 일정 간격으로 보낸다.
    def wait_node_receives(self, node, publishers, limit_s=10.0):
        probe_stamp = 1 * NS
        probes = {'image': support.color_message(probe_stamp), 'depth_image': support.depth_message(probe_stamp),
                  'camera_info': support.info_message(probe_stamp),
                  'detections': support.detections(probe_stamp, [])}
        keys = {'image': 'received_image', 'depth_image': 'received_depth_image',
                'camera_info': 'received_camera_info', 'detections': 'received_detection_arrays'}
        end = time.monotonic() + limit_s
        missing = list(keys)

        while time.monotonic() < end:
            for topic in missing:
                publishers[topic].publish(probes[topic])

            pause = time.monotonic() + 0.2

            while time.monotonic() < pause:
                rclpy.spin_once(node, timeout_sec=0.05)

            states = [row for row in rows(self.record) if row['event'] == 'state']
            counts = states[-1]['counts'] if states else {}
            missing = [topic for topic, key in keys.items() if counts.get(key, 0) < 1]

            if not missing:
                return

        self.fail(f'{limit_s} s 안에 노드가 받지 못한 입력: {missing}')

    # 노드 시계가 stamp_ns 이상인 기록이 생길 때까지 /clock을 발행한다(상한 5 s, 넘으면 실패).
    # 노드는 state 기록(state_period_s)마다 ros_now_ns를 남긴다
    def wait_node_clock(self, node, clock, stamp_ns, limit_s=5.0):
        message = Clock(clock=Time(sec=stamp_ns // NS, nanosec=stamp_ns % NS))
        end = time.monotonic() + limit_s

        while time.monotonic() < end:
            if any(row.get('ros_now_ns', 0) >= stamp_ns for row in rows(self.record)[self.ready_row:]):
                return

            clock.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)

        self.fail(f'{limit_s} s 안에 노드 시계가 {stamp_ns} ns에 닿지 않음')

    # 한 stamp의 세 부품(주어진 순서)
    def frame(self, stamp, order=('image', 'depth_image', 'camera_info')):
        makers = {'image': support.color_message, 'depth_image': support.depth_message,
                  'camera_info': support.info_message}

        return [(part, makers[part](stamp)) for part in order]

    def test_installed_node_links_real_messages_and_exits_cleanly(self):
        process = launch(write_parameters(self.parameters, self.record))
        send = self.connect(process)
        start = rows(self.record)
        self.assertTrue(start, 'start 기록 없음')
        modules = start[0]['modules']
        self.assertTrue(all(path.startswith(os.path.realpath(PREFIX)) for path in modules.values()), modules)
        self.assertEqual(start[0]['topics']['depth_image'], NAMESPACE + '/sensors/front_rgbd/processed/depth_image')
        self.assertEqual(start[0]['detection_topic'], NAMESPACE + '/' + support.TOPIC)

        boxes = [(box, 'box', 0.9) for box in support.inner_boxes()]
        t0, t1, t2 = 10 * NS, 10 * NS + 100_000_000, 10 * NS + 200_000_000

        # 검출 선도착 + 부품 역순, 다중 bbox, 빈 배열, 다른 카메라 검출(시계 0 거부는 프로세스 내부 시험이 검사).
        # 각 stamp의 메시지는 노드 시계가 그 시각에 닿은 것을 확인한 뒤 보낸다.
        send(t0, confirm_clock=True)
        send(t0, ('detections', support.detections(t0, boxes)))
        send(t0, *self.frame(t0, ('camera_info', 'depth_image', 'image')))
        send(t1, *self.frame(t1), ('detections', support.detections(t1, [])), confirm_clock=True)
        send(t2, ('detections', support.detections(t2, boxes, frame=support.OTHER_FRAME)), confirm_clock=True)

        end = time.monotonic() + 15

        while time.monotonic() < end and len([row for row in rows(self.record)[self.ready_row:]
                                              if row['event'] != 'state']) < 4:
            send(t2)

        self.assertEqual(self.stop(process), 0)
        output = process.stdout.read()
        self.assertIn('prefix=' + PREFIX, output)
        events = [row for row in rows(self.record)[self.ready_row:] if row['event'] != 'state']
        results = [row for row in events if row['event'] == 'detection_result']
        self.assertEqual([row['link_status'] for row in results], ['LINKED', 'LINKED'], events)
        self.assertEqual([row['stamp_ns'] for row in results], [t0, t0])
        self.assertEqual([row['stamp_ns'] for row in events if row['event'] == 'detection_array_empty'], [t1])
        self.assertIn(['DETECTION_FRAME_MISMATCH'],
                      [row['reasons'] for row in events if row['event'] == 'detection_array_rejected'])
        self.assertEqual(events[-1]['event'], 'shutdown')
        self.assertEqual(events[-1]['linker']['buffered'], 0)

        # 종료 뒤 같은 소유 표식 프로세스가 남지 않는다.
        leftovers = subprocess.run(['pgrep', '-f', f'__ns:={NAMESPACE}'], capture_output=True, text=True).stdout
        self.assertEqual(leftovers.strip(), '')

    def test_record_limit_exits_with_code_4(self):
        # 기록 바이트 상한(시험값 8 KiB): 도달하면 표식 1행을 남기고 종료 코드 4로 스스로 멈춘다(정상 완료 아님).
        process = launch(write_parameters(self.parameters, self.record, extra={'max_record_bytes': 8192}))
        send = self.connect(process)
        boxes = [(box, 'box', 0.9) for box in support.inner_boxes()]
        stamp = 10 * NS
        end = time.monotonic() + 20

        while time.monotonic() < end and process.poll() is None:
            send(stamp, *self.frame(stamp), ('detections', support.detections(stamp, boxes)))
            stamp += 50_000_000

        self.assertEqual(self.stop(process, wait_s=10.0), 4)
        output = process.stdout.read()
        self.assertIn('RECORD_LIMIT_REACHED', output)
        events = [row['event'] for row in rows(self.record)]
        self.assertEqual(events[-1], 'record_limit_reached')
        self.assertNotIn('shutdown', events)
        self.assertLessEqual(Path(self.record).stat().st_size, 8192)
        leftovers = subprocess.run(['pgrep', '-f', f'__ns:={NAMESPACE}'], capture_output=True, text=True).stdout
        self.assertEqual(leftovers.strip(), '')


if __name__ == '__main__':
    unittest.main()
