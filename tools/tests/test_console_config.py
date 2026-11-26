"""관제 화면 설정 검사(console_config) 단위 시험. ROS 실행 없음.

확인: 패키지 기본 설정 파일 통과, 로봇 목록 거부 조건(출처·형식·중복·역할·spare),
Topic profile(운영에 /ax/dev 금지, 개발은 /ax/dev만), 바인딩·포트, 판정 수치, 기준 지도 읽기.
"""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / 'src' / 'ax_operations_console'
sys.path.insert(0, str(PKG))

from ax_operations_console import console_config as cfg  # noqa: E402

ROBOTS_FILE = PKG / 'config' / 'console_robots_dev.yaml'
TOPICS_FILE = PKG / 'config' / 'console_topics.yaml'
MAP_YAML = ROOT / 'src' / 'ax_localization' / 'maps' / 'ax_warehouse' / 'map.yaml'


class ConfigTest(unittest.TestCase):
    """설정 파일 검사."""

    # 임시 폴더
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    # 임시 YAML 쓰기
    def write(self, text, name='f.yaml'):
        path = self.dir / name
        path.write_text(text, encoding='utf-8')
        return path

    # 패키지 기본 로봇 목록: 출처 dev_config, 4대, 예비 2대
    def test_default_robots(self):
        source, robots = cfg.load_robots(ROBOTS_FILE)
        self.assertEqual(source, 'dev_config')
        self.assertEqual([r['robot_id'] for r in robots],
                         ['robot_1', 'robot_2', 'robot_3', 'robot_4'])
        self.assertEqual([r['spare'] for r in robots], [False, False, True, True])

    # 로봇 목록 거부 조건
    def test_robot_rejections(self):
        head = 'source: dev_config\nrobots: '
        cases = {
            'source: fleet_snapshot\nrobots: [{robot_id: robot_1, role: MANIPULATOR}]': '출처',
            head + '[]': 'robots',
            head + '[{robot_id: r1, role: MANIPULATOR}]': 'robot_id',
            head + '[{robot_id: robot_1, role: MANIPULATOR}, '
                   '{robot_id: robot_1, role: TRANSPORTER}]': '중복',
            head + '[{robot_id: robot_1, role: PICKER}]': '역할',
            head + '[{robot_id: robot_1, role: MANIPULATOR, spare: "yes"}]': 'spare',
            head + '[{robot_id: robot_1, role: MANIPULATOR, extra: 1}]': '형식',
            '- a\n- b': '사전',
        }

        for text, word in cases.items():
            with self.subTest(text=text):
                with self.assertRaisesRegex(cfg.ConfigError, word):
                    cfg.load_robots(self.write(text))

    # 기본 Topic profile
    def test_default_topics(self):
        operational = cfg.load_topics(TOPICS_FILE, 'operational')
        dev = cfg.load_topics(TOPICS_FILE, 'dev')
        self.assertEqual(operational['tasks'], '/ax/fleet/tasks')
        self.assertEqual(dev['tasks'], '/ax/dev/d2_mock/fleet_tasks')
        self.assertEqual(cfg.robot_topic('robot_2', operational['robot']['actuation']),
                         '/ax/robot_2/safety/actuation_state')

    # Topic profile 거부 조건
    def test_topic_rejections(self):
        robot = ('robot_topics: {actuation: a, safety: b, health: c, executor: d, pose: e, '
                 'battery: f}\n')
        fleet = 'profiles: {operational: {tasks: /ax/fleet/tasks}}\n'
        cases = [
            ('profiles: {operational: {tasks: /ax/dev/x}}\n' + robot, 'operational', '/ax/dev/'),
            ('profiles: {dev: {tasks: /ax/fleet/tasks}}\n' + robot, 'dev', '/ax/dev/'),
            (fleet + robot, 'missing', 'profile'),
            ('profiles: {operational: {tasks: "ax/fleet"}}\n' + robot, 'operational', '형식'),
            (fleet + 'robot_topics: {actuation: a}\n', 'operational', 'robot_topics'),
            (fleet + robot.replace('a,', '/abs,'), 'operational', '상대'),
        ]

        for text, profile, word in cases:
            with self.subTest(text=text, profile=profile):
                with self.assertRaisesRegex(cfg.ConfigError, word):
                    cfg.load_topics(self.write(text), profile)

    # 바인딩: 루프백은 외부 아님, 다른 주소는 외부, 금지 포트·범위 밖 거부
    def test_bind(self):
        self.assertFalse(cfg.check_bind('127.0.0.1', 18780))
        self.assertTrue(cfg.check_bind('0.0.0.0', 18780))

        for address, port in (('localhost', 18780), ('127.0.0.1', 8765), ('127.0.0.1', 9090),
                              ('127.0.0.1', 80), ('127.0.0.1', '18780')):
            with self.subTest(address=address, port=port):
                with self.assertRaises(cfg.ConfigError):
                    cfg.check_bind(address, port)

    # 판정 수치 범위
    def test_timing(self):
        cfg.check_timing(0.1, 0.5)

        for future, grace in ((-0.1, 0.5), (1.5, 0.5), (0.1, -1.0), (0.1, 3.0)):
            with self.subTest(future=future, grace=grace):
                with self.assertRaises(cfg.ConfigError):
                    cfg.check_timing(future, grace)

    # 기준 지도 읽기(ax_warehouse, 읽기 전용)
    def test_load_map(self):
        info, data = cfg.load_map(MAP_YAML)
        self.assertEqual((info['width'], info['height']), (883, 324))
        self.assertAlmostEqual(info['resolution'], 0.05)
        self.assertEqual(info['name'], 'ax_warehouse')
        self.assertTrue(data.startswith(b'P5'))

    # 잘못된 지도: PGM 아님·머리 불완전
    def test_bad_map(self):
        self.write('P2\n1 1\n255\n0', 'm.pgm')
        with self.assertRaises(cfg.ConfigError):
            cfg.load_map(self.write('image: m.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'))

        (self.dir / 'n.pgm').write_bytes(b'P5\n2 2\n255\n\x00')
        with self.assertRaises(cfg.ConfigError):
            text = 'image: n.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
            cfg.load_map(self.write(text, 'n.yaml'))


if __name__ == '__main__':
    unittest.main()
