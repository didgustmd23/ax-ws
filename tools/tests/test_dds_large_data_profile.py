"""큰 영상·점군 발행 노드용 Fast DDS 프로필(fastdds_large_data.xml)의 내용과 launch 적용 범위를 검사한다.

- XML: 파싱 가능, SHM 세그먼트 32 MB·UDP 버퍼 4 MB·기본 전송 끔·기본 프로필.
- robot_test.launch.py: parameter_bridge에만 FASTRTPS_DEFAULT_PROFILES_FILE을 넣고 다른 노드·gz sim에는 넣지 않는다.
- sensors.launch.py: sensor_processing 노드마다 넣는다.
- 사용자가 이미 FASTRTPS_DEFAULT_PROFILES_FILE을 지정했으면 덮지 않는다. 프로필 파일이 없으면 넣지 않는다.
Gazebo·ROS 통신 없이 launch 구성만 평가한다. 설치본이 아닌 src 파일을 읽는다.
"""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / 'src/ax_description'
PROFILE = DESCRIPTION / 'config/fastdds_large_data.xml'
ENV = 'FASTRTPS_DEFAULT_PROFILES_FILE'
NS = {'p': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}


# launch 파일을 모듈로 읽는다. ROS launch 패키지가 없으면 None.
def load(path, name):
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except ImportError:
        return None


ROBOT_TEST = load(DESCRIPTION / 'launch/robot_test.launch.py', 'dds_robot_test_launch')
SENSORS = load(ROOT / 'src/ax_perception/launch/sensors.launch.py', 'dds_sensors_launch')


# launch 액션의 추가 환경을 {이름: 값} 문자열 사전으로 평가한다. 없으면 빈 사전.
def evaluated_env(action, context):
    from launch.utilities import perform_substitutions

    pairs = action.additional_env or []
    return {perform_substitutions(context, key): perform_substitutions(context, value) for key, value in pairs}


# 공유 경로 조회에서 ax_description만 src 폴더로 돌린다(설치본에는 새 XML이 아직 없을 수 있음).
def share_with_src_description(real):
    def lookup(name):
        return str(DESCRIPTION) if name == 'ax_description' else real(name)
    return lookup


class ProfileFileTest(unittest.TestCase):
    # 프로필 XML이 파싱되고 시험에서 확인한 값과 같은지 본다.
    def test_profile_values(self):
        root = ET.parse(PROFILE).getroot()
        descriptors = {d.findtext('p:transport_id', namespaces=NS): d
                       for d in root.findall('p:transport_descriptors/p:transport_descriptor', NS)}
        types = {name: d.findtext('p:type', namespaces=NS) for name, d in descriptors.items()}
        self.assertEqual(sorted(types.values()), ['SHM', 'UDPv4'])

        shm = next(d for d in descriptors.values() if d.findtext('p:type', namespaces=NS) == 'SHM')
        udp = next(d for d in descriptors.values() if d.findtext('p:type', namespaces=NS) == 'UDPv4')
        self.assertEqual(int(shm.findtext('p:segment_size', namespaces=NS)), 32 * 1024 * 1024)
        self.assertEqual(int(udp.findtext('p:sendBufferSize', namespaces=NS)), 4194304)
        self.assertEqual(int(udp.findtext('p:receiveBufferSize', namespaces=NS)), 4194304)

        participant = root.find('p:participant', NS)
        self.assertEqual(participant.get('is_default_profile'), 'true')
        self.assertEqual(participant.findtext('p:rtps/p:useBuiltinTransports', namespaces=NS), 'false')
        used = [t.text for t in participant.findall('p:rtps/p:userTransports/p:transport_id', NS)]
        self.assertEqual(sorted(used), sorted(descriptors))


@unittest.skipIf(ROBOT_TEST is None or SENSORS is None, 'ROS launch 패키지 필요(source install/setup.bash)')
class LaunchScopeTest(unittest.TestCase):
    # robot_test.launch.py를 평가해 노드별 추가 환경을 모은다.
    def robot_test_actions(self, environment):
        from launch import LaunchContext
        from launch_ros.actions import Node

        context = LaunchContext()
        context.launch_configurations.update({
            'world': 'planned/ax_warehouse.world', 'test_fixture_world': '', 'robots': '2', 'gui': 'false',
            'controller_config_dir': '', 'tool_attach': 'false', 'sensors': 'true', 'tool': 'epick',
            'payload_mass': '0.0', 'robot_ids': ''})
        lookup = share_with_src_description(ROBOT_TEST.get_package_share_directory)

        with patch.dict(os.environ, environment, clear=False), \
                patch.object(ROBOT_TEST, 'get_package_share_directory', side_effect=lookup):
            if ENV not in environment:
                os.environ.pop(ENV, None)
            actions = ROBOT_TEST.setup(context)

        nodes = [a for a in actions if isinstance(a, Node)]
        return context, nodes, actions

    # parameter_bridge에만 프로필이 들어가고 나머지 노드·gz sim include에는 없다.
    def test_bridge_only(self):
        from launch.actions import IncludeLaunchDescription

        context, nodes, actions = self.robot_test_actions({})
        envs = {node.node_executable: evaluated_env(node, context) for node in nodes}
        self.assertEqual(envs['parameter_bridge'], {ENV: str(PROFILE)})

        others = {name: env for name, env in envs.items() if name != 'parameter_bridge'}
        self.assertTrue(others)
        self.assertTrue(all(ENV not in env for env in others.values()), others)
        # gz sim은 include로 띄우며 이 프로필을 받지 않는다(큰 DDS 메시지를 내지 않음).
        self.assertTrue(any(isinstance(a, IncludeLaunchDescription) for a in actions))

    # 사용자가 이미 프로필을 지정했으면 덮지 않는다.
    def test_existing_user_profile_kept(self):
        context, nodes, _ = self.robot_test_actions({ENV: '/tmp/user_profile.xml'})
        bridge = next(n for n in nodes if n.node_executable == 'parameter_bridge')
        self.assertEqual(evaluated_env(bridge, context), {})

    # 프로필 파일이 없으면 넣지 않는다.
    def test_missing_profile_file(self):
        with tempfile.TemporaryDirectory() as empty, patch.dict(os.environ, {}, clear=False):
            os.environ.pop(ENV, None)
            self.assertEqual(ROBOT_TEST.large_data_env(empty), {})
            self.assertEqual(ROBOT_TEST.large_data_env(DESCRIPTION), {ENV: str(PROFILE)})

    # sensors.launch.py는 로봇별 sensor_processing마다 프로필을 넣고, 사용자 지정값은 존중한다.
    def test_sensor_processing(self):
        from launch import LaunchContext

        context = LaunchContext()
        context.launch_configurations.update({'robot_ids': 'robot_1,robot_2', 'tool': 'epick'})
        lookup = share_with_src_description(SENSORS.get_package_share_directory)

        with patch.object(SENSORS, 'get_package_share_directory', side_effect=lookup):
            with patch.dict(os.environ, {}, clear=False):
                os.environ.pop(ENV, None)
                nodes = SENSORS.setup(context)
            self.assertEqual(len(nodes), 2)
            for node in nodes:
                self.assertEqual(node.node_executable, 'sensor_processing.py')
                self.assertEqual(evaluated_env(node, context), {ENV: str(PROFILE)})

            with patch.dict(os.environ, {ENV: '/tmp/user_profile.xml'}, clear=False):
                kept = SENSORS.setup(context)
            self.assertTrue(all(evaluated_env(node, context) == {} for node in kept))


if __name__ == '__main__':
    unittest.main()
