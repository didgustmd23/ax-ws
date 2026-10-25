"""손으로 계산할 수 있는 사례로 tools/check_joint_torques.py의 부호·단위·하중 증가분을 시험한다."""
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
spec = importlib.util.spec_from_file_location('check_joint_torques',
                                              ROOT / 'tools/check_joint_torques.py')
torques = importlib.util.module_from_spec(spec)
spec.loader.exec_module(torques)
G = 9.81


# 시험용 5축 사슬 URDF를 만든다: base -yaw(z)-> column -shoulder(x)-> upper
# 이어서 -elbow(x, +0.5 y)-> fore -wrist(x)-> wrist -rotation(y)-> tool 순서다.
# mass_link만 실제 질량을 갖고 나머지 링크 질량은 무시할 만큼 작다.
def chain(mass_link='upper', mass=2.0, com='0 0.5 0'):
    links = ['base_link', 'column', 'upper', 'fore', 'wrist', 'tool']
    body = ['<robot name="t">']
    for link in links:
        m, c = (mass, com) if link == mass_link else (1e-9, '0 0 0')
        body.append(f'<link name="{link}"><inertial><origin xyz="{c}"/><mass value="{m}"/>'
                    '<inertia ixx="1e-9" iyy="1e-9" izz="1e-9" ixy="0" ixz="0" iyz="0"/>'
                    '</inertial></link>')

    joints = [('column_yaw_joint', 'base_link', 'column', '0 0 0.3', '0 0 1'),
              ('shoulder_joint', 'column', 'upper', '0 0 0.2', '1 0 0'),
              ('elbow_joint', 'upper', 'fore', '0 0.5 0', '1 0 0'),
              ('wrist_joint', 'fore', 'wrist', '0 0.4 0', '1 0 0'),
              ('gripper_rotation_joint', 'wrist', 'tool', '0 0.1 0', '0 1 0')]
    for name, parent, child, xyz, axis in joints:
        body.append(f'<joint name="{name}" type="revolute"><parent link="{parent}"/>'
                    f'<child link="{child}"/><origin xyz="{xyz}"/><axis xyz="{axis}"/>'
                    '<limit lower="-3" upper="3" effort="1" velocity="1"/></joint>')
    return ET.fromstring(''.join(body) + '</robot>')


# 손으로 계산한 값과 비교하는 시험.
class HandComputedTest(unittest.TestCase):
    # 수평으로 떨어진 질량의 유지 토크 부호와 단위(N·m)를 확인한다.
    def test_horizontal_mass_sign_and_unit(self):
        # 어깨축(+X)에서 +Y로 0.5 m 떨어진 2 kg: +X 기준 중력 토크는
        # r × F = (0.5 × -19.62) = -9.81 N·m이므로 유지 토크는 +9.81 N·m다.
        holding, _ = torques.joint_torques(chain(), [[0, 0, 0, 0, 0]])
        self.assertAlmostEqual(holding[0, 1], 2 * G * 0.5, places=6)
        self.assertAlmostEqual(holding[0, 2], 0, places=6)     # 질량이 팔꿈치 아래쪽에 없음

    # 관절 회전에 따라 모멘트 팔과 부호가 바뀌는지 확인한다.
    def test_rotation_changes_lever_and_sign(self):
        holding, _ = torques.joint_torques(
            chain(), [[0, math.pi / 2, 0, 0, 0], [0, math.pi, 0, 0, 0],
                      [0, math.pi / 6, 0, 0, 0]])
        self.assertAlmostEqual(holding[0, 1], 0, places=6)               # 질량이 바로 위
        self.assertAlmostEqual(holding[1, 1], -2 * G * 0.5, places=6)    # 질량이 -Y 쪽
        self.assertAlmostEqual(holding[2, 1], 2 * G * 0.5 * math.cos(math.pi / 6), places=6)

    # 수직축(기둥 yaw)에는 정적 중력 토크가 없는지 확인한다.
    def test_vertical_axis_has_no_static_gravity_torque(self):
        holding, _ = torques.joint_torques(chain(), [[0, 0, 0, 0, 0], [1.0, 0.3, 0, 0, 0]])
        np.testing.assert_allclose(holding[:, 0], 0, atol=1e-9)

    # 기둥 yaw 각도가 다른 관절의 중력 토크를 바꾸지 않는지 확인한다.
    def test_column_yaw_does_not_change_other_joints(self):
        holding, _ = torques.joint_torques(chain('fore', 3.0, '0 0.2 0'),
                                           [[0, -0.4, 0.7, 0, 0], [1.3, -0.4, 0.7, 0, 0]])
        np.testing.assert_allclose(holding[0, 1:], holding[1, 1:], atol=1e-9)

    # 하중이 관절별로 질량 × g × 모멘트 팔만큼 토크를 더하는지 확인한다.
    def test_payload_adds_mass_times_lever(self):
        # tool (0, 0.2, 0)의 1 kg: 어깨축에서 0.5 + 0.4 + 0.1 + 0.2 = 1.2 m, 팔꿈치에서 0.7 m,
        # 손목에서 0.3 m이며 말단 회전축 위에 있다.
        base, _ = torques.joint_torques(chain(), [[0, 0, 0, 0, 0]])
        loaded, _ = torques.joint_torques(chain(), [[0, 0, 0, 0, 0]], (1.0, 'tool', [0, 0.2, 0]))
        np.testing.assert_allclose(loaded[0] - base[0], [0, G * 1.2, G * 0.7, G * 0.3, 0],
                                   atol=1e-9)
        # 수평인 말단 회전축(+Y)에서 옆(+X)으로 0.05 m 옮기면 그 관절에
        # 1 kg × g × 0.05 m가 걸린다.
        offset, _ = torques.joint_torques(chain(), [[0, 0, 0, 0, 0]],
                                          (1.0, 'tool', [0.05, 0.2, 0]))
        self.assertAlmostEqual(abs(offset[0, 4]), G * 0.05, places=9)

    # 점질량의 축 기준 관성이 m r²인지 확인한다.
    def test_axis_inertia_point_mass(self):
        _, inertia = torques.joint_torques(chain(), [[0, 0, 0, 0, 0]])
        self.assertAlmostEqual(inertia[0, 1], 2 * 0.5 ** 2, places=6)
        self.assertAlmostEqual(inertia[0, 0], 2 * 0.5 ** 2, places=6)   # yaw축에서 0.5 m

    # 회전된 inertial 좌표계의 관성 텐서를 링크 좌표로 바르게 바꾸는지 확인한다.
    def test_rotated_inertial_frame(self):
        root = chain('upper', 1.0, '0 0 0')
        inertia = root.find("link[@name='upper']/inertial/inertia")
        inertia.attrib.update(ixx='0.3', iyy='0.1', izz='0.2')
        root.find("link[@name='upper']/inertial/origin").set('rpy', '0 0 1.5707963267948966')
        _, axis_inertia = torques.joint_torques(root, [[0, 0, 0, 0, 0]])
        # Z축으로 90도 돌리면 inertial X축이 링크 Y축에 놓이므로, 어깨(+X) 기준으로는
        # 텐서의 iyy = 0.1이 나온다.
        self.assertAlmostEqual(axis_inertia[0, 1], 0.1, places=6)


# 현재 모델 값을 log/ax_verify/shoulder_gravity.txt의 독립 스크립트 결과와 대조한다.
class CurrentModelTest(unittest.TestCase):

    # 공구별 현재 URDF를 한 번만 생성해 둔다.
    @classmethod
    def setUpClass(cls):
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('ax_description'))
        cls.roots = {}
        with tempfile.TemporaryDirectory() as tmp:
            for tool in torques.TOOLS:
                urdf = Path(tmp) / f'{tool}.urdf'
                subprocess.run(['xacro', str(share / 'urdf/ax.urdf.xacro'),
                                f'gripper_model:={tool}', '-o', str(urdf)], check=True)
                cls.roots[tool] = ET.parse(urdf).getroot()

    # primitive 어깨 토크가 이전 독립 계산값과 0.01 N·m 안에서 같은지 확인한다.
    def test_primitive_matches_independent_values(self):
        holding, _ = torques.joint_torques(
            self.roots['primitive'],
            [[0, 0, 0, 0, 0], [0, -1, 1, 0, 0], [0, -1.5708, 0.95, 0.4, 0]])
        np.testing.assert_allclose(holding[:, 1], [11.55, 38.75, 34.90], atol=0.01)

    # 2 kg 하중의 어깨 토크 증가분이 무게 × 수평 모멘트 팔과 같은지 확인한다.
    def test_payload_increment_equals_weight_times_horizontal_lever(self):
        for tool, (frame, point, *_) in torques.TOOLS.items():
            root = self.roots[tool]
            pose = np.array([[0, -0.6, 0.8, -0.2, 0]])
            base, _ = torques.joint_torques(root, pose)
            loaded, _ = torques.joint_torques(root, pose, (2.0, frame, point))
            transforms = torques.fk(root, pose)
            payload = (transforms[frame][0] @ np.r_[point, 1])[:3]
            shoulder = transforms['upper_arm_link'][0, :3, 3]
            # 기둥 yaw 0에서 어깨축은 base_link +X이므로 모멘트 팔은 Y 방향 거리다.
            self.assertAlmostEqual(loaded[0, 1] - base[0, 1],
                                   2.0 * G * (payload[1] - shoulder[1]), places=6)


if __name__ == '__main__':
    unittest.main()
