"""Conservative URDF collision screening for model tests, not an A4 safety gate."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import xacro
from ament_index_python.packages import get_package_share_directory

ARM_JOINTS = ['column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint']

def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return result

def overlap(a, b):
    center_a, axes_a, half_a = a
    center_b, axes_b, half_b = b
    if np.any(np.abs(center_a-center_b) > np.abs(axes_a) @ half_a + np.abs(axes_b) @ half_b - 1e-6):
        return False
    axes = [*axes_a.T, *axes_b.T]
    axes.extend(np.cross(first, second) for first in axes_a.T for second in axes_b.T)
    delta = center_b - center_a
    for axis in axes:
        length = np.linalg.norm(axis)
        if length < 1e-8:
            continue
        axis = axis / length
        reach = np.abs(axes_a.T @ axis) @ half_a + np.abs(axes_b.T @ axis) @ half_b
        if abs(delta @ axis) >= reach - 1e-6:
            return False
    return True

class ArmGeometry:
    def __init__(self, payload=0.0):
        share = Path(get_package_share_directory('ax_description'))
        self.robot = ET.fromstring(xacro.process_file(str(share / 'urdf/ax.urdf.xacro'), mappings={
            'gripper_model': 'epick', 'sensors_enabled': 'false', 'test_payload_mass': str(payload)}).toxml())
        self.joints = self.robot.findall('joint')
        self.links = {link.get('name'): link for link in self.robot.findall('link')}
        self.limits = {joint.get('name'): (float(joint.find('limit').get('lower')), float(joint.find('limit').get('upper')))
                       for joint in self.joints if joint.get('name') in ARM_JOINTS}
        groups = {name: name for name in self.links}
        def group(name):
            while groups[name] != name:
                name = groups[name]
            return name
        for joint in self.joints:
            if joint.get('type') == 'fixed':
                groups[group(joint.find('child').get('link'))] = group(joint.find('parent').get('link'))
        self.groups = {name: group(name) for name in self.links}
        self.adjacent = {frozenset((self.groups[joint.find('parent').get('link')],
                                   self.groups[joint.find('child').get('link')])) for joint in self.joints}
        self.shapes = []
        for name, link in self.links.items():
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry.find('box') is not None:
                    half = np.fromstring(geometry.find('box').get('size'), sep=' ') / 2
                elif geometry.find('cylinder') is not None:
                    cylinder = geometry.find('cylinder')
                    half = np.array([float(cylinder.get('radius'))]*2 + [float(cylinder.get('length'))/2])
                elif geometry.find('sphere') is not None:
                    half = np.array([float(geometry.find('sphere').get('radius'))]*3)
                else:
                    raise ValueError('Unscreened collision shape: ' + name)
                self.shapes.append((name, origin(collision.find('origin')), half))

    def frames(self, positions):
        values = dict(zip(ARM_JOINTS, positions))
        frames, pending = {'base_link': np.eye(4)}, list(self.joints)
        while pending:
            before = len(pending)
            for joint in pending[:]:
                parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
                if parent not in frames:
                    continue
                motion = np.eye(4)
                if joint.get('type') == 'revolute':
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    motion[:3, :3] = Rotation.from_rotvec(axis * values.get(joint.get('name'), 0)).as_matrix()
                frames[child] = frames[parent] @ origin(joint.find('origin')) @ motion
                pending.remove(joint)
            if len(pending) == before:
                raise ValueError('Disconnected model')
        return frames

    def collisions(self, positions):
        frames = self.frames(positions)
        transformed = []
        for name, offset, half in self.shapes:
            pose = frames[name] @ offset
            transformed.append((name, (pose[:3, 3], pose[:3, :3], half)))
        pairs = set()
        for index, (first, a) in enumerate(transformed):
            # Wheel floor contact is expected; arm/structure below the floor is not.
            low = a[0][2] - np.abs(a[1][2]) @ a[2] + .38908325
            if 'wheel' not in first and low < -.002:
                pairs.add((first, 'ground'))
            for second, b in transformed[index+1:]:
                if self.groups[first] == self.groups[second]:
                    continue
                if frozenset((self.groups[first], self.groups[second])) in self.adjacent:
                    continue
                if overlap(a, b):
                    pairs.add(tuple(sorted((first, second))))
        return sorted(pairs)

    def screen_path(self, start, end, step=.03):
        for index, joint in enumerate(ARM_JOINTS):
            lower, upper = self.limits[joint]
            if end[index] < lower or end[index] > upper:
                return {'allowed': False, 'reason': 'JOINT_LIMIT', 'joint': joint}
        count = max(2, int(np.max(np.abs(np.array(end)-start))/step)+2)
        for fraction in np.linspace(0, 1, count):
            positions = np.array(start) + fraction * (np.array(end)-start)
            pairs = self.collisions(positions)
            if pairs:
                return {'allowed': False, 'fraction': float(fraction), 'pairs': pairs}
        return {'allowed': True, 'samples': count}
