#!/usr/bin/env python3
"""Check AX mass properties and sample static COM projections on level ground.

This does not validate collision-free poses, grasping, tracking or dynamic tipping.
Requires sourced ROS workspace, numpy, scipy, matplotlib.
"""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]
ARM = ['column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint']
TARGETS = {
    'suction': ('suction_grasp_frame', [0, 0, .15]),
    'parallel_wide': ('wide_grasp_frame', [0, 0, 0]),
    'primitive': ('gripper_base_link', [.07, 0, 0]),
    'robotiq': ('robotiq_85_base_link', [0, 0, .17]),
}


def xyz(element, key, default='0 0 0'):
    return np.array([float(v) for v in (element.get(key, default) if element is not None else default).split()])


def origin(element):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', xyz(element, 'rpy')).as_matrix()
    transform[:3, 3] = xyz(element, 'xyz')
    return transform


def fk(root, poses):
    count = len(poses)
    transforms = {'base_link': np.broadcast_to(np.eye(4), (count, 4, 4))}
    remaining = list(root.findall('joint'))
    while remaining:
        progress = False
        for joint in remaining[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            motion = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
            name = joint.get('name')
            if name in ARM:
                axis = xyz(joint.find('axis'), 'xyz', '1 0 0')
                motion[:, :3, :3] = Rotation.from_rotvec(poses[:, ARM.index(name), None] * axis).as_matrix()
            # Non-arm joints stay at q=0 (open Robotiq; default primitive and wheels).
            transforms[joint.find('child').get('link')] = transforms[parent] @ origin(joint.find('origin')) @ motion
            remaining.remove(joint)
            progress = True
        assert progress, 'URDF has a disconnected graph'
    return transforms


def center_of_mass(root, transforms, payload_mass, payload_frame, payload_point):
    total = 0.
    weighted = np.zeros((len(next(iter(transforms.values()))), 3))
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').get('value'))
        assert mass > 0
        i = inertial.find('inertia')
        matrix = np.array([[float(i.get('ixx')), float(i.get('ixy')), float(i.get('ixz'))],
                           [float(i.get('ixy')), float(i.get('iyy')), float(i.get('iyz'))],
                           [float(i.get('ixz')), float(i.get('iyz')), float(i.get('izz'))]])
        eigen = np.linalg.eigvalsh(matrix)
        assert np.all(eigen > 0) and eigen[-1] <= sum(eigen[:2]) + 1e-9
        point = np.r_[xyz(inertial.find('origin'), 'xyz'), 1]
        weighted += mass * (transforms[link.get('name')] @ point)[:, :3]
        total += mass
    if payload_mass:
        weighted += payload_mass * (transforms[payload_frame] @ np.r_[payload_point, 1])[:, :3]
        total += payload_mass
    return total, weighted / total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=20000)
    parser.add_argument('--before-dir', type=Path)
    parser.add_argument('--output-dir', type=Path,
                        default=ROOT / 'docs/evidence/physical_properties')
    args = parser.parse_args()
    assert args.samples > 0
    share = Path(get_package_share_directory('ax_description'))
    properties = json.loads((share / 'config/physical_properties.json').read_text())
    results = {'scope': 'Static level-ground COM only. Sampled joint poses NOT filtered for self/ground collision or reachability. No dynamic or grasp validation.',
               'seed': 42, 'random_samples': args.samples, 'required_margin_m': .05,
               'payload_assumption': '2 kg rigidly attached at declared point; suction box COM 150 mm beyond cup plane. Tool grasp force not tested.', 'models': {}}
    plots = {}
    with tempfile.TemporaryDirectory() as tmp:
        for model, (payload_frame, point) in TARGETS.items():
            output = Path(tmp) / f'{model}.urdf'
            subprocess.run(['xacro', str(share/'urdf/ax.urdf.xacro'), f'gripper_model:={model}', '-o', str(output)], check=True)
            subprocess.run(['check_urdf', str(output)], check=True, stdout=subprocess.DEVNULL)
            root = ET.parse(output).getroot()
            # Round-trip validation of serialized COM and full tensor, not just diagonal.
            for name, prop in properties['links'].items():
                inertial = root.find(f"link[@name='{name}']/inertial")
                assert np.allclose(xyz(inertial.find('origin'), 'xyz'), prop['center_of_mass_m'], atol=1e-10)
                assert np.isclose(float(inertial.find('mass').get('value')), prop['mass_kg'])
                collisions = root.findall(f"link[@name='{name}']/collision")
                assert len(collisions) == prop['collision_count']
                # Verify every unrounded mesh vertex lies in at least one installed collision box.
                vertices = np.array([[float(x) for x in line.split()[1:4]] for line in (share/properties['mesh_directory']/prop['mesh']).read_text().splitlines() if line.startswith('v ')])*.001 - prop['pivot_m']
                covered = np.zeros(len(vertices), dtype=bool)
                for collision in collisions:
                    transform = origin(collision.find('origin'))
                    size = xyz(collision.find('geometry/box'), 'size')
                    local = (vertices-transform[:3, 3]) @ transform[:3, :3]
                    covered |= np.all(np.abs(local) <= size/2 + 1e-9, axis=1)
                assert np.all(covered), (model, name, int((~covered).sum()))
            if args.before_dir:
                before = ET.parse(args.before_dir / f'{model}.urdf').getroot()
                assert len(root.findall('link')) == len(before.findall('link'))
                for joint in root.findall('joint'):
                    old = before.find(f"joint[@name='{joint.get('name')}']")
                    assert ET.tostring(joint) == ET.tostring(old)
                for link in root.findall('link'):
                    old = before.find(f"link[@name='{link.get('name')}']")
                    assert [ET.canonicalize(ET.tostring(v), strip_text=True) for v in link.findall('visual')] == [ET.canonicalize(ET.tostring(v), strip_text=True) for v in old.findall('visual')]
            limits = np.array([[float(root.find(f"joint[@name='{name}']/limit").get(k)) for k in ('lower','upper')] for name in ARM])
            random = np.random.default_rng(42).uniform(limits[:, 0], limits[:, 1], size=(args.samples, 5))
            grid = np.array(list(itertools.product(*[np.linspace(lo, hi, 5) for lo, hi in limits])))
            poses = np.vstack([np.zeros((1, 5)), grid, random])
            transforms = fk(root, poses)
            wheel_centers = np.array([transforms[name][0, :3, 3] for name in ['front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']])
            xmin, ymin = wheel_centers[:, :2].min(0)
            xmax, ymax = wheel_centers[:, :2].max(0)
            results['support_rectangle_base_link_m'] = [float(xmin), float(xmax), float(ymin), float(ymax)]
            scenarios = {}
            for payload in (0., 2.):
                total, com = center_of_mass(root, transforms, payload, payload_frame, point)
                distances = np.column_stack([com[:, 0]-xmin, xmax-com[:, 0], com[:, 1]-ymin, ymax-com[:, 1]])
                margins = distances.min(1)
                worst = int(np.argmin(margins))
                ground_z = -wheel_centers[0, 2] + .19631
                scenarios[str(payload)] = {'total_mass_kg': total, 'nominal_com_base_link_m': com[0].tolist(),
                    'nominal_com_ground_height_m': float(com[0, 2]+ground_z), 'nominal_margin_m': float(margins[0]),
                    'nominal_restoring_moment_nearest_edge_Nm': float(total*9.81*margins[0]),
                    'sample_count': len(poses), 'minimum_sampled_margin_m': float(margins[worst]),
                    'outside_support_samples': int(np.sum(margins < 0)), 'below_50mm_samples': int(np.sum(margins < .05)),
                    'worst_joint_angles_rad': dict(zip(ARM, poses[worst].tolist()))}
                if payload == 2:
                    plots[model] = (com[:, :2], com[0, :2], xmin, xmax, ymin, ymax)
            results['models'][model] = {'payload_frame': payload_frame, 'payload_center_local_m': point, 'scenarios_payload_kg': scenarios}
            print(model, 'mass/2kg=', round(scenarios['2.0']['total_mass_kg'],3),
                  'nominal margin=', round(scenarios['2.0']['nominal_margin_m'],4),
                  'worst unfiltered margin=', round(scenarios['2.0']['minimum_sampled_margin_m'],4))
    destination = args.output_dir
    destination.mkdir(parents=True, exist_ok=True)
    (destination/'static_stability.json').write_text(json.dumps(results, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, axes = plt.subplots(2, 2, figsize=(10, 10))
    for ax, (model, (points, nominal, xmin, xmax, ymin, ymax)) in zip(axes.flat, plots.items()):
        ax.scatter(points[::5, 0], points[::5, 1], s=1, alpha=.2, label='Unfiltered joint samples')
        ax.add_patch(Rectangle((xmin,ymin), xmax-xmin, ymax-ymin, fill=False, color='black', label='Wheel-center support'))
        ax.add_patch(Rectangle((xmin+.05,ymin+.05), xmax-xmin-.1, ymax-ymin-.1, fill=False, color='orange', linestyle='--', label='50 mm inset'))
        ax.plot(*nominal, 'r*', ms=10, label='q=0, 2 kg payload')
        ax.set(title=model, xlabel='base_link X (m)', ylabel='base_link Y (m)', aspect='equal')
        ax.grid(alpha=.2)
    axes.flat[0].legend(fontsize=7)
    fig.suptitle('AX static COM projections: 2 kg attached payload\nSelf/ground collisions and dynamic stability NOT evaluated')
    fig.tight_layout(rect=(0,0,1,.95))
    fig.savefig(destination/'com_projection.png', dpi=160)
    print('Saved report and plot to', destination)


if __name__ == '__main__':
    main()
