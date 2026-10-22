#!/usr/bin/env python3
"""Generate conservative mesh component boxes and explicit virtual mass properties.

Run from any directory with Python, numpy and scipy. Meshes remain unchanged.
Masses describe an assumed virtual design, not measurements of real hardware.
"""
import json
import itertools
import warnings
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
PARTS = ROOT / 'src/ax_description/meshes/parts'
OUT = ROOT / 'src/ax_description/urdf/physical_properties.xacro'
XACRO = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', XACRO)
# Source mesh origin/pivot in assembled CAD coordinates, in meters.
LINKS = {
    'base_link': ('ax_body', [.003, 0, .38277325], 8.),
    'arm_mount_link': ('arm_01_support', [.18, .185, .612], 3.),
    'column_link': ('arm_01_column', [.18, .185, .780], 4.),
    'arm_rotating_base_link': ('arm_02_movie', [.18, .185, 1.5], 1.),
    'upper_arm_link': ('arm_03_upperarm', [.240, .185, 1.6], 4.),
    'forearm_link': ('arm_04_forearm', [.372, .185, 2.160], 3.),
    'wrist_link': ('gripper_wristhousing', [.462, .640, 1.720], 1.),
}


def numbers(v):
    return ' '.join(f'{x:.12g}' for x in v)


def mesh_components(path, pivot):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith('v '):
            vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            face = []
            for token in line.split()[1:]:
                idx = int(token.split('/')[0])
                face.append(idx - 1 if idx > 0 else len(vertices) + idx)
            faces.append(face)
    vertices = np.array(vertices) * .001 - pivot
    # Weld coincident exported surface vertices to recover connected CAD pieces.
    unique, inverse = np.unique(np.round(vertices, 7), axis=0, return_inverse=True)
    edges = np.array([(inverse[f[0]], inverse[i]) for f in faces for i in f[1:]])
    graph = coo_matrix((np.ones(len(edges)), edges.T), shape=(len(unique), len(unique)))
    count, labels = connected_components(graph.tocsr(), directed=False)
    return vertices, [unique[labels == i] for i in range(count)]


def box_for_points(points):
    # Select PCA box only if it is smaller than the axis-aligned box.
    # Every component vertex (and hence each triangle) remains enclosed.
    _, axes = np.linalg.eigh(np.cov(points.T))
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    choices = []
    for rotation in (np.eye(3), axes):
        local = points @ rotation
        lo, hi = local.min(0), local.max(0)
        size = hi - lo + .002  # 1 mm padding on each face, also for planar decals.
        center = rotation @ ((lo + hi) / 2)
        choices.append((float(np.prod(size)), center, size, rotation))
    _, center, size, rotation = min(choices, key=lambda x: x[0])
    assert np.all(np.abs((points - center) @ rotation) <= size / 2 + 1e-10)
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', message='Gimbal lock detected.*')
        rpy = Rotation.from_matrix(rotation).as_euler('xyz').tolist()
    return {'shape': 'box', 'center': center.tolist(), 'size': size.tolist(), 'rpy': rpy}


def remove_contained_boxes(boxes):
    kept = []
    signs = np.array(list(itertools.product((-1, 1), repeat=3)))
    for i, box in enumerate(boxes):
        rotation = Rotation.from_euler('xyz', box['rpy']).as_matrix()
        corners = np.array(box['center']) + (signs * np.array(box['size']) / 2) @ rotation.T
        redundant = False
        for j, other in enumerate(boxes):
            if i == j:
                continue
            own_volume = np.prod(box['size'])
            other_volume = np.prod(other['size'])
            if other_volume < own_volume - 1e-12 or (abs(other_volume-own_volume) < 1e-12 and j > i):
                continue
            other_rotation = Rotation.from_euler('xyz', other['rpy']).as_matrix()
            if np.all(np.abs((corners - other['center']) @ other_rotation) <= np.array(other['size']) / 2 + 1e-10):
                redundant = True
                break
        if not redundant:
            kept.append(box)
    return kept


def mass_box(mass, center, size, rpy=(0, 0, 0), label='housing'):
    return dict(shape='box', mass=mass, center=list(center), size=list(size),
                rpy=list(rpy), label=label)


def mass_cylinder(mass, center, radius, length, rpy=(0, 0, 0), label='motor', inner=0):
    return dict(shape='cylinder', mass=mass, center=list(center), radius=radius,
                length=length, rpy=list(rpy), label=label, inner_radius=inner)


def distribute(mass, boxes):
    weights = np.array([np.prod(b['size']) for b in boxes])
    return [dict(b, mass=float(mass * w / weights.sum()), label='housing_volume_proxy')
            for b, w in zip(boxes, weights)]


def virtual_mass_parts(name, boxes):
    # Explicit engineering assumptions; motor masses are NOT manufacturer specs.
    if name == 'base_link':
        return [mass_box(5.6, [0, 0, -.15], [.55, .35, .12], label='low_battery_ballast_budget')] + distribute(2.4, boxes)
    if name == 'arm_mount_link':
        return [mass_box(1.8, [-.005, 0, .084], [.23, .29, .168], label='fixed_base_budget')] + distribute(1.2, boxes)
    if name == 'column_link':
        return [mass_cylinder(1.6, [0, 0, .05], .08, .10),
                mass_cylinder(2.4, [0, 0, .36], .1, .72, inner=.095, label='column_tube_budget')]
    if name == 'arm_rotating_base_link':
        return [mass_cylinder(.6, [0, 0, .10], .065, .06, [0, np.pi/2, 0])] + distribute(.4, boxes)
    if name == 'upper_arm_link':
        return [mass_cylinder(1.6, [.04, 0, 0], .075, .06, [0, np.pi/2, 0])] + distribute(2.4, boxes)
    if name == 'forearm_link':
        return [mass_cylinder(1.05, [.11, 0, 0], .075, .035, [0, np.pi/2, 0])] + distribute(1.95, boxes)
    return [mass_cylinder(.45, [0, 0, -.09], .05, .08)] + distribute(.55, boxes)


def aggregate(parts):
    mass = sum(p['mass'] for p in parts)
    center = sum(p['mass'] * np.array(p['center']) for p in parts) / mass
    inertia = np.zeros((3, 3))
    for p in parts:
        m = p['mass']
        if p['shape'] == 'box':
            x, y, z = p['size']
            diagonal = m / 12 * np.array([y*y+z*z, x*x+z*z, x*x+y*y])
        else:
            rr = p['radius']**2 + p.get('inner_radius', 0)**2
            transverse = m / 12 * (3*rr + p['length']**2)
            diagonal = [transverse, transverse, .5*m*rr]
        rotation = Rotation.from_euler('xyz', p['rpy']).as_matrix()
        d = np.array(p['center']) - center
        inertia += rotation @ np.diag(diagonal) @ rotation.T + m * (np.dot(d, d)*np.eye(3)-np.outer(d, d))
    return mass, center, inertia


def generate():
    robot = ET.Element('robot', {'xmlns:xacro': XACRO})
    # Namespace declaration is supplied by registered macro tags below.
    del robot.attrib['xmlns:xacro']
    report = {'assumptions': 'Virtual mass budgets and volume-weighted box proxies, not measured hardware. Component boxes fill internal cavities; not convex decomposition.', 'links': {}}
    for name, (mesh, pivot, budget) in LINKS.items():
        vertices, components = mesh_components(PARTS / (mesh + '.obj'), np.array(pivot))
        boxes = remove_contained_boxes([box_for_points(c) for c in components])
        parts = virtual_mass_parts(name, boxes)
        mass, center, inertia = aggregate(parts)
        assert abs(mass - budget) < 1e-10
        macro = ET.SubElement(robot, '{'+XACRO+'}macro', name='ax_physics_'+name)
        for i, box in enumerate(boxes):
            collision = ET.SubElement(macro, 'collision', name=f'{name}_part_{i:02d}')
            ET.SubElement(collision, 'origin', xyz=numbers(box['center']), rpy=numbers(box['rpy']))
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=numbers(box['size']))
        inertial = ET.SubElement(macro, 'inertial')
        ET.SubElement(inertial, 'origin', xyz=numbers(center), rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value=f'{mass:.12g}')
        ET.SubElement(inertial, 'inertia', **{key: f'{inertia[i,j]:.12g}' for key, i, j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]})
        aabb_volume = float(np.prod(np.ptp(vertices, axis=0)))
        report['links'][name] = dict(mesh=mesh+'.obj', pivot_m=pivot,
            mesh_bounds_link_m=[vertices.min(0).tolist(), vertices.max(0).tolist()],
            mesh_connected_components=len(components), collision_count=len(boxes), all_component_vertices_enclosed=True,
            collision_box_volume_sum_m3=sum(float(np.prod(b['size'])) for b in boxes),
            old_single_aabb_volume_m3=aabb_volume,
            mass_kg=mass, center_of_mass_m=center.tolist(), inertia_kg_m2=inertia.tolist(),
            mass_parts=parts, collision_boxes=boxes)
    ET.indent(robot, space='  ')
    OUT.write_text('<?xml version="1.0"?>\n<!-- Generated by tools/generate_physical_properties.py; do not edit manually. -->\n'+ET.tostring(robot, encoding='unicode')+'\n')
    destination = ROOT / 'src/ax_description/config/physical_properties.json'
    destination.write_text(json.dumps(report, indent=2)+'\n')
    print('Generated', OUT.relative_to(ROOT))
    for name, link in report['links'].items():
        print(name, 'collisions', link['collision_count'], 'COM', np.round(link['center_of_mass_m'], 4))


if __name__ == '__main__':
    generate()
