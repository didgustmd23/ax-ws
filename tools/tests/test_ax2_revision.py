"""새 AX_2 CAD·충돌·명칭·적재 센서·두 월드 계약을 함께 검증한다."""
import importlib.util
import itertools
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT/'src/ax_description'
sys.path.insert(0, str(ROOT/'tools'))
from arm_geometry import origin


class Ax2Revision(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ET.fromstring(xacro.process_file(str(PKG/'urdf/ax_2.urdf.xacro')).toxml())
        cls.frames = {'base_link': np.eye(4)}
        pending = list(cls.robot.findall('joint'))
        while pending:
            before = len(pending)
            for joint in pending[:]:
                parent = joint.find('parent').get('link')
                if parent in cls.frames:
                    cls.frames[joint.find('child').get('link')] = cls.frames[parent] @ origin(joint.find('origin'))
                    pending.remove(joint)
            if len(pending)==before:
                raise AssertionError('Disconnected model')

    def test_current_cad_parts_reproduce_manifest_and_no_retired_housing(self):
        from update_ax2_parts import generated
        folder = PKG/'meshes/parts/AX_2'
        for name, data in generated().items():
            self.assertEqual((folder/name).read_bytes(), data, name)
        self.assertEqual(len(list(folder.glob('*.obj'))), 7)
        self.assertNotIn('top_lidar_housing_link', self.frames)
        self.assertTrue((PKG/'meshes/versions/AX_2_previous/ax_top_lidar.obj').is_file())

    def test_all_replaced_and_retained_mesh_vertices_enclosed_by_collisions(self):
        report = json.loads((PKG/'config/ax_2_physical_properties.json').read_text())
        for name, data in report['links'].items():
            vertices = np.array([list(map(float,s.split()[1:4])) for s in
                (PKG/'meshes/parts/AX_2'/data['mesh']).read_text().splitlines() if s.startswith('v ')])*.001
            vertices -= data['pivot_m']
            covered = np.zeros(len(vertices), dtype=bool)
            for c in self.robot.find(f"link[@name='{name}']").findall('collision'):
                t = origin(c.find('origin'))
                local = (vertices-t[:3,3]) @ t[:3,:3]
                half = np.fromstring(c.find('geometry/box').get('size'),sep=' ')/2
                covered |= np.all(abs(local)<=half+1e-7,axis=1)
            self.assertTrue(covered.all(), name)

    def test_updated_footprint_encloses_every_box_and_wheel_collision(self):
        footprint = np.array(yaml.safe_load((PKG/'config/a1_validation.yaml').read_text())['mobility']['robot_2']['footprint_xy_m'])
        corners = np.array(list(itertools.product((-1,1),repeat=3)))
        for link in self.robot.findall('link'):
            for c in link.findall('collision'):
                geom = c.find('geometry')
                if geom.find('box') is not None:
                    half = np.fromstring(geom.find('box').get('size'),sep=' ')/2
                elif geom.find('cylinder') is not None:
                    cy=geom.find('cylinder');half=np.array([float(cy.get('radius'))]*2+[float(cy.get('length'))/2])
                else:
                    self.fail('Unexpected collision geometry')
                t=self.frames[link.get('name')] @ origin(c.find('origin'))
                points=(corners*half) @ t[:3,:3].T+t[:3,3]
                self.assertTrue(np.all(points[:,:2]>=footprint.min(0)-1e-8),link.get('name'))
                self.assertTrue(np.all(points[:,:2]<=footprint.max(0)+1e-8),link.get('name'))

    def test_cargo_camera_removed_and_contact_deck_preserved(self):
        cfg=yaml.safe_load((PKG/'config/cargo_height.yaml').read_text())['/**']['ros__parameters']
        deck=self.frames['cargo_link']
        self.assertNotIn('cargo_rgbd_link',self.frames)
        self.assertNotIn('cargo_rgbd_optical_frame',self.frames)
        self.assertIsNone(self.robot.find(".//sensor[@name='cargo_rgbd']"))
        self.assertIsNotNone(self.robot.find(".//sensor[@name='front_rgbd']"))
        self.assertIsNotNone(self.robot.find(".//sensor[@name='cargo_contact']"))
        spec=importlib.util.spec_from_file_location('camera_removal_launch',PKG/'launch/robot_test.launch.py')
        launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
        self.assertFalse(any('cargo_rgbd' in b['ros_topic_name'] for b in launch.bridges('robot_2',False)))
        self.assertAlmostEqual(cfg['max_height_m'],.5)
        self.assertAlmostEqual(deck[2,3], .573-.33208145)

    def test_specs_labels_and_aggregate_mass_consistent(self):
        from generate_robot_specs import calculate
        labels=yaml.safe_load((PKG/'config/part_labels.yaml').read_text())
        self.assertTrue(all(link.get('name') in labels for link in self.robot.findall('link') if link.find('visual') is not None))
        data=calculate(2)
        self.assertEqual(json.loads((PKG/'config/ax2_spec.json').read_text()),data)
        self.assertAlmostEqual(data['total_mass_kg'],sum(p['mass_kg'] for p in data['links']))
        self.assertTrue(data['unloaded_com_xy_inside_wheel_centers'])
        self.assertTrue(np.all(np.linalg.eigvalsh(data['inertia_at_total_com_kg_m2'])>0))

    def test_worlds_three_clear_corridors_and_all_size_samples(self):
        self.assertEqual({p.relative_to(ROOT).as_posix() for p in ROOT.rglob('*.world')
                          if not any(part in ('build','install','log','.git') for part in p.parts)},
                         {'src/ax_description/worlds/planned/ax_warehouse.world'})
        world=ET.parse(PKG/'worlds/planned/ax_warehouse.world').getroot().find('world')
        models={m.get('name'):m for m in world.findall('model')}
        self.assertEqual(sum(n.startswith('floor_transfer_corridor') for n in models),3)
        self.assertEqual(sum(n.startswith('cargo_box_') for n in models),120)
        self.assertEqual(sum(n.startswith('sample_') for n in models),23)
        for name,m in models.items():
            if not name.startswith('floor_transfer_corridor'):continue
            self.assertEqual(m.findtext('static'),'true')
            width=float(m.findtext('link/collision/geometry/box/size').split()[1])
            self.assertAlmostEqual(width,1.8)

    def test_compact_stacks_have_support_and_rear_routes_are_clear(self):
        world=ET.parse(PKG/'worlds/planned/ax_warehouse.world').getroot().find('world')
        models={m.get('name'):m for m in world.findall('model')}
        boxes={}
        for name,m in models.items():
            if name.startswith(('cargo_box_','sample_')):
                self.assertEqual(m.findtext('static'),'false',name)
                boxes[name]=(np.fromstring(m.findtext('pose'),sep=' ')[:3],
                             np.fromstring(m.findtext('link/collision/geometry/box/size'),sep=' '))
        for name,(p,size) in boxes.items():
            bottom=p[2]-size[2]/2
            if bottom>.005:
                supported=any(abs(bottom-(q[2]+other[2]/2))<.004 and
                    np.all(abs(p[:2]-q[:2])+size[:2]/2<=other[:2]/2+1e-6)
                    for other_name,(q,other) in boxes.items() if other_name!=name)
                self.assertTrue(supported,name)
            for other_name,(q,other) in boxes.items():
                if other_name!=name:
                    self.assertFalse(np.all(abs(p-q)<(size+other)/2-1e-6),(name,other_name))
        catalog=json.loads((PKG/'worlds/planned/box_samples.json').read_text())
        self.assertEqual(len(catalog['layout']['stacks']),54)
        for sample in catalog['samples']:
            np.testing.assert_allclose(sample['position_m'],boxes[sample['model']][0])
        for lane in catalog['layout']['clear_lanes']:
            xmin,ymin,xmax,ymax=lane['bounds_xy_m']
            self.assertGreaterEqual(ymax-ymin,1.8-1e-8)
            for name,m in models.items():
                if name.startswith('floor'):continue
                shape=m.find('link/collision/geometry/box/size')
                if shape is None:continue
                x,y,_=np.fromstring(m.findtext('pose'),sep=' ')[:3]
                sx,sy,_=np.fromstring(shape.text,sep=' ')
                self.assertFalse(x+sx/2>xmin and x-sx/2<xmax and
                    y+sy/2>ymin and y-sy/2<ymax,(lane['name'],name))


if __name__=='__main__':
    unittest.main()
