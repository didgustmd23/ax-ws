"""OBJ 인덱스 재배치와 조립 좌표 보존의 회귀 검사."""
import importlib.util
from collections import Counter
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve().parents[1] / 'split_robot_meshes.py'
MESHES = SCRIPT.parents[1] / 'src/ax_description/meshes'


# 분할기와 독립적인 OBJ 판독기. 인덱스 번호가 아니라 풀어낸 면 내용(좌표·UV·법선·재질·스무딩)을
# 객체별로 모아 비교에 쓴다.
def face_values(path, include_material=True):
    vertices, uv, normals = [], [], []
    current, material, smooth = None, None, 'off'
    result = {}

    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue

        key = fields[0]
        if key in ('v', 'vt', 'vn'):
            {'v': vertices, 'vt': uv, 'vn': normals}[key].append(tuple(map(float, fields[1:])))
        elif key == 'o':
            current = fields[1]
            result.setdefault(current, Counter())
        elif key == 'usemtl':
            material = fields[1]
        elif key == 's':
            smooth = fields[1]
        elif key == 'f':
            corners = []

            for token in fields[1:]:
                corner = []
                for table, index in zip((vertices, uv, normals), token.split('/')):
                    corner.append(table[int(index) - 1 if int(index) > 0 else int(index)]
                                  if index else None)
                corners.append(tuple(corner))

            value = (tuple(corners), material, smooth) if include_material else tuple(corners)
            result[current][value] += 1

    return result


# OBJ 분할기(split_robot_meshes.py)의 좌표·면·재질 보존을 확인하는 시험.
class SplitMeshTests(unittest.TestCase):
    # 분할 스크립트를 불러오고 시험별 임시 폴더를 만든다.
    def setUp(self):
        self.assertTrue(SCRIPT.exists(), 'Reproducible OBJ splitter is missing')
        spec = importlib.util.spec_from_file_location('split_robot_meshes', SCRIPT)
        self.splitter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.splitter)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    # 분할 후 객체마다 좌표·UV·법선·재질·스무딩이 유지되는지 확인한다(음수 인덱스 포함).
    def test_objects_keep_coordinates_uv_normals_material_and_smoothing(self):
        source = self.root / 'AX_2.obj'
        # 내보내기 도구는 객체 선언을 정점 표 뒤에 둔다. 두 번째 면은 음수 인덱스를 쓰고
        # 일부러 첫 정점을 다시 쓴다.
        source.write_text('mtllib AX_2.mtl\n'
                          'v 10 20 30\nv 11 20 30\nv 10 21 30\n'
                          'vt 0 0\nvt 1 0\nvt 0 1\nvn 0 0 1\n'
                          'o AX_2_Body\ng AX_2_Body\nusemtl red\ns 2\n'
                          'f 1/1/1 2/2/1 3/3/1\n'
                          'v 12 20 30\no AX_2_Shelf\ng AX_2_Shelf\n'
                          'usemtl blue\ns off\nf -4/3/1 -1/2/1 -2/1/1\n')
        (self.root / 'AX_2.mtl').write_text('newmtl red\nnewmtl blue\n')
        original = source.read_bytes()

        output = self.root / 'parts'
        manifest = self.splitter.split_obj(source, output)
        self.assertEqual((output / manifest['source']).resolve(), source.resolve())

        body = (output / 'ax_body.obj').read_text().splitlines()
        shelf = (output / 'ax_shelf.obj').read_text().splitlines()
        self.assertEqual([line for line in body if line.startswith('v ')],
                         ['v 10 20 30', 'v 11 20 30', 'v 10 21 30'])
        self.assertEqual([line for line in shelf if line.startswith('v ')],
                         ['v 10 20 30', 'v 10 21 30', 'v 12 20 30'])

        self.assertIn('f 1/1/1 2/2/1 3/3/1', body)
        self.assertIn('f 1/3/1 3/2/1 2/1/1', shelf)
        self.assertIn('usemtl red', body)
        self.assertIn('s 2', body)
        self.assertIn('usemtl blue', shelf)
        self.assertIn('s off', shelf)

        self.assertEqual((output / 'AX_2.mtl').read_bytes(),
                         (self.root / 'AX_2.mtl').read_bytes())
        self.assertEqual(source.read_bytes(), original)
        self.assertEqual(len(manifest['parts']), 2)

    # 정점만 있거나 텍스처 인덱스가 빠진 면도 올바른 인덱스로 써지는지 확인한다.
    def test_vertex_only_and_missing_texture_indices_remain_valid(self):
        source = self.root / 'AX_1.obj'
        source.write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\n'
                          'o Gripper_Finger_01\nf 1 2 3\nf 1//1 3//1 2//1\n')
        output = self.root / 'parts'
        self.splitter.split_obj(source, output)
        lines = (output / 'gripper_finger_01.obj').read_text().splitlines()
        self.assertIn('f 1 2 3', lines)
        self.assertIn('f 1//1 3//1 2//1', lines)

    # 잘못된 정점 참조는 부품 파일을 쓰기 전에 거부되는지 확인한다.
    def test_invalid_reference_is_rejected_before_writing_parts(self):
        source = self.root / 'AX_1.obj'
        source.write_text('v 0 0 0\no AX1_Body\nf 1 2 3\n')
        output = self.root / 'parts'
        with self.assertRaises(ValueError):
            self.splitter.split_obj(source, output)
        self.assertFalse(output.exists())

    # 실제 원본 메시 3개의 모든 면이 자기 객체에 정확히 한 번씩 남는지 확인한다.
    def test_every_real_source_face_is_preserved_once_in_its_object(self):
        for stem in ('AX_1', 'AX_2', 'AX_Box'):
            with self.subTest(source=stem):
                source = MESHES / f'{stem}.obj'
                output = self.root / stem
                manifest = self.splitter.split_obj(source, output)
                original = face_values(source)
                self.assertEqual(len(manifest['parts']), len(original))
                self.assertEqual(manifest['source_sha256'],
                                 hashlib.sha256(source.read_bytes()).hexdigest())
                for part in manifest['parts']:
                    name = part['source_object']
                    self.assertEqual(face_values(output / part['file'])[name], original[name])

    # 받침·기둥 분리가 모든 면을 보존하고 기존 소유 구분(원통 + 링 3개)을 따르는지 확인한다.
    def test_column_partition_keeps_all_geometry_and_existing_ownership(self):
        output = self.root / 'AX_1'
        self.splitter.split_obj(MESHES / 'AX_1.obj', output)
        report = self.splitter.split_column(output)
        original = face_values(output / 'arm_01_fixed.obj', False)['Arm_01_Fixed']
        combined = Counter()
        for part in report['outputs']:
            faces = face_values(output / part['file'], False)
            combined.update(next(iter(faces.values())))
        self.assertEqual(combined, original)
        self.assertEqual(sum(c['assignment'] == 'column' for c in report['components']), 4)
        self.assertAlmostEqual(report['tube_length_m'], .5666586)
        self.assertEqual({p['part']: p['faces'] for p in report['outputs']},
                         {'column': 800, 'support': 1776})

    # 설치된 부품 manifest가 현재 원본 메시를 가리키고 원본·부품 파일 해시가 모두 일치하는지 확인한다.
    def test_installed_asset_manifests_resolve_and_match_current_sources(self):
        for folder in ('AX_1', 'AX_2', 'boxes'):
            directory = MESHES / 'parts' / folder
            manifest = json.loads((directory / 'manifest.json').read_text())
            source = (directory / manifest['source']).resolve()
            self.assertEqual(manifest['source_sha256'],
                             hashlib.sha256(source.read_bytes()).hexdigest())
            for part in manifest['parts']:
                self.assertEqual(
                    part['sha256'],
                    hashlib.sha256((directory / part['file']).read_bytes()).hexdigest())

    # 기둥 회전축이 차체 X/Y 중심에 있는지 확인한다.
    def test_column_rotation_axis_is_at_chassis_xy_center(self):
        robot = ET.parse(MESHES.parent / 'urdf/ax.urdf.xacro').getroot()
        mount = robot.find("joint[@name='arm_mount_joint']/origin")
        column = robot.find("joint[@name='column_yaw_joint']/origin")
        position = [sum(pair) for pair in zip(map(float, mount.get('xyz').split()),
                                              map(float, column.get('xyz').split()))]
        self.assertEqual(position[:2], [0., 0.])
        self.assertAlmostEqual(position[2], .39722675)


if __name__ == '__main__':
    unittest.main()
