"""두 창고 기준 월드(질감, ax_warehouse.world)가 단색 보존 월드(ax_warehouse_plain.world)와 재질 외 같고,
질감 파일·패키지 경로·재생성 결과가 맞는지 확인한다.

Gazebo를 띄우지 않는 정적 검사다. 질감이 실제 화면에 보이는지는 별도 실행으로 확인한다.
질감 이미지 재생성 비교(gen_textures.py)는 OpenCV·libjpeg 버전에 따라 JPG 바이트가 달라질 수 있어
AX_TEXTURE_REGEN=1일 때만 실행한다(약 6 s, 메모리 약 0.8 GB).
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]
PLANNED = ROOT / 'src/ax_description/worlds/planned'
LAUNCH = ROOT / 'src/ax_description/launch/robot_test.launch.py'


# tools/make_textured_world.py를 모듈로 읽는다.
def load_tool():
    path = ROOT / 'tools/make_textured_world.py'
    spec = importlib.util.spec_from_file_location('make_textured_world', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TOOL = load_tool()


class TexturedWorld(unittest.TestCase):

    # 단색 월드와 구조(태그·속성·텍스트·자식 수)가 같고, 차이는 material 아래 pbr 240개뿐이다
    def test_structure_matches_base_except_pbr(self):
        base = TOOL.BASE_WORLD.read_text(encoding='utf-8')
        textured = TOOL.TEXTURED_WORLD.read_text(encoding='utf-8')

        errors, pbr_list = TOOL.compare_structure(base, textured)

        self.assertEqual(errors, [])
        self.assertEqual(len(pbr_list), 240)
        self.assertNotIn('<pbr>', base)

    # 단색 월드에 없는 줄은 pbr 블록뿐이고, 단색 월드 줄은 하나도 빠지지 않는다
    def test_only_pbr_lines_added(self):
        base_lines = TOOL.BASE_WORLD.read_text(encoding='utf-8').splitlines()
        textured_lines = TOOL.TEXTURED_WORLD.read_text(encoding='utf-8').splitlines()
        allowed = (
            '<pbr>', '</pbr>', '<metal>', '</metal>',
            '<albedo_map>', '<metalness>', '<roughness>',
            TOOL.PBR_COMMENT,
        )

        kept = [line for line in textured_lines if not line.strip().startswith(allowed)]

        self.assertEqual(kept, base_lines)
        self.assertEqual(len(textured_lines) - len(base_lines), 240 * 8)

    # Gazebo 월드 이름은 도구 호환을 위해 단색 월드와 같은 ax_test다
    def test_world_name_kept(self):
        textured = TOOL.TEXTURED_WORLD.read_text(encoding='utf-8')

        self.assertEqual(re.findall(r'<world name="([^"]+)"', textured), ['ax_test'])

    # 모든 albedo_map은 model://ax_description/... 패키지 경로이고 파일이 있으며 manifest 해시와 같다
    def test_albedo_maps_exist_with_package_uri(self):
        textured = TOOL.TEXTURED_WORLD.read_text(encoding='utf-8')
        manifest = json.loads(TOOL.MANIFEST.read_text(encoding='utf-8'))

        _, pbr_list = TOOL.compare_structure(TOOL.BASE_WORLD.read_text(encoding='utf-8'), textured)
        uris = {item['albedo_map'] for item in pbr_list}

        self.assertEqual(TOOL.texture_problems(pbr_list, manifest), [])
        self.assertEqual(len(uris), 33)
        prefix = 'model://ax_description/worlds/planned/textures/'
        self.assertTrue(all(uri.startswith(prefix) for uri in uris))
        # 작성 PC 절대 경로가 없어야 한다(check_portability.py와 같은 방식으로 문자열을 나눠 적는다)
        for prefix in ('/' + 'home/', '/' + 'mnt/', 'file' + '://'):
            self.assertNotIn(prefix, textured)

    # 질감 폴더에는 manifest에 적힌 파일과 manifest 자신만 있고, 해시가 manifest와 같다
    def test_texture_folder_matches_manifest(self):
        manifest = json.loads(TOOL.MANIFEST.read_text(encoding='utf-8'))
        present = {path.name for path in TOOL.TEXTURE_DIR.iterdir()}

        self.assertEqual(present, set(manifest['files']) | {'textures_manifest.json'})

        for name, record in manifest['files'].items():
            data = (TOOL.TEXTURE_DIR / name).read_bytes()
            self.assertEqual(len(data), record['bytes'], name)
            self.assertEqual(hashlib.sha256(data).hexdigest(), record['sha256'], name)

    # 단색 월드에서 다시 만들면 저장된 기준 월드와 바이트가 같다(--check 종료 코드 0)
    def test_regenerated_world_identical(self):
        result = TOOL.check()

        self.assertTrue(result['regenerated_identical'])
        self.assertTrue(result['ok'], result)

        command = [sys.executable, str(ROOT / 'tools/make_textured_world.py'), '--check']
        completed = subprocess.run(command, capture_output=True, text=True, timeout=60)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)

    # 단색 월드 구조가 바뀌면(모델 하나 이동) 확인이 실패한다
    def test_check_detects_base_change(self):
        base = TOOL.BASE_WORLD.read_text(encoding='utf-8')
        textured = TOOL.TEXTURED_WORLD.read_text(encoding='utf-8')
        moved = re.sub(
            r'(<model name="floor_source">.*?<pose>)([^<]+)', r'\g<1>0 0 0 0 0 0',
            base, count=1, flags=re.S)

        self.assertNotEqual(moved, base)

        errors, _ = TOOL.compare_structure(moved, textured)
        self.assertTrue(errors)

    # launch는 기준 월드(기본)와 단색 월드를 선택지로 받고 두 월드에 같은 출발 위치를 쓴다
    # 값은 config/worlds.yaml 한 곳에서 읽는다(launch 소스에 목록을 두지 않음).
    def test_launch_lists_both_worlds(self):
        source = LAUNCH.read_text(encoding='utf-8')
        catalog = yaml.safe_load((LAUNCH.parents[1] / 'config/worlds.yaml').read_text(encoding='utf-8'))
        worlds = catalog['worlds']

        self.assertEqual(
            worlds['planned/ax_warehouse_plain.world']['spawn_positions'],
            worlds['planned/ax_warehouse.world']['spawn_positions'])
        self.assertEqual(list(worlds), ['planned/ax_warehouse.world', 'planned/ax_warehouse_plain.world'])
        self.assertEqual(catalog['default'], 'planned/ax_warehouse.world')
        self.assertIn("default_value=WORLD_CATALOG['default']", source)
        self.assertIn('choices=WORLD_CHOICES', source)

    # (선택) 질감 이미지를 v2_slim 설정으로 다시 만들면 저장된 34개 파일과 바이트가 같다
    @unittest.skipUnless(os.environ.get('AX_TEXTURE_REGEN') == '1',
                         'AX_TEXTURE_REGEN=1일 때만 실행(OpenCV JPG 바이트 의존)')
    def test_regenerated_textures_identical(self):
        with tempfile.TemporaryDirectory() as folder:
            command = [
                sys.executable, str(ROOT / 'tools/gen_textures.py'), str(TOOL.BASE_WORLD), folder,
                '--ppm', '220', '--upright-ppm', '512', '--deck-ppm', '256',
                '--box-px', '512', '--sample-px', '256', '--format', 'jpg', '--jpg-quality', '92',
            ]
            subprocess.run(command, check=True, capture_output=True, timeout=300)

            for path in sorted(TOOL.TEXTURE_DIR.iterdir()):
                regenerated = (Path(folder) / path.name).read_bytes()
                self.assertEqual(regenerated, path.read_bytes(), path.name)


if __name__ == '__main__':
    unittest.main()
