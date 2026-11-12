"""채택 지도 ax_warehouse의 적용 월드 목록(applicable_worlds.yaml)이 실제 월드 파일·map_info와 맞는지 확인한다.

기준 월드나 단색 보존 월드가 바뀌면 이 시험이 실패한다.
그때는 LiDAR 동등성을 다시 확인한 뒤에만 목록의 해시를 고친다.
"""
import hashlib
from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]
MAP_DIR = ROOT / 'src/ax_localization/maps/ax_warehouse'
WORLDS = ROOT / 'src/ax_description/worlds'


# 파일의 sha256 16진 문자열을 돌려준다.
def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


# YAML 파일을 읽는다.
def load(name):
    return yaml.safe_load((MAP_DIR / name).read_text(encoding='utf-8'))


class MapApplicableWorlds(unittest.TestCase):

    # 목록의 모든 월드 해시가 저장소의 실제 월드 파일과 같다
    def test_listed_world_hashes_match_files(self):
        data = load('applicable_worlds.yaml')

        for entry in data['applicable_worlds']:
            with self.subTest(world=entry['launch_name']):
                self.assertEqual(sha256(WORLDS / entry['launch_name']), entry['sha256'])

    # 선택 가능한 두 창고 월드가 모두 목록에 있다
    def test_both_selectable_worlds_listed(self):
        data = load('applicable_worlds.yaml')

        names = {entry['launch_name'] for entry in data['applicable_worlds']}

        self.assertEqual(names, {'planned/ax_warehouse.world', 'planned/ax_warehouse_plain.world'})

    # 작성 월드는 map_info.yaml(생성 파일)의 월드 기록과 같은 바이트이고, revision도 같다
    def test_authored_world_matches_map_info(self):
        data = load('applicable_worlds.yaml')
        info = load('map_info.yaml')

        authored = data['authored_world']

        self.assertEqual(authored['sha256'], info['world']['world_sha256'])
        self.assertEqual(authored['sha256'], info['world']['world_copy']['sha256'])
        self.assertEqual(sha256(ROOT / authored['source']), authored['sha256'])
        self.assertEqual(data['map_revision'], info['map_revision'])


if __name__ == '__main__':
    unittest.main()
