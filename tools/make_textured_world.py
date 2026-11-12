#!/usr/bin/env python3
r"""단색 두 창고 월드(ax_warehouse_plain.world)에서 기준 질감 월드(ax_warehouse.world)를 만들거나,
저장된 기준 월드가 맞는지 확인한다.

단색 월드 원문은 그대로 두고, manifest에 적힌 모델의 `</material>` 바로 앞에 `<pbr>` 블록만 끼워 넣는다.
geometry·pose·collision·inertial·plugin 등 다른 요소는 바꾸지 않는다.
질감 경로는 `model://ax_description/worlds/planned/textures/<파일>` 형식이다.
robot_test.launch.py가 ax_description share의 상위 폴더를 GZ_SIM_RESOURCE_PATH에 넣으므로 설치본에서 풀린다.

질감 이미지는 tools/gen_textures.py로 만든다(v2_slim 설정, seed 20261112).
  python3 tools/gen_textures.py src/ax_description/worlds/planned/ax_warehouse_plain.world <빈 폴더> \
    --ppm 220 --upright-ppm 512 --deck-ppm 256 --box-px 512 --sample-px 256 \
    --format jpg --jpg-quality 92

사용:
  python3 tools/make_textured_world.py          # 단색 월드 → 기준(질감) 월드 생성(덮어씀)
  python3 tools/make_textured_world.py --check  # 재생성 결과·구조·질감 파일을 확인(파일을 쓰지 않음)
종료 코드: 정상 0, 확인 실패 1
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PLANNED = ROOT / 'src/ax_description/worlds/planned'
# 구조의 원본(단색)과 생성 결과(기준 월드). 구조를 바꿀 때는 단색 월드를 고치고 다시 만든다.
BASE_WORLD = PLANNED / 'ax_warehouse_plain.world'
TEXTURED_WORLD = PLANNED / 'ax_warehouse.world'
TEXTURE_DIR = PLANNED / 'textures'
MANIFEST = TEXTURE_DIR / 'textures_manifest.json'
URI_PREFIX = 'model://ax_description/worlds/planned/textures'

# pbr 블록 첫 줄 주석. 구조 비교에서 허용하는 추가 줄을 가려내는 데도 쓴다.
PBR_COMMENT = '<!--질감: 기존 ambient·diffuse 색에 곱해지는 밝기 변조 이미지(tools/gen_textures.py 절차 생성).-->'

# 재질 반응: 도장·콘크리트·합판·골판지는 비금속으로 둔다. SDF 기본값(metalness 0.5)을 피하려고 명시한다.
SURFACE = {
    'concrete': {'metalness': 0.0, 'roughness': 0.9},
    'wall': {'metalness': 0.0, 'roughness': 0.8},
    'rack_upright': {'metalness': 0.0, 'roughness': 0.7},
    'shelf_deck': {'metalness': 0.0, 'roughness': 0.9},
    'cargo_box': {'metalness': 0.0, 'roughness': 0.9},
    'sample': {'metalness': 0.0, 'roughness': 0.9},
}


# 모델 블록 하나 안의 첫 `</material>` 앞에 pbr 블록을 넣은 새 텍스트를 반환한다.
def insert_pbr(block, texture_uri, surface, indent):
    pbr = (
        f'{indent}  {PBR_COMMENT}\n'
        f'{indent}  <pbr>\n'
        f'{indent}    <metal>\n'
        f'{indent}      <albedo_map>{texture_uri}</albedo_map>\n'
        f'{indent}      <metalness>{surface["metalness"]}</metalness>\n'
        f'{indent}      <roughness>{surface["roughness"]}</roughness>\n'
        f'{indent}    </metal>\n'
        f'{indent}  </pbr>\n'
    )

    position = block.index('</material>')
    line_start = block.rindex('\n', 0, position) + 1
    return block[:line_start] + pbr + block[line_start:]


# 기준 월드 텍스트와 manifest로 질감 월드 텍스트를 만든다. 파일은 쓰지 않는다.
def build_textured(base_text, manifest, uri_prefix=URI_PREFIX):
    text = base_text

    for target in manifest['targets']:
        name = target['model']

        # 모델 블록은 `<model name="...">`부터 다음 `</model>`까지다(중첩 모델 없음).
        match = re.search(r'<model name="%s">.*?</model>' % re.escape(name), text, flags=re.S)
        if match is None or match.group(0).count('<material>') != 1:
            raise ValueError('대상 모델의 material이 정확히 하나가 아님: ' + name)

        block = match.group(0)
        indent = re.search(r'\n( *)<material>', block).group(1)
        texture_uri = uri_prefix + '/' + target['texture']
        new_block = insert_pbr(block, texture_uri, SURFACE[target['kind']], indent)
        text = text[:match.start()] + new_block + text[match.end():]

    return text


# 두 XML 요소를 재귀 비교한다. 질감 월드 material의 pbr 자식은 따로 모으고 비교에서 뺀다.
def walk(base, textured, path, errors, pbr_list):
    if base.tag != textured.tag or base.attrib != textured.attrib:
        errors.append({
            'path': path,
            'base': [base.tag, base.attrib],
            'textured': [textured.tag, textured.attrib],
        })
        return

    if (base.text or '').strip() != (textured.text or '').strip():
        errors.append({'path': path, 'text_base': base.text, 'text_textured': textured.text})

    textured_children = list(textured)

    if textured.tag == 'material':
        if any(child.tag == 'pbr' for child in base):
            errors.append({'path': path, 'error': '기준 월드에 이미 pbr가 있음'})

        for item in textured_children:
            if item.tag == 'pbr':
                pbr_list.append({
                    'path': path,
                    'albedo_map': item.findtext('metal/albedo_map'),
                    'metalness': item.findtext('metal/metalness'),
                    'roughness': item.findtext('metal/roughness'),
                })

        textured_children = [child for child in textured_children if child.tag != 'pbr']

    base_children = list(base)
    if len(base_children) != len(textured_children):
        errors.append({
            'path': path,
            'children_base': len(base_children),
            'children_textured': len(textured_children),
        })
        return

    for index, (child_base, child_textured) in enumerate(zip(base_children, textured_children)):
        label = child_base.get('name')
        child_path = '%s/%s[%s]' % (path, child_base.tag, label if label else index)
        walk(child_base, child_textured, child_path, errors, pbr_list)


# 기준·질감 월드 텍스트의 구조를 비교해 (구조 오류 목록, pbr 목록)을 돌려준다.
def compare_structure(base_text, textured_text):
    errors, pbr_list = [], []
    walk(ET.fromstring(base_text), ET.fromstring(textured_text), 'sdf', errors, pbr_list)
    return errors, pbr_list


# albedo_map 경로가 패키지 경로이고, 파일이 질감 폴더에 있으며 manifest 해시와 같은지 확인한다.
def texture_problems(pbr_list, manifest, texture_dir=TEXTURE_DIR, uri_prefix=URI_PREFIX):
    problems = []
    listed = manifest.get('files', {})

    for name in sorted({item['albedo_map'] for item in pbr_list}):
        if not name.startswith(uri_prefix + '/'):
            problems.append('패키지 경로가 아님: ' + name)
            continue

        filename = name[len(uri_prefix) + 1:]
        path = texture_dir / filename

        if not path.is_file():
            problems.append('질감 파일 없음: ' + filename)
        elif filename not in listed:
            problems.append('manifest files에 없음: ' + filename)
        elif hashlib.sha256(path.read_bytes()).hexdigest() != listed[filename]['sha256']:
            problems.append('manifest 해시 불일치: ' + filename)

    return problems


# 저장된 질감 월드를 확인하고 결과 사전을 돌려준다.
def check(base_path=BASE_WORLD, textured_path=TEXTURED_WORLD, manifest_path=MANIFEST,
          texture_dir=TEXTURE_DIR):
    base_text = base_path.read_text(encoding='utf-8')
    textured_text = textured_path.read_text(encoding='utf-8')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))

    errors, pbr_list = compare_structure(base_text, textured_text)
    problems = texture_problems(pbr_list, manifest, texture_dir)
    regenerated = build_textured(base_text, manifest) == textured_text

    complete = len(pbr_list) == len(manifest['targets'])

    return {
        'ok': not errors and not problems and regenerated and complete,
        'regenerated_identical': regenerated,
        'structural_error_count': len(errors),
        'structural_errors': errors[:20],
        'changed_material_count': len(pbr_list),
        'manifest_target_count': len(manifest['targets']),
        'texture_files': len({item['albedo_map'] for item in pbr_list}),
        'texture_problems': problems,
    }


# 명령행 진입점: 생성 또는 확인.
def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--check', action='store_true', help='파일을 쓰지 않고 저장된 질감 월드를 확인')
    args = parser.parse_args()

    if args.check:
        result = check()
        print(json.dumps(result, indent=1, ensure_ascii=False))
        sys.exit(0 if result['ok'] else 1)

    manifest = json.loads(MANIFEST.read_text(encoding='utf-8'))
    text = build_textured(BASE_WORLD.read_text(encoding='utf-8'), manifest)
    TEXTURED_WORLD.write_text(text, encoding='utf-8')
    print('changed materials', len(manifest['targets']), '->', TEXTURED_WORLD.relative_to(ROOT))


if __name__ == '__main__':
    main()
