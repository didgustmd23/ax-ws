"""실행 월드 목록(src/ax_description/config/worlds.yaml)을 읽는 도구 공용 함수.

ROS 환경이 없어도(바탕화면 런처) 쓸 수 있게 저장소 소스 파일을 먼저 읽고, 없으면 설치본을 찾는다.
"""
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


# 월드 목록 사전을 돌려준다({'default': ..., 'worlds': {이름: {gz_world, structure, spawn_positions}}})
def load(workspace=None):
    base = Path(workspace) if workspace else ROOT
    for path in (base / 'src/ax_description/config/worlds.yaml',
                 base / 'install/ax_description/share/ax_description/config/worlds.yaml'):
        if path.is_file():
            return yaml.safe_load(path.read_text(encoding='utf-8'))

    raise FileNotFoundError('worlds.yaml not found under ' + str(base))


# 선택 가능한 월드 이름 목록(기본 월드가 먼저)
def choices(workspace=None):
    catalog = load(workspace)
    names = list(catalog['worlds'])

    return [catalog['default']] + [name for name in names if name != catalog['default']]


# 기본 월드 이름
def default(workspace=None):
    return load(workspace)['default']
