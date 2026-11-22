"""D-03 실행 ID 최소 구현과 기존 ax_d4 함수(RFC 8785 모듈 사용)의 고정 벡터 대조.

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_execution_ids.py
"""
import hashlib
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))

from ax_fleet_manager.execution_ids import make_execution_id  # noqa: E402

VECTORS = [
    ('root_task_dev_1', 'load_pick_and_stow', 'PICK_AND_STOW', 0),
    ('root_task_dev_1', 'src_nav_transporter', 'NAVIGATE_TO', 0),
    ('exec-0f8e', 'unload_confirm_empty', 'CONFIRM_LOAD', 7),
    ('루트_작업', '단계_키', 'RETRACT_ARM', 2**32 - 1),
    ('root"quote\\back', 'tab\tnew\nline', 'LOCK_HANDOFF', 1),
    ('ctl\x01\x1f\x7f', 'emoji_😀', 'UNLOCK_HANDOFF', 3),
    ('a', 'b', 'c', 0),
]


# 기존 D4 구현(rfc8785 사용)과 같은 값인지 대조한다. rfc8785가 없으면 시험을 진행하지 않는다
@pytest.mark.parametrize('vector', VECTORS)
def test_matches_existing_d4_function(vector):
    pytest.importorskip('rfc8785')
    sys.path.insert(0, str(ROOT / 'src' / 'ax_d4' / 'ax_d4'))
    from common_contract import make_execution_id as d4_make_execution_id

    assert make_execution_id(*vector) == d4_make_execution_id(*vector)


# 계약 문서 정의를 직접 계산한 고정값(외부 모듈 없이도 확인)
def test_fixed_vector_by_definition():
    text = ('{"attempt_index":0,"operation":"PICK_AND_STOW",'
            '"root_execution_id":"root_task_dev_1","step_key":"load_pick_and_stow"}')
    expected = hashlib.sha256(text.encode('utf-8')).hexdigest()[:32]

    value = make_execution_id('root_task_dev_1', 'load_pick_and_stow', 'PICK_AND_STOW', 0)
    assert value == expected
    assert len(value) == 32 and value == value.lower()


# 같은 입력은 같은 ID, 단계·operation·시도가 다르면 다른 ID
def test_distinct_inputs_give_distinct_ids():
    base = make_execution_id('r', 's', 'PICK_AND_STOW', 0)
    assert base == make_execution_id('r', 's', 'PICK_AND_STOW', 0)
    assert base != make_execution_id('r', 's2', 'PICK_AND_STOW', 0)
    assert base != make_execution_id('r', 's', 'UNLOAD_AND_PLACE', 0)
    assert base != make_execution_id('r', 's', 'PICK_AND_STOW', 1)


# 잘못된 입력 거부
@pytest.mark.parametrize('args', [
    ('', 's', 'OP', 0), ('r', '', 'OP', 0), ('r', 's', '', 0),
    ('r', 's', 'OP', -1), ('r', 's', 'OP', 2**32), ('r', 's', 'OP', True),
    ('r', 's', 'OP', 1.0), (None, 's', 'OP', 0), ('\ud800', 's', 'OP', 0),
])
def test_rejects_invalid_input(args):
    with pytest.raises(ValueError):
        make_execution_id(*args)
