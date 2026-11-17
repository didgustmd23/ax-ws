#!/usr/bin/env python3
"""C2 관측 표면 → C3 Surface 어댑터(순수 함수).

C2 관측과 출처가 명시된 외부 메타정보로 C3 ax_grasp_planner.planner.Surface를 만들 수 있는지 검사한다.
필수 근거가 하나라도 없으면 Surface를 만들지 않고 사유를 돌려준다(보류). 기본 정상값을 만들지 않는다.
  - 목표 대응: C2 VALID는 평면 적합 유효일 뿐이다(배경 평면일 수 있음). 현재 관측이 등록 물체의 것이라는
    외부 판정(TargetAssociation, 결과 MATCH)이 없으면 기본 보류한다. 어댑터는 판정이 현재 관측·로봇·물체·
    관측 stamp·처음 관측 frame과 결합돼 있는지만 대조한다. 외부 판정의 유효성·정확성 검증은 구현돼 있지 않다
    (판정 주체 C4 대응 판정기 미구현). 따라서 운영 정상 경로는 아직 구성할 수 없고, 판정 fixture는 합성 시험 전용이다.
  - object_id는 D 등록 ID를 외부에서 받는다. C2 observation_id로 대체하지 않는다.
  - confidence·mass_kg·suction_suitable은 외부 값이다. C2 품질 지표(plane RMS 등)로 만들지 않는다.
  - flatness_error_m은 명시적으로 고른 정의(plane_rms_m 또는 plane_p95_m, m)가 주어질 때만 C2 품질에서 옮긴다.
  - 법선: C2는 camera_facing 관측 규약이다. C3는 바깥 방향 법선을 요구하므로 바깥 방향 근거가 주어질 때만 넘긴다.
  - frame: C3/C4가 받을 frame 목록이 주어질 때만 넘긴다. 이 목록이 C4 실행 가능을 보장하지는 않는다.
  - 축 모호(axis_ambiguous) 관측은 허용 정책이 명시될 때만 넘긴다.
  - 시각: 권위 시각은 관측의 정수 stamp_ns다. 대응 판정도 observation_stamp_ns(정수)로 결합한다.
    비정확 시각(stamp_origin='legacy_float') 관측은 MATCH가 있어도 STAMP_NOT_EXACT로 보류한다.
    C3 Surface.stamp_s는 C3 소유 float API라 파생 초를 넘기며, C3 이후로는 ns 정밀성이 보존되지 않는다.
  - 문자열 상한: 보존·기록하는 외부 문자열(출처·object_id·판정 주체·방법·근거 등)은 호출자가 정한
    max_text_len(문자 수) 이하만 받는다. 넘으면 *_TEXT_LIMIT로 보류한다(기본값 없음).
출처 문자열은 기록용이며 근거가 검증됐다는 뜻이 아니다. C3 후보 계산 성공은 IK·충돌·흡착·조작 성공이 아니다.
"""

from dataclasses import dataclass
import math
from pathlib import Path
import sys

# C3 실제 Surface를 그대로 쓴다(패키지 설치 전에도 저장소 경로에서 import).
_PLANNER = Path(__file__).resolve().parents[2] / 'ax_grasp_planner'

if _PLANNER.is_dir() and str(_PLANNER) not in sys.path:
    sys.path.insert(0, str(_PLANNER))

from ax_grasp_planner.planner import Surface  # noqa: E402
from surface_estimation import is_integer, lineage_problem, VALIDITY_SCOPE  # noqa: E402

FLATNESS_DEFINITIONS = ('plane_rms_m', 'plane_p95_m')
ASSOCIATION_MATCH = 'MATCH'
ASSOCIATION_RESULTS = ('MATCH', 'NO_MATCH', 'UNKNOWN')


@dataclass(frozen=True)
class Sourced:
    # 값과 출처(필수 문자열). 출처는 기록용이며 검증 근거가 아니다.
    value: object
    source: str


@dataclass(frozen=True)
class TargetAssociation:
    # 현재 관측과 등록 물체의 대응 외부 판정. 어댑터는 결합·형식만 대조하며 판정의 정확성을 검증하지 않는다.
    observation_id: str           # 판정 대상 C2 관측 ID
    robot_id: str                 # 관측 로봇
    object_id: str                # 대응한다고 판정한 D 등록 object_id
    observation_stamp_ns: int     # 판정 대상 관측 stamp(정수 ns, 권위 값)
    observation_frame_id: str     # 판정 대상 관측의 처음 관측 frame(카메라 optical)
    judged_by: str                # 판정 주체(예: C4 구성 요소 이름)
    method: str                   # 판정 방법 이름
    evidence_ref: str             # 판정 근거 참조(기록·관측 ID 등)
    result: str                   # MATCH / NO_MATCH / UNKNOWN


@dataclass(frozen=True)
class AdapterInputs:
    # 외부 메타정보. None은 미정(출처 없음)이며 해당 사유로 보류한다.
    object_id: Sourced = None             # D 등록 object_id(str)
    scene_revision: Sourced = None        # 양의 정수
    mass_kg: Sourced = None               # 유한, ≥ 0
    confidence: Sourced = None            # 유한, 0~1
    suction_suitable: Sourced = None      # bool(False도 그대로 전달)
    flatness_definition: Sourced = None   # FLATNESS_DEFINITIONS 중 하나
    normal_outward: Sourced = None        # camera_facing 법선이 물체 바깥 방향이라는 근거(True만 통과)
    accepted_frames: Sourced = None       # C3/C4가 받을 frame 문자열 목록
    allow_axis_ambiguous: Sourced = None  # 축 모호 관측 전달 허용(bool)
    target_association: TargetAssociation = None  # 관측–등록 물체 대응 외부 판정(없으면 보류)


@dataclass(frozen=True)
class AdapterResult:
    surface: object       # C3 Surface 또는 None(보류)
    reasons: tuple        # 보류 사유(빈 튜플이면 전달)
    provenance: dict      # 필드별 출처 기록


# 출처가 있는 값인지 본다
def sourced(item):
    return isinstance(item, Sourced) and isinstance(item.source, str) and bool(item.source.strip())


# 문자열 길이 상한 안인지(문자열이 아니면 False)
def short_text(value, limit):
    return isinstance(value, str) and len(value) <= limit


# 유한 실수(bool 제외)
def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


# 대응 판정이 현재 관측·대상과 결합돼 있는지 대조한다. 사유 목록(정상이면 빈 목록)
def association_problems(association, observation, object_id, max_text_len):
    if association is None:
        return ['TARGET_ASSOCIATION_UNDEFINED']

    if not isinstance(association, TargetAssociation):
        return ['TARGET_ASSOCIATION_INVALID']

    texts = (association.judged_by, association.method, association.evidence_ref)

    if not all(isinstance(text, str) and text.strip() for text in texts):
        return ['TARGET_ASSOCIATION_INCOMPLETE']

    retained = texts + (association.observation_id, association.robot_id, association.object_id,
                        association.observation_frame_id, association.result)

    if not all(short_text(text, max_text_len) for text in retained):
        return ['TARGET_ASSOCIATION_TEXT_LIMIT']

    if not is_integer(association.observation_stamp_ns):
        return ['TARGET_ASSOCIATION_INVALID']

    if association.result not in ASSOCIATION_RESULTS:
        return ['TARGET_ASSOCIATION_INVALID']

    problems = []

    if association.result == 'NO_MATCH':
        problems.append('TARGET_ASSOCIATION_NOT_MATCH')
    elif association.result != ASSOCIATION_MATCH:
        problems.append('TARGET_ASSOCIATION_UNCONFIRMED')

    checks = (
        ('OBSERVATION', association.observation_id == observation.observation_id),
        ('ROBOT', association.robot_id == observation.robot_id),
        ('OBJECT', object_id is not None and association.object_id == object_id),
        ('STAMP', association.observation_stamp_ns == observation.stamp_ns),
        ('FRAME', association.observation_frame_id == observation.observed_frame_id),
    )
    problems += [f'TARGET_ASSOCIATION_{name}_MISMATCH' for name, same in checks if not same]

    # 비정확 시각 관측에 대한 판정은 정확 결합이 아니다.
    if not observation.stamp_exact:
        problems.append('TARGET_ASSOCIATION_STAMP_NOT_EXACT')

    return problems


# C2 관측과 외부 메타정보로 C3 Surface를 만든다. 근거가 부족하면 surface None과 사유 목록.
# max_text_len: 보존할 외부 문자열 길이 상한(문자 수, 호출자 명시. 기본값 없음)
def to_surface(observation, inputs, max_text_len):
    if not is_integer(max_text_len) or max_text_len < 1:
        raise ValueError('ADAPTER_SETTINGS_INVALID: max_text_len')

    reasons = []
    provenance = {}

    if not observation.valid:
        return AdapterResult(None, ('OBSERVATION_REJECTED',) + tuple(observation.reasons), {})

    # 평면 적합 범위·frame 계보가 맞는 관측만 다룬다.
    if observation.validity_scope != VALIDITY_SCOPE:
        reasons.append('VALIDITY_SCOPE_UNKNOWN')

    lineage = lineage_problem(observation)

    if lineage:
        reasons.append(lineage)

    # 비정확 시각(legacy float) 관측은 대응 판정이 있어도 넘기지 않는다.
    if not observation.stamp_exact:
        reasons.append('STAMP_NOT_EXACT')

    # 각 외부 값: 미정이면 *_UNDEFINED, 형식이 틀리면 *_INVALID
    def take(name, check):
        item = getattr(inputs, name)

        if item is None:
            reasons.append(f'{name.upper()}_UNDEFINED')
            return None

        if not sourced(item):
            reasons.append(f'{name.upper()}_SOURCE_MISSING')
            return None

        if not short_text(item.source, max_text_len) or (isinstance(item.value, str)
                                                         and not short_text(item.value, max_text_len)):
            reasons.append(f'{name.upper()}_TEXT_LIMIT')
            return None

        if not check(item.value):
            reasons.append(f'{name.upper()}_INVALID')
            return None

        provenance[name] = item.source

        return item.value

    object_id = take('object_id', lambda value: isinstance(value, str) and bool(value.strip()))

    # 관측 ID를 등록 물체 ID로 쓰지 않는다.
    if object_id is not None and (object_id == observation.observation_id or object_id.startswith('obs-')):
        reasons.append('OBJECT_ID_IS_OBSERVATION_ID')
        object_id = None

    scene_revision = take('scene_revision', lambda value: isinstance(value, int) and not isinstance(value, bool)
                          and value > 0)
    mass_kg = take('mass_kg', lambda value: finite(value) and value >= 0)
    confidence = take('confidence', lambda value: finite(value) and 0 <= value <= 1)
    suction = take('suction_suitable', lambda value: isinstance(value, bool))
    definition = take('flatness_definition', lambda value: value in FLATNESS_DEFINITIONS)
    outward = take('normal_outward', lambda value: isinstance(value, bool))
    frames = take('accepted_frames', lambda value: isinstance(value, (list, tuple))
                  and all(isinstance(item, str) and item and len(item) <= max_text_len for item in value))

    if outward is False:
        reasons.append('NORMAL_OUTWARD_NOT_CONFIRMED')

    if frames is not None and observation.frame_id not in frames:
        reasons.append('FRAME_NOT_ACCEPTED')

    if observation.axis_ambiguous:
        allow = take('allow_axis_ambiguous', lambda value: isinstance(value, bool))

        if allow is False:
            reasons.append('AXIS_AMBIGUOUS')

    # 목표 대응: 외부 판정이 현재 관측·대상과 결합돼 있어야 한다(판정 정확성은 검증하지 않음).
    association = inputs.target_association
    problems = association_problems(association, observation, object_id, max_text_len)
    reasons += problems

    if not problems:
        provenance['target_association'] = (f'외부 판정 {association.judged_by}/{association.method} '
                                            f'근거 {association.evidence_ref}(유효성·정확성 미검증)')

    flatness = None

    if definition is not None:
        flatness = observation.quality.get(definition)

        if not finite(flatness) or flatness < 0:
            reasons.append('FLATNESS_VALUE_MISSING')
            flatness = None
        else:
            provenance['flatness_error_m'] = f'C2 quality.{definition} (m), 정의 출처: {provenance["flatness_definition"]}'

    if reasons:
        return AdapterResult(None, tuple(reasons), provenance)

    surface = Surface(
        robot_id=observation.robot_id, object_id=object_id, scene_revision=scene_revision,
        stamp_s=observation.stamp_s, frame_id=observation.frame_id, center=observation.center,
        normal=observation.normal, tangent=observation.tangent,
        width_m=observation.width_m, depth_m=observation.depth_m,
        confidence=confidence, suction_suitable=suction, flatness_error_m=flatness, mass_kg=mass_kg,
    )
    provenance['geometry'] = (f'C2 {observation.observation_id} '
                              f'({observation.normal_convention}, roi {observation.roi_origin})')
    # C3 Surface.stamp_s는 float 파생값이다. 권위 시각은 여기 남긴다.
    provenance['stamp_ns'] = observation.stamp_ns

    return AdapterResult(surface, (), provenance)
