"""C2 어댑터 결과를 C3 후보 계산에 연결하며 미확인 근거를 기본값으로 채우지 않습니다."""
from dataclasses import dataclass
import math

from ax_grasp_planner.planner import Constraints, plan_candidates, Surface


@dataclass(frozen=True)
class C2HandoffResult:
    """후보와 원본 시각·출처를 함께 돌려줍니다. reasons는 내부 진단이며 공개 계약 코드가 아닙니다."""

    plan: object
    reasons: tuple
    stamp_ns: int
    provenance: dict

    @property
    def ready(self):
        """대응 근거와 도달·충돌 검사를 통과한 후보만 게시 대상으로 봅니다."""
        return self.plan is not None and self.plan.valid and not self.reasons


def prepare_c2_candidates(adapted, constraints, now_ns, *, robot_id,
                          reachability=None, collision=None):
    """C2 소유 어댑터가 반환한 결과만 받아 등록 물체·관측 시각을 보존합니다."""
    if type(now_ns) is not int or now_ns <= 0:
        raise ValueError('positive integer ROS now_ns required')
    if not isinstance(constraints, Constraints):
        raise ValueError('C3 Constraints required')
    provenance = dict(adapted.provenance)
    stamp_ns = provenance.get('stamp_ns', 0)

    def pending(*reasons):
        # C2의 보류 사유는 그대로 전달한다. 가짜 Surface·물체 ID·MATCH를 만들지 않는다.
        return C2HandoffResult(None, tuple(reasons), stamp_ns, provenance)

    if adapted.reasons:
        return pending(*adapted.reasons)
    surface = adapted.surface
    if not isinstance(surface, Surface):
        return pending('C2_SURFACE_MISSING')
    if surface.robot_id != robot_id:
        return pending('C2_ROBOT_MISMATCH')
    if type(stamp_ns) is not int or not 0 < stamp_ns <= now_ns:
        return pending('C2_STAMP_INVALID')
    # epoch 크기의 float 초에서는 1ns 차이가 사라진다. 신선도 검사는 변환 전에 정수로 한다.
    ttl = constraints.observation_ttl_s
    if isinstance(ttl, bool) or not isinstance(ttl, (int, float)) or ttl <= 0:
        return pending('CONSTRAINTS_INVALID')
    try:
        finite_ttl = math.isfinite(ttl) and math.isfinite(ttl * 1_000_000_000)
    except OverflowError:
        # Python 정수는 무제한이다. 잘못된 설정의 float 변환 실패도 실행 예외 대신 보류한다.
        finite_ttl = False
    if not finite_ttl:
        return pending('CONSTRAINTS_INVALID')
    if now_ns - stamp_ns > int(ttl * 1_000_000_000):
        return pending('OBSERVATION_STALE_OR_INVALID')
    if surface.stamp_s != stamp_ns / 1_000_000_000:
        return pending('C2_STAMP_MISMATCH')
    # 기존 Surface/Plan 공개 경계는 float 초다. 정확한 원본 ns와 출처는 별도 결과에 남긴다.
    plan = plan_candidates(surface, constraints, now_ns / 1_000_000_000,
                           reachability=reachability, collision=collision)
    return C2HandoffResult(plan, () if plan.valid else (plan.reason,), stamp_ns, provenance)
