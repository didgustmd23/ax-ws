"""A3가 실제 관절 피드백으로 후퇴 자세와 연속 정지를 함께 확인한다."""

from dataclasses import dataclass
import math
import threading


@dataclass(frozen=True)
class RetractionEvidence:
    """내부 관측 결과이며 Action 성공이나 주행 허가를 뜻하지 않는다."""

    # Q11: reason은 A3 내부 진단 이름이다. ResultStatus·validity_reason·공유 응답에
    # 그대로 옮기기 전 외부 의미·소비자 동작을 정하고 D-13 제안/검토를 거쳐야 한다.
    state: str
    reason: str


class ArmRetractionObserver:
    """승인된 후퇴 자세가 없거나 피드백이 불명확하면 완료를 확정하지 않는다."""

    def __init__(self, names=(), positions=(), approved=False, tolerance=0.02):
        # 목표 관절값을 임의의 0으로 채우지 않는다. 실제 모델 담당자의 승인값이 필요하다.
        self.names = tuple(names)
        self.positions = tuple(positions)
        self.approved = approved
        self.tolerance = tolerance
        if (len(self.names) != len(self.positions) or len(set(self.names)) != len(self.names)
                or any(not name for name in self.names)
                or any(not math.isfinite(x) for x in self.positions)
                or not math.isfinite(tolerance) or tolerance <= 0):
            raise ValueError('invalid retraction configuration')
        if approved and not self.names:
            raise ValueError('approved retraction needs joint targets')
        self.lock = threading.Lock()
        self.since = None
        self.last_stamp = None
        self.received = None
        self.evidence = RetractionEvidence('UNKNOWN', 'NO_JOINT_FEEDBACK')

    def update(self, message, ros_now, steady_now):
        """표본의 시각·필수 관절·자세·실측 속도를 검증한 뒤 연속 구간을 갱신한다."""
        with self.lock:
            return self._update(message, ros_now, steady_now)

    def _update(self, message, ros_now, steady_now):
        # ROS 시각은 센서 stamp 대조, 단조 시각은 수신 중단 감시에만 사용한다.
        if not self.approved:
            return self._reset('POSE_NOT_APPROVED')
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if (not all(math.isfinite(x) for x in (stamp, ros_now, steady_now))
                or stamp <= 0 or stamp > ros_now or ros_now - stamp > 0.3):
            return self._reset('JOINT_STAMP_INVALID')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return self._reset('JOINT_STAMP_NOT_INCREASING')
        names = tuple(message.name)
        if (len(set(names)) != len(names) or len(message.position) != len(names)
                or len(message.velocity) != len(names)
                or not set(self.names).issubset(names)):
            return self._reset('JOINT_FIELDS_INVALID')
        values = [(message.position[names.index(n)], message.velocity[names.index(n)])
                  for n in self.names]
        if any(not math.isfinite(x) for pair in values for x in pair):
            return self._reset('JOINT_VALUE_INVALID')
        # 표본 누락 구간까지 정지한 것으로 계산하지 않는다.
        if (self.last_stamp is None or stamp - self.last_stamp > 0.3
                or self.received is None or steady_now - self.received > 0.3):
            self.since = None
        self.last_stamp, self.received = stamp, steady_now
        if any(abs(p - target) > self.tolerance
               for (p, _), target in zip(values, self.positions)):
            self.since = None
            self.evidence = RetractionEvidence('NOT_RETRACTED', 'POSE_MISMATCH')
        elif any(abs(v) > 0.01 for _, v in values):
            self.since = None
            self.evidence = RetractionEvidence('NOT_RETRACTED', 'ARM_MOVING')
        else:
            # D-06: 자세 일치와 팔 속도 ≤ 0.01 rad/s가 함께 0.5초 이어져야 한다.
            if self.since is None:
                self.since = stamp
            complete = stamp - self.since >= 0.5
            self.evidence = RetractionEvidence(
                'RETRACTED' if complete else 'UNKNOWN',
                'CONFIRMED' if complete else 'STOP_WINDOW_INCOMPLETE')
        return self.evidence

    def snapshot(self, ros_now, steady_now):
        """마지막 성공 표본도 만료되거나 시계가 되돌아가면 완료 근거로 쓰지 않는다."""
        with self.lock:
            if not self.approved:
                return self._reset('POSE_NOT_APPROVED')
            if (self.received is None or self.last_stamp is None
                    or not math.isfinite(ros_now) or not math.isfinite(steady_now)
                    or not 0 <= steady_now - self.received <= 0.3
                    or not 0 <= ros_now - self.last_stamp <= 0.3):
                return self._reset('JOINT_FEEDBACK_STALE')
            return self.evidence

    def _reset(self, reason):
        """잘못된 표본은 이전의 완료 판정과 연속 정지 구간을 모두 무효화한다."""
        self.since = self.last_stamp = self.received = None
        self.evidence = RetractionEvidence('UNKNOWN', reason)
        return self.evidence
