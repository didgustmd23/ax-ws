"""원장 기반 lease 프로토콜을 공통 C4 토픽과 Action cancel 서비스에 연결합니다."""
from dataclasses import dataclass
import threading
import time

from action_msgs.srv import CancelGoal
from ax_interfaces.msg import ExecutionLeaseGrant, ExecutionLeaseRequest, ExecutionLeaseRevoke
from ax_task_executor.lease_protocol import LeaseProtocol
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy


@dataclass
class RevocationDelivery:
    """철회 토픽 전달과 취소 응답 대기를 서로 독립적으로 추적합니다."""

    event: dict
    published: bool = False
    canceled: bool = False
    future: object = None
    deadline: float = 0.0
    retry_at: float = 0.0
    attempts: int = 0


class LeaseTopics:
    """요청 콜백이 외부 Goal이나 배정을 자동 등록하지 않도록 권한 관리와 분리합니다."""

    def __init__(self, node, authority, ledger, callback_group, clock=time.monotonic):
        """C4 계약 경로와 EVENT depth 50을 사용합니다."""
        self.node, self.authority = node, authority
        self.protocol = LeaseProtocol(authority, ledger)
        self.pending = {}
        self.event_lock = threading.RLock()
        self.clock = clock
        self.closed = False
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        root = f'/ax/{authority.robot_id}/manipulation'
        self.grants = node.create_publisher(ExecutionLeaseGrant, root + '/lease/grant', qos)
        self.revokes = node.create_publisher(ExecutionLeaseRevoke, root + '/lease/revoke', qos)
        self.cancel_client = node.create_client(
            CancelGoal, root + '/execute/_action/cancel_goal', callback_group=callback_group)
        self.subscription = node.create_subscription(
            ExecutionLeaseRequest, root + '/lease/request', self.on_request, qos,
            callback_group=callback_group)
        self.timer = node.create_timer(0.1, self.tick, callback_group=callback_group,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_request(self, request):
        """기록 완료된 응답만 발행하며 예외가 ROS spin 밖으로 나가지 않게 합니다."""
        try:
            result, events = self.protocol.handle(request)
            with self.event_lock:
                self._enqueue_locked(events)
                self.flush()
                self.grants.publish(ExecutionLeaseGrant(**result))
        except Exception as error:
            self.authority.invalidate_assignment()
            self.node.get_logger().error(f'Lease topic request failed: {error!r}')

    def revoke_current(self, reason='LEASE_REVOKED'):
        """내부 취소 경로는 서비스 권한을 먼저 막은 뒤 revoke와 정확한 부모 Goal cancel을 보냅니다."""
        events = self.protocol.revoke(reason)
        with self.event_lock:
            self._enqueue_locked(events)
            self.flush()

    def finish_goal(self, execution_id, goal_id):
        """종료 대상 확인·철회·권한 제거는 직렬화하고 ROS 발행은 권한 잠금 밖에서 합니다."""
        with self.authority.lock:
            ctx = self.authority.context
            if ctx is None or (ctx.execution_id, ctx.goal_id) != (execution_id, goal_id):
                return False
            events = self.protocol.revoke()
            self.authority.finish_goal(execution_id, goal_id)
        with self.event_lock:
            self._enqueue_locked(events)
            self.flush()
        return True

    def tick(self):
        """배정 상실과 명시적 철회를 알리고 발행·cancel 준비 실패는 다음 주기에 재시도합니다."""
        try:
            self.authority.expire_assignment()
            with self.authority.lock:
                ctx = self.authority.context
                invalid = ctx is not None and ctx.revoked
            if invalid:
                reason = ('ASSIGNMENT_STALE' if self.authority.assignment is None
                          else 'LEASE_REVOKED')
                events = self.protocol.revoke(reason)
                with self.event_lock:
                    self._enqueue_locked(events)
            self.flush()
        except Exception as error:
            self.authority.invalidate_assignment()
            self.node.get_logger().error(f'Lease topic timer failed: {error!r}')

    def flush(self):
        """DB 잠금 밖에서 발행합니다. 재동기화는 같은 Goal을 유지하고 이전 세대만 차단합니다."""
        with self.event_lock:
            self._flush_locked()

    def _flush_locked(self):
        """모든 철회를 먼저 발행하고, 작업별 취소 대기는 다른 작업과 독립적으로 처리합니다."""
        if self.closed:
            return
        # 앞 Goal의 취소 서비스가 없어도 뒤 Goal의 revoke를 먼저 전달한다.
        for delivery in list(self.pending.values()):
            if not delivery.published:
                try:
                    message = {key: value for key, value in delivery.event.items()
                               if key != 'cancel'}
                    self.revokes.publish(ExecutionLeaseRevoke(**message))
                    delivery.published = True
                except Exception as error:
                    self.node.get_logger().error(f'Lease revoke publish failed: {error!r}')
        for key, delivery in list(self.pending.items()):
            if not delivery.event['cancel']:
                delivery.canceled = True
            if delivery.canceled:
                if delivery.published:
                    self.pending.pop(key, None)
                continue
            now = self.clock()
            if delivery.future is not None:
                if now < delivery.deadline:
                    continue
                # 응답 없는 Future도 영원히 붙잡지 않는다. 늦게 온 응답은 식별자로 걸러낸다.
                old_future, delivery.future = delivery.future, None
                old_future.cancel()
                self._retry_locked(delivery, 'response timeout')
            if now < delivery.retry_at:
                continue
            try:
                if not self.cancel_client.service_is_ready():
                    continue
                request = CancelGoal.Request()
                request.goal_info.goal_id.uuid = list(bytes.fromhex(delivery.event['goal_id']))
                # 부모 Goal UUID를 지정한다. 빈 UUID로 다른 실행 전체를 취소하지 않는다.
                delivery.attempts += 1
                future = self.cancel_client.call_async(request)
                delivery.future, delivery.deadline = future, now + 1.0
                future.add_done_callback(lambda done, token=key: self._cancel_done(token, done))
            except Exception as error:
                delivery.future = None
                self._retry_locked(delivery, repr(error))

    def _enqueue_locked(self, events):
        """같은 실행·부모 Goal·세대는 한 항목으로 보관하며 진행 중 취소를 중복 전송하지 않습니다."""
        if not self.closed:
            for event in events:
                key = (event['execution_id'], event['goal_id'], event['lease_generation'])
                self.pending.setdefault(key, RevocationDelivery(dict(event)))

    def _retry_locked(self, delivery, detail):
        """실패를 버리지 않고 0.5~5초 간격으로 재시도합니다. 권한은 계속 철회 상태입니다."""
        delay = min(5.0, 0.5 * 2 ** min(max(0, delivery.attempts - 1), 4))
        delivery.retry_at = self.clock() + delay
        self.node.get_logger().error(
            f'Lease Action cancel retry in {delay}s '
            f'({delivery.event["execution_id"]}): {detail}')

    def _cancel_done(self, key, future):
        """취소 응답 실패를 성공으로 숨기지 않습니다. 물리 정지는 별도 증거가 필요합니다."""
        with self.event_lock:
            delivery = self.pending.get(key)
            if self.closed or delivery is None or delivery.future is not future:
                return
            delivery.future = None
            try:
                response = future.result()
                safe = (CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
                        CancelGoal.Response.ERROR_GOAL_TERMINATED)
                if response.return_code not in safe:
                    self._retry_locked(delivery, f'rejected: {response.return_code}')
                    return
                # 취소 응답은 요청 처리 근거일 뿐 실제 정지·Action 종료 확인이 아니다.
                delivery.canceled = True
                if delivery.published:
                    self.pending.pop(key, None)
            except Exception as error:
                self._retry_locked(delivery, repr(error))

    def close(self):
        """종료 뒤 늦게 온 취소 응답이 재전송이나 노드 접근을 일으키지 않게 합니다."""
        with self.event_lock:
            self.closed = True
            self.timer.cancel()
            for delivery in self.pending.values():
                if delivery.future is not None:
                    delivery.future.cancel()
            self.pending.clear()
