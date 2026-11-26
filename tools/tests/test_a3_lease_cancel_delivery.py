"""A3 철회 발행·취소 실패 재시도·작업별 격리를 실제 전달 코드로 검증합니다."""

from concurrent.futures import Future
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch

from action_msgs.srv import CancelGoal
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.lease_topics import LeaseTopics  # noqa: E402


class Clock:
    """실제 대기 없이 재시도 시각과 응답 제한 시간을 진행합니다."""

    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


def event(number=1):
    return dict(execution_id=f'exec-{number}', goal_id=f'{number:032x}',
                lease_generation=number, reason_code='LEASE_REVOKED', cancel=True)


def reply(code=CancelGoal.Response.ERROR_NONE):
    return CancelGoal.Response(return_code=code)


@pytest.fixture
def bridge():
    # 전송만 대체한다. 큐·상태 전이·콜백은 제품 LeaseTopics 구현을 그대로 실행한다.
    clock = Clock()
    node = Mock()
    with patch('ax_task_executor.lease_topics.LeaseProtocol'):
        value = LeaseTopics(node, SimpleNamespace(robot_id='robot_1'), None, None, clock)
    value.cancel_client.service_is_ready.return_value = True
    return value, clock


def enqueue(value, *events):
    with value.event_lock:
        value._enqueue_locked(events)


def test_unavailable_cancel_does_not_block_later_revoke(bridge):
    value, _ = bridge
    value.cancel_client.service_is_ready.return_value = False
    enqueue(value, event(1), event(2))
    value.flush()
    value.flush()
    messages = [c.args[0] for c in value.revokes.publish.call_args_list]
    assert [m.execution_id for m in messages] == ['exec-1', 'exec-2']
    assert len(value.pending) == 2
    value.cancel_client.call_async.assert_not_called()
    # 서비스 복구 후에는 두 Goal을 각각 취소하며 전체 취소 UUID를 보내지 않는다.
    value.cancel_client.service_is_ready.return_value = True
    value.cancel_client.call_async.side_effect = [Future(), Future()]
    value.flush()
    ids = [bytes(c.args[0].goal_info.goal_id.uuid).hex()
           for c in value.cancel_client.call_async.call_args_list]
    assert ids == [event(1)['goal_id'], event(2)['goal_id']]


def test_async_exception_is_retained_and_retried(bridge):
    value, clock = bridge
    first, second = Future(), Future()
    value.cancel_client.call_async.side_effect = [first, second]
    enqueue(value, event())
    value.flush()
    first.set_exception(RuntimeError('DDS failure'))
    assert len(value.pending) == 1
    value.flush()
    assert value.cancel_client.call_async.call_count == 1
    clock.now += 0.5
    value.flush()
    assert value.cancel_client.call_async.call_count == 2
    second.set_result(reply())
    assert not value.pending


def test_rejected_cancel_is_retried(bridge):
    value, clock = bridge
    first, second = Future(), Future()
    value.cancel_client.call_async.side_effect = [first, second]
    enqueue(value, event())
    value.flush()
    first.set_result(reply(CancelGoal.Response.ERROR_REJECTED))
    assert len(value.pending) == 1
    clock.now += 0.5
    value.flush()
    second.set_result(reply(CancelGoal.Response.ERROR_GOAL_TERMINATED))
    assert not value.pending


def test_call_exception_does_not_discard_delivery(bridge):
    value, clock = bridge
    ready = Future()
    value.cancel_client.call_async.side_effect = [RuntimeError('publish failed'), ready]
    enqueue(value, event())
    value.flush()
    assert len(value.pending) == 1
    clock.now += 0.5
    value.flush()
    ready.set_result(reply(CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID))
    assert not value.pending


def test_timeout_retries_and_late_response_cannot_remove_new_attempt(bridge):
    value, clock = bridge
    first, second = Future(), Future()
    first.set_running_or_notify_cancel()  # 원격 요청은 취소되지 않고 뒤늦게 응답할 수도 있다.
    value.cancel_client.call_async.side_effect = [first, second]
    enqueue(value, event())
    value.flush()
    clock.now += 1.1
    value.flush()
    assert value.cancel_client.call_async.call_count == 1
    clock.now += 0.5
    value.flush()
    first.set_result(reply())
    assert len(value.pending) == 1
    second.set_result(reply())
    assert not value.pending


def test_one_revoke_publish_failure_does_not_block_others(bridge):
    value, _ = bridge
    value.revokes.publish.side_effect = [RuntimeError('DDS publish failed'), None, None]
    value.cancel_client.service_is_ready.return_value = False
    enqueue(value, event(1), event(2))
    value.flush()
    value.flush()
    messages = [c.args[0].execution_id for c in value.revokes.publish.call_args_list]
    assert messages == ['exec-1', 'exec-2', 'exec-1']


def test_cancel_success_does_not_drop_unpublished_revoke(bridge):
    value, _ = bridge
    done = Future()
    done.set_result(reply())
    value.revokes.publish.side_effect = [RuntimeError('publish failed'), None]
    value.cancel_client.call_async.return_value = done
    enqueue(value, event())
    value.flush()
    assert len(value.pending) == 1
    value.flush()
    assert not value.pending
    assert value.cancel_client.call_async.call_count == 1


def test_concurrent_flush_and_duplicate_events_send_cancel_once(bridge):
    value, _ = bridge
    value.cancel_client.call_async.return_value = Future()
    enqueue(value, event(), event())
    workers = [threading.Thread(target=value.flush) for _ in range(8)]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join(2)
        assert not worker.is_alive()
    assert value.revokes.publish.call_count == 1
    assert value.cancel_client.call_async.call_count == 1


def test_retry_delay_is_capped_and_not_busy_looped(bridge):
    value, clock = bridge
    value.cancel_client.call_async.side_effect = RuntimeError('still unavailable')
    enqueue(value, event())
    for _ in range(10):
        value.flush()
        delivery = next(iter(value.pending.values()))
        assert 0.5 <= delivery.retry_at - clock.now <= 5.0
        previous_calls = value.cancel_client.call_async.call_count
        value.flush()
        assert value.cancel_client.call_async.call_count == previous_calls
        clock.now = delivery.retry_at


def test_close_prevents_retry_and_ignores_late_callback(bridge):
    value, _ = bridge
    first = Future()
    first.set_running_or_notify_cancel()
    value.cancel_client.call_async.return_value = first
    enqueue(value, event())
    value.flush()
    value.close()
    first.set_exception(RuntimeError('late failure'))
    value.flush()
    assert not value.pending
    assert value.cancel_client.call_async.call_count == 1


def test_no_cancel_event_is_completed_after_revoke(bridge):
    value, _ = bridge
    notice = event()
    notice['cancel'] = False
    enqueue(value, notice)
    value.flush()
    assert not value.pending
    value.cancel_client.call_async.assert_not_called()
