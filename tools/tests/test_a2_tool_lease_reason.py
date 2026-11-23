"""A2 suction_tool_node의 A3 명령 확인(check_lease) 거절 사유 시험(계약 D-13, 2026-11-24 결정).

[검사 범위] check_lease 한 함수만 가짜 lease 클라이언트로 부른다. ROS 통신·실제 A3·A4·Gazebo는 쓰지 않는다.
- A3 무응답(서비스 미준비·0.5 s 무응답) → A3_LEASE_TIMEOUT
- A3 거절 → 사유 그대로, 빈 사유 → LEASE_EXPIRED
- 허용 뒤 기한(t_first + valid_for_s) 경과 → LEASE_EXPIRED, 기한 안 → 허용
실제 A3 서버와의 왕복은 test_a3_lease_check·test_a3_local_task_executor가 본다.
"""
import importlib.util
from pathlib import Path
import threading
import time
import unittest

from ax_interfaces.srv import CheckCommandLease, ToolCommand

SCRIPTS = Path(__file__).resolve().parents[2] / 'src/ax_control/scripts'


# suction_tool_node 스크립트를 모듈로 읽는다(실행 경로와 같은 코드를 쓰기 위해)
def load_tool_module():
    spec = importlib.util.spec_from_file_location('suction_tool_node_lease_reason', SCRIPTS / 'suction_tool_node.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


TOOL = load_tool_module()


# 가짜 future: 응답을 바로 주거나(응답 있음) 끝까지 주지 않는다(무응답)
class FakeFuture:
    def __init__(self, response):
        self.response = response
        self.cancelled = False

    # 응답이 있으면 콜백을 바로 부른다(실제 rclpy처럼 다른 스레드에서 불러도 결과는 같다)
    def add_done_callback(self, callback):
        if self.response is not None:
            threading.Thread(target=callback, args=(self,), daemon=True).start()

    def result(self):
        return self.response

    def cancel(self):
        self.cancelled = True


# 가짜 lease 클라이언트: 준비 여부와 돌려줄 응답(None이면 무응답)을 정한다. 받은 요청을 기록한다
class FakeLeaseClient:
    def __init__(self, ready=True, response=None):
        self.ready = ready
        self.response = response
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = FakeFuture(self.response)
        self.futures.append(future)

        return future


# check_lease가 쓰는 속성(lease_client)만 가진 노드 대역
class FakeNode:
    def __init__(self, client):
        self.lease_client = client


# A3 응답 하나를 만든다
def lease_reply(allowed, reason='', valid_for_s=2.0):
    reply = CheckCommandLease.Response()
    reply.allowed = allowed
    reply.reason_code = reason
    reply.valid_for_s = valid_for_s

    return reply


# check_tool_attach와 같은 형식의 공구 명령
def tool_request():
    request = ToolCommand.Request()
    request.request_id = 'lease-reason-1'
    request.robot_id = 'robot_1'
    request.execution_id = 'exec-1'
    request.goal_id = '0123456789abcdef0123456789abcdef'
    request.object_id = 'box_1'
    request.lease_generation = 1
    request.command = ToolCommand.Request.ATTACH

    return request


# 가짜 클라이언트로 check_lease를 한 번 부른다. first_sent 기본값은 지금
def check(client, first_sent=None):
    request = tool_request()
    sent = TOOL.boottime_seconds() if first_sent is None else first_sent

    return TOOL.SuctionToolNode.check_lease(FakeNode(client), request, TOOL.command_digest(request), sent)


class LeaseReasonTest(unittest.TestCase):

    # 상수: 무응답은 A3_LEASE_TIMEOUT, 기한 경과·빈 사유는 LEASE_EXPIRED(서로 다른 코드)
    def test_constants(self):
        self.assertEqual(TOOL.REASON_LEASE_NO_RESPONSE, 'A3_LEASE_TIMEOUT')
        self.assertEqual(TOOL.REASON_LEASE_EXPIRED, 'LEASE_EXPIRED')

    # 서비스 미준비: 요청을 보내지 않고 A3_LEASE_TIMEOUT
    def test_service_not_ready(self):
        client = FakeLeaseClient(ready=False, response=lease_reply(True))

        self.assertEqual(check(client), (False, 'A3_LEASE_TIMEOUT'))
        self.assertEqual(client.requests, [])

    # 0.5 s 안에 응답 없음: 요청을 취소하고 A3_LEASE_TIMEOUT. 대기 시간은 한도 근처
    def test_no_response_timeout(self):
        client = FakeLeaseClient(response=None)
        start = time.monotonic()

        self.assertEqual(check(client), (False, 'A3_LEASE_TIMEOUT'))
        elapsed = time.monotonic() - start
        self.assertGreaterEqual(elapsed, TOOL.LEASE_TIMEOUT_SECONDS - 0.05)
        self.assertLess(elapsed, TOOL.LEASE_TIMEOUT_SECONDS + 0.5)
        self.assertEqual(len(client.requests), 1)
        self.assertTrue(client.futures[0].cancelled)

    # A3 거절 사유는 그대로 전달한다(A3가 실제로 LEASE_EXPIRED를 준 경우도 그대로)
    def test_rejection_passes_reason(self):
        for reason in ('ASSIGNMENT_STALE', 'LEASE_EXPIRED', 'LEASE_REVOKED', 'GOAL_MISMATCH', 'DUPLICATE_REQUEST',
                       'AUTHORITY_INVALID', 'A3_RECORD_FAILED'):
            with self.subTest(reason=reason):
                self.assertEqual(check(FakeLeaseClient(response=lease_reply(False, reason))), (False, reason))

    # 빈 사유 거절은 LEASE_EXPIRED로 채운다(A4 보호 팔 서버와 같은 대체값)
    def test_rejection_without_reason(self):
        self.assertEqual(check(FakeLeaseClient(response=lease_reply(False, ''))), (False, 'LEASE_EXPIRED'))

    # 허용 뒤 기한 경과(최초 송신 + valid_for_s가 지남)는 LEASE_EXPIRED, 기한 안이면 허용
    def test_allowed_then_expired(self):
        expired = check(FakeLeaseClient(response=lease_reply(True, 'DEV_POLICY_ALLOW', 0.1)),
                        first_sent=TOOL.boottime_seconds() - 1.0)
        self.assertEqual(expired, (False, 'LEASE_EXPIRED'))

        self.assertEqual(check(FakeLeaseClient(response=lease_reply(True, 'DEV_POLICY_ALLOW', 2.0))), (True, ''))
        self.assertEqual(check(FakeLeaseClient(response=lease_reply(True, '', 2.0))), (True, ''))

    # 보내는 확인 요청 필드는 바뀌지 않았다(C 조작 ACCEPT, A2 공구 출처, command_id = request_id)
    def test_request_fields(self):
        client = FakeLeaseClient(response=lease_reply(True, '', 2.0))
        check(client)
        sent = client.requests[0]

        self.assertEqual(sent.authority, CheckCommandLease.Request.AUTH_C_MANIPULATION)
        self.assertEqual(sent.phase, CheckCommandLease.Request.ACCEPT)
        self.assertEqual(sent.command_source, CheckCommandLease.Request.SOURCE_A2_TOOL)
        self.assertEqual(sent.command_id, 'lease-reason-1')


if __name__ == '__main__':
    unittest.main()
