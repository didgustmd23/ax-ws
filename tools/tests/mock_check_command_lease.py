#!/usr/bin/env python3
"""성민님 A3의 미구현 CheckCommandLease 서버를 대신하는 W1 전용 Mock.

보안 경고: 실제 허가 시스템이 아니다! 실제 로봇에 연결하거나 승인 근거로 사용 금지.
기본은 무조건 거절. 연습용 허용은 환경변수 AX_W1_MOCK_ALLOW=1을 지정해야 함.
"""
import os
import rclpy
from rclpy.node import Node
from ax_interfaces.srv import CheckCommandLease


# 실제 A3 CheckCommandLease 서버가 준비되지 않은 W1 시험에서 사용하는 대체 노드이다.
# 허가 계약의 연결·거부 경로만 확인하며, 운영 권한을 대신 검증하는 서버는 아니다.
class MockLease(Node):
    def __init__(self):
        # 환경변수 기반 허용 정책과 로봇별 서비스 경로를 설정한다.
        super().__init__('mock_check_command_lease_w1')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.allow = os.getenv('AX_W1_MOCK_ALLOW', '0') == '1'
        self.server = self.create_service(CheckCommandLease,
            f'/ax/{self.robot_id}/executor/lease/check_command', self.on_check)
        self.get_logger().warn('TEST MOCK ONLY / 실제 A3 권한 서버가 아님 / allow=%s' % self.allow)

    def on_check(self, request, response):
        # 요청 구조와 A4 팔 command source를 먼저 확인한 뒤 시험용 정책을 적용한다.
        # 이 응답은 실제 A3 권한 발급·회수·영속화를 구현하지 않는다.
        structurally_valid = bool(request.request_id and request.robot_id == self.robot_id
                                  and request.execution_id and request.command_id
                                  and len(request.command_digest) == 64
                                  and request.phase in (CheckCommandLease.Request.ACCEPT,
                                                        CheckCommandLease.Request.CONTINUE)
                                  and request.command_source == CheckCommandLease.Request.SOURCE_A4_ARM)
        response.allowed = bool(self.allow and structurally_valid)
        response.reason_code = 'MOCK_ALLOWED_TEST_ONLY' if response.allowed else 'MOCK_DENIED_TEST_ONLY'
        response.accept_record_id = ('mock-accept-record' if response.allowed and
                                     request.phase == CheckCommandLease.Request.ACCEPT else '')
        response.valid_for_s = 2.0 if response.allowed else 0.0
        return response


def main():
    # A4의 비동기 lease 요청을 처리할 수 있도록 ROS executor를 실행한다.
    rclpy.init()
    node = MockLease()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
