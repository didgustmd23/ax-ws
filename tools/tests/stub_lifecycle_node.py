#!/usr/bin/env python3
# localization.launch.py 시작 감시 시험용 가짜 lifecycle 노드. 매개변수 behavior로 실패 방식을 고른다.
#   ok               configure·activate 성공
#   reject_configure configure에서 FAILURE(→ unconfigured)
#   hang_configure   configure 콜백에서 응답하지 않음(무응답)
#   fail_activate    activate에서 FAILURE(→ inactive)
#   exit_configure   configure 중 프로세스 종료(종료 코드 3)
#   exit_active      active가 된 뒤 exit_delay_s 지나 프로세스 종료(종료 코드 4)
import os
import time

import rclpy
from rclpy.lifecycle import Node, TransitionCallbackReturn


# 매개변수에 따라 전이 결과를 정하는 노드
class StubNode(Node):
    # 매개변수를 읽는다.
    def __init__(self):
        super().__init__("stub_lifecycle")
        self.behavior = self.declare_parameter("behavior", "ok").value
        self.exit_delay_s = self.declare_parameter("exit_delay_s", 1.0).value

    # configure 전이
    def on_configure(self, state):
        if self.behavior == "reject_configure":
            return TransitionCallbackReturn.FAILURE

        if self.behavior == "hang_configure":
            while True:
                time.sleep(1.0)

        if self.behavior == "exit_configure":
            os._exit(3)

        return TransitionCallbackReturn.SUCCESS

    # activate 전이
    def on_activate(self, state):
        if self.behavior == "fail_activate":
            return TransitionCallbackReturn.FAILURE

        if self.behavior == "exit_active":
            self.create_timer(self.exit_delay_s, lambda: os._exit(4))

        return super().on_activate(state)


# 노드를 실행한다. Ctrl+C로 끝난다.
def main():
    rclpy.init()
    node = StubNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
