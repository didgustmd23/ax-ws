"""초기 모의 대기 상태를 D4 원본에 전달해 인계 오류를 진단합니다. ROS 노드는 시작하지 않습니다."""
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
for relative in ('src/ax_task_executor', 'src/ax_d4/ax_d4'):
    sys.path.insert(0, str(ROOT / relative))
sys.path.insert(0, str(ROOT.parent / '.ax-ws-backups/d4-review-deps'))

from ax_interfaces.msg import RobotTaskStatus
from ax_task_executor.mock_executor import MockExecutor
from common_contract import check_status
from input_source import to_d4_status
from rclpy.time import Time

# MockExecutor의 실제 시작 상태와 동일한, 아직 ExecutionRef가 없는 모의 상태입니다.
status = RobotTaskStatus(robot_id='robot_1')
status.meta.schema_version = 1
status.meta.source_id = 'a3_mock_executor_robot_1'
status.meta.source_session_id = 'offline-idle-review'
status.meta.valid = False
status.meta.validity_reason = 'MOCK_ONLY'
source = SimpleNamespace(task_status=status, _status_lock=threading.Lock(),
                         task_status_publisher=Mock(),
                         get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))
source.publish_task_status = MethodType(MockExecutor.publish_task_status, source)
MockExecutor.update_task_status(source, 'MOCK_IDLE')
payload = to_d4_status(source.task_status_publisher.publish.call_args.args[0])
try:
    check_status(payload)
except (ValueError, KeyError, TypeError) as error:
    print(f'REPRODUCED: A3 MOCK_IDLE -> D4 validation failed: {error}')
else:
    print('A3 MOCK_IDLE -> D4 validation accepted')
