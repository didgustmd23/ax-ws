"""초기 모의 대기 상태를 D4 원본에 전달해 인계 오류를 진단합니다. ROS 노드는 시작하지 않습니다."""
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
for relative in ('src/ax_task_executor', 'src/ax_d4'):
    sys.path.insert(0, str(ROOT / relative))
# 설치된 D4 의존성을 사용해 실제 실행과 같은 import 조건으로 확인합니다.

from ax_task_executor.mock_executor import MockExecutor
from ax_task_executor.status_contract import make_unassigned_mock_idle
# 파일 직접 실행용 import 대신 최신 D4 패키지 실행 경로에 맞춥니다.
from ax_d4.common_contract import check_status
from ax_d4.input_source import to_d4_status
from rclpy.time import Time

# 실제 초기 생성·게시 함수를 사용합니다. 기존 D4 원본의 검증 오류가 남아 있는지 확인합니다.
status = make_unassigned_mock_idle('robot_1', 'offline-idle-review')
source = SimpleNamespace(task_status=status, _status_lock=threading.Lock(),
                         task_status_publisher=Mock(),
                         get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))
source.publish_task_status = MethodType(MockExecutor.publish_task_status, source)
source.publish_task_status()
payload = to_d4_status(source.task_status_publisher.publish.call_args.args[0])
try:
    check_status(payload)
except (ValueError, KeyError, TypeError) as error:
    print(f'REPRODUCED: A3 MOCK_IDLE -> D4 validation failed: {error}')
else:
    print('A3 MOCK_IDLE -> D4 validation accepted')
