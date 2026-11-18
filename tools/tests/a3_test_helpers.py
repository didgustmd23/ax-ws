"""A3 시험에서 반복되는 격리 환경과 모의 작업 요청을 준비합니다."""
import os
import tempfile
from unittest.mock import patch

from ax_interfaces.action import ExecuteLocalTask


def isolated_ros_home(test_case):
    """사용자 원장을 건드리지 않도록 임시 ROS_HOME과 정리 순서를 등록합니다."""
    folder = tempfile.TemporaryDirectory()
    test_case.addCleanup(folder.cleanup)
    patcher = patch.dict(os.environ, {'ROS_HOME': folder.name + '/ros_home'})
    patcher.start()
    # unittest 정리는 역순이다. 환경을 먼저 복원한 뒤 임시 폴더를 지운다.
    test_case.addCleanup(patcher.stop)
    return folder


def confirm_load_goal(execution_id):
    """기존 시험과 동일한 CONFIRM_LOAD 요청을 만듭니다."""
    request = ExecuteLocalTask.Goal()
    request.execution.execution_id = execution_id
    request.execution.operation = 'CONFIRM_LOAD'
    request.task_kind = ExecuteLocalTask.Goal.CONFIRM_LOAD
    request.object_id = 'test_box'
    return request
