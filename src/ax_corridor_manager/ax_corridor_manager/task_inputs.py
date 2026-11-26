"""D3가 읽을 작업 목록 경로만 선택합니다. 발행·배정 승인·실행은 하지 않습니다."""
import re


OPERATIONAL_TASK_TOPIC = '/ax/fleet/tasks'


def checked_task_topic(topic):
    """기존 운영 경로 또는 D2가 허용하는 명시적 개발 경로만 받습니다."""
    if topic == OPERATIONAL_TASK_TOPIC:
        return topic
    # 개발 자료를 운영 토픽으로 remap하지 않고 소비자 D3의 입력만 바꾼다.
    if (isinstance(topic, str) and re.fullmatch(r'/ax/dev/[a-z0-9_]+(?:/[a-z0-9_]+)*', topic)
            and '/ax/fleet' not in topic and '/ax/robot_' not in topic):
        return topic
    raise ValueError('fleet_tasks_topic must be /ax/fleet/tasks or an explicit /ax/dev/ topic')
