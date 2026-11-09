"""W1 학습용: 외부 구동 없이 CONFIRM_LOAD의 단계 전환만 실행한다.

실제 ROS 서버·센서·SQLite·취소 확인은 연결하지 않는 독립 연습 예제다.
기존 mock_executor.py를 대체하지 않는다.
"""
import argparse
from enum import Enum


class Phase(Enum):
    # 현재 작업이 어느 단계인지 사람이 읽을 수 있는 이름으로 정의한다.
    IDLE = '대기'
    VALIDATING = '요청 검사'
    CONFIRMING_LOAD = '모의 적재 확인'
    COMPLETED = '완료'
    FAILED = '실패'
    UNCERTAIN = '불확실'
    CANCELING = '취소 처리 중'
    CANCELED = '모의 취소 확인 완료'


class ConfirmLoadDemo:
    # 표에 있는 순서로만 이동할 수 있다. 종료 단계에는 다음 단계가 없다.
    ALLOWED = {
        Phase.IDLE: {Phase.VALIDATING},
        Phase.VALIDATING: {Phase.CONFIRMING_LOAD, Phase.FAILED},
        Phase.CONFIRMING_LOAD: {
            Phase.COMPLETED, Phase.FAILED, Phase.UNCERTAIN, Phase.CANCELING},
        Phase.CANCELING: {Phase.CANCELED},
        Phase.COMPLETED: set(),
        Phase.FAILED: set(),
        Phase.UNCERTAIN: set(),
        Phase.CANCELED: set(),
    }

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.phase = Phase.IDLE
        self.show('새 연습 작업을 만들었습니다.')

    def show(self, explanation):
        # 실제 구현에서는 이 시점의 상태를 RobotTaskStatus로도 발행한다.
        print(f'[{self.robot_id}] {self.phase.name}: {self.phase.value} — {explanation}')

    def move(self, next_phase, explanation):
        # 잘못된 호출 순서는 여기서 차단한다.
        if next_phase not in self.ALLOWED[self.phase]:
            raise ValueError(f'허용하지 않는 전환: {self.phase.name} → {next_phase.name}')
        self.phase = next_phase
        self.show(explanation)

    def run(self, execution_id, object_id, mock_load, cancel=False):
        self.move(Phase.VALIDATING, 'execution_id와 object_id가 있는지 검사합니다.')
        if not execution_id or not object_id:
            self.move(Phase.FAILED, '필수 입력이 없어 확인 단계로 넘어가지 않습니다.')
            return self.phase

        self.move(Phase.CONFIRMING_LOAD, '센서 대신 예제의 시험 입력을 받습니다.')
        if cancel:
            self.move(Phase.CANCELING, '취소 요청을 받아 결과 처리로 진행하지 않습니다.')
            # 이 예제는 외부 작업이 없으므로 모의 확인만 한다.
            # 실제 조작 중에는 C4 등의 취소·종료 확인 없이 완료로 바꾸면 안 된다.
            self.move(Phase.CANCELED, '외부 구동 없는 연습 작업의 취소를 확인했습니다.')
        elif mock_load == 'STOWED':
            self.move(Phase.COMPLETED, '모의 입력이 STOWED라 완료로 이동합니다.')
        elif mock_load == 'EMPTY':
            # 첫 과제의 시험 매핑을 따른다. 운영 EMPTY 의미는 별도 계약 확인 대상이다.
            self.move(Phase.FAILED, '첫 과제의 모의 규칙에 따라 EMPTY를 실패로 처리합니다.')
        else:
            self.move(Phase.UNCERTAIN, '상태를 모르므로 완료로 처리하지 않습니다.')
        return self.phase


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', choices=('robot_1', 'robot_2', 'both'), default='both')
    parser.add_argument('--scenario', choices=(
        'success', 'failure', 'unknown', 'cancel', 'invalid', 'all'), default='success')
    args = parser.parse_args()
    robots = ('robot_1', 'robot_2') if args.robot_id == 'both' else (args.robot_id,)
    scenarios = {
        'success': ('STOWED', False, 'test_box'),
        'failure': ('EMPTY', False, 'test_box'),
        'unknown': ('UNKNOWN', False, 'test_box'),
        'cancel': ('STOWED', True, 'test_box'),
        'invalid': ('STOWED', False, ''),
    }
    selected = scenarios if args.scenario == 'all' else {args.scenario: scenarios[args.scenario]}
    for robot in robots:
        for name, (load, cancel, object_id) in selected.items():
            print(f'\n=== {robot} / {name} (학습용 모의 실행) ===')
            task = ConfirmLoadDemo(robot)
            task.run(f'{robot}-{name}-001', object_id, load, cancel)


if __name__ == '__main__':
    main()
