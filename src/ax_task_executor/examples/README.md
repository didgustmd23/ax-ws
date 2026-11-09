# W1 단계 머신 연습

`w1_confirm_load_demo.py`는 기존 ROS 실행기를 변경하지 않는 독립 학습 예제다.
실제 로봇·센서·Action·SQLite·RobotTaskStatus에는 연결하지 않는다.

프로젝트 루트의 PowerShell 터미널에서 실행한다. 별도 빌드는 필요 없다.

```powershell
# 로봇 1: 정상 흐름
wsl -d Ubuntu-24.04 -e python3 src/ax_task_executor/examples/w1_confirm_load_demo.py --robot-id robot_1 --scenario success

# 로봇 2: 모의 취소 흐름
wsl -d Ubuntu-24.04 -e python3 src/ax_task_executor/examples/w1_confirm_load_demo.py --robot-id robot_2 --scenario cancel

# 두 로봇의 성공·실패·불확실·취소·입력 오류 흐름 모두 출력
wsl -d Ubuntu-24.04 -e python3 src/ax_task_executor/examples/w1_confirm_load_demo.py --scenario all
```

읽는 순서는 `Phase`(단계 이름), `ALLOWED`(허용 전환), `move`(전환 검사),
`run`(입력·결과에 따른 흐름)이다. 출력의 COMPLETED 상태는 실제 적재 확인이 아닌 모의 결과다.
취소도 외부 실행이 없는 예제에서 즉시 확인하는 모의 동작이다.

2026-11-09 두 로봇의 5개 시나리오를 모두 실행해 출력 흐름을 확인했다.
이 예제만으로 운영 A3 단계 머신·ROS 취소·영속 복구 구현 완료를 의미하지 않는다.
