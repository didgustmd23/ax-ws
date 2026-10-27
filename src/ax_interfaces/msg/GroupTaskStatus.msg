# D → 관제·Agent·A. 그룹 작업 상태(설계 8절).
uint8 UNSPECIFIED=0             # 채우지 않은 상태. 정상으로 해석하지 않는다
uint8 QUEUED=1                  # 대기 중
uint8 ASSIGNED=2                # 로봇 배정됨
uint8 RUNNING=3                 # 실행 중
uint8 COMPLETED=4               # 완료
uint8 FAILED=5                  # 실패 종료
uint8 CANCELED=6                # 취소
uint8 RETRY_PENDING=7           # 재시도 대기
string team_id                  # 협업 그룹 ID
string task_id                  # 그룹 작업 ID
uint64 task_revision            # D 원장의 작업 revision(재시작 후에도 이어짐). 물체·지점·계획 단계 정의·배정이 바뀔 때 증가. 우선순위·연기는 schedule_revision, 진행 상태 전이로는 증가하지 않는다
uint8 state                     # 위 상수 중 작업 상태
uint8 priority                  # D가 반영한 작업 우선순위 1~9, 클수록 먼저. 0은 미설정이며 D가 등록 때 기본값을 채운다
uint64 schedule_revision         # 우선순위·연기 반영마다 증가(D 원장 영속). 일정 관련 제안은 이 값으로 대조
bool deferred                   # D가 시작 연기를 반영했는지
string object_id                # 운반 물체 ID
string source_location_id       # 출발 등록 지점 ID
string destination_location_id  # 도착 등록 지점 ID
# 인계 상태(설계 9.3절). A4 최종 게이트의 입력이며 0(UNKNOWN)·만료면 두 로봇 모두 주행 금지
uint8 TRANSFER_UNKNOWN=0        # 인계 상태 불명. 두 로봇 주행 금지
uint8 TRANSFER_NONE=1           # 인계 없음 또는 시작 전
uint8 TRANSFER_LOCKED=2         # 두 로봇 실제 정지·잠금 확인. 두 로봇 주행 금지
uint8 TRANSFER_IN_PROGRESS=3    # 적재·하역 중. 두 로봇 주행 금지
uint8 TRANSFER_UNLOCKED=4       # 적재: 적재 확인·흡착 해제·팔 후퇴 / 하역: 배치 확인·흡착 해제·팔 후퇴·적재함 EMPTY 확인 후 인계 잠금 해제. 다른 구동 차단 사유는 따로 적용
string manipulator_robot_id     # 이 작업의 조작 역할 로봇
string transporter_robot_id     # 이 작업의 운송 역할 로봇
uint64 assignment_generation    # 현재 배정 세대
string transfer_id              # 인계 ID. 인계 없으면 빈 문자열
uint8 transfer_state            # 위 TRANSFER_ 상수 중 인계 상태
string[] hold_ids               # 이 작업에 걸린 보류 ID 목록
string failure_event_id         # 실패 종료 시 사건 ID
