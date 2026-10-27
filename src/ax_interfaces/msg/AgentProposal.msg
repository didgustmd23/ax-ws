# Agent → D. 제안 1건(설계 7.6절 필드표). 조건부 필드는 해당 action에서만 채운다.
uint8 UNSPECIFIED=0             # 채우지 않은 제안은 형식 오류(FALLBACK)
uint8 NO_ACTION=1               # 근거 있는 무조치 의견
uint8 ASSIGN_TASK=2             # 작업을 특정 로봇에 배정 제안
uint8 DEFER_TASK=3              # 시작 전 대기 작업의 시작 연기(잠금을 만들지 않음)
uint8 RECOMMEND_RETRY=4         # 재시도 권고(D 예산 안에서만)
uint8 RECOMMEND_REASSIGN=5      # 다른 로봇으로 재배정 권고
uint8 RECOMMEND_PRIORITY_CHANGE=6 # 작업 우선순위 변경 권고
uint8 REQUEST_HOLD=7            # 다음 단계 시작 차단 요청(진행 중 동작 취소 아님)
uint8 REQUEST_RESUME_REVIEW=8   # 기존 hold_id 조건 재검토 요청(해제·재개 명령 아님)
uint8 PREFER_CANDIDATE=9        # 집기 후보 선호 의견(로봇 Agent)
uint8 MOVE_TO_LOCATION=10       # 등록 지점 이동 제안
uint8 WAIT_AT_LOCATION=11       # 등록 지점 대기 제안
# 전체 계획은 제안이 아니라 AgentPlan Topic으로 제출한다(D-17)
string sender_agent_id          # 발신 Agent ID. 어댑터가 채운다(예: agent_central)
string agent_session_id         # Agent 프로세스 세션 ID. 재시작마다 새로 발급
uint64 agent_receive_generation # 어댑터가 알고 있는 수신 세대. D가 현재 세대와 대조
string proposal_id              # 제안 ID. 재전송은 같은 ID로 멱등 처리
uint32 schema_version           # 메시지 계약 버전(D-14)
uint8 action                    # 위 상수 중 제안 종류
string team_id                  # 대상 협업 그룹 ID
string target_robot_id          # 제안 대상 로봇 ID
string task_id                  # 대상 그룹 작업 ID
string assignment_id            # 제안 근거가 된 현재 배정 ID
uint64 assignment_generation    # 제안 근거가 된 배정 세대. 다르면 거절
uint64 snapshot_id              # 판단 근거로 쓴 snapshot ID
uint64 schedule_revision         # 근거 일정 revision. ASSIGN_TASK·DEFER_TASK·RECOMMEND_PRIORITY_CHANGE에서 D가 대조
uint64 task_revision            # 제안 근거가 된 작업 revision. 다르면 D가 거절
string reason_code              # 제안 사유 코드
string[] evidence_state_ids     # 근거 상태 ID 목록. 형식: state:<source_id>:<source_session_id>:<revision> 또는 원장 ID(hold-…, fail-…, rcpt-…). snapshot에 없는 ID면 거절
string object_id                # 대상 물체 ID(해당 제안일 때)
string precondition_id          # 대상 계획 조건 ID(보류·재개 제안일 때)
string failure_event_id         # 관련 실패 사건 ID(재시도·재배정 제안일 때)
string hold_id                  # 대상 보류 ID(재개 검토일 때)
string transfer_id              # 관련 인계 ID
string corridor_id              # 관련 통로 ID
string grant_id                 # 관련 통로 허가 ID
uint64 corridor_revision        # 근거로 쓴 통로 상태 revision
CandidatePreference candidate   # 후보 선호 내용(PREFER_CANDIDATE일 때)
string plan_id                  # 이 제안의 근거가 된 계획(선택). 계획 제출이 아니다
uint64 plan_revision            # 근거 계획의 revision
string location_id              # 이동·대기 대상 등록 지점 ID
uint64 location_revision        # 근거로 쓴 등록 지점 revision
uint64 map_revision             # 근거로 쓴 지도 revision
uint8 priority                  # 제안 우선순위 1~9, 클수록 먼저(ASSIGN_TASK·RECOMMEND_PRIORITY_CHANGE). 해당 없는 제안은 0
string explanation              # 기록용. 검증 근거로 쓰지 않는다
