# Agent → D. 제안 1건(설계 7.6절 필드표). 조건부 필드는 해당 action에서만 채운다.
uint8 UNSPECIFIED=0            # 채우지 않은 제안은 형식 오류(FALLBACK)
uint8 NO_ACTION=1
uint8 ASSIGN_TASK=2
uint8 DEFER_TASK=3
uint8 RECOMMEND_RETRY=4
uint8 RECOMMEND_REASSIGN=5
uint8 RECOMMEND_PRIORITY_CHANGE=6
uint8 REQUEST_HOLD=7
uint8 REQUEST_RESUME_REVIEW=8
uint8 PREFER_CANDIDATE=9
uint8 MOVE_TO_LOCATION=10
uint8 WAIT_AT_LOCATION=11
# 전체 계획은 제안이 아니라 AgentPlan Topic으로 제출한다(D-17)
string sender_agent_id
string agent_session_id
uint64 agent_receive_generation
string proposal_id
uint32 schema_version
uint8 action
string team_id
string target_robot_id
string task_id
string assignment_id
uint64 assignment_generation
uint64 snapshot_id
uint64 task_revision
string reason_code
string[] evidence_state_ids
string object_id
string precondition_id
string failure_event_id
string hold_id
string transfer_id
string corridor_id
string grant_id
uint64 corridor_revision
CandidatePreference candidate
string plan_id                  # 이 제안의 근거가 된 계획(선택). 계획 제출이 아니다
uint64 plan_revision
string location_id
uint64 location_revision
uint64 map_revision
uint8 priority
string explanation              # 기록용. 검증 근거로 쓰지 않는다
