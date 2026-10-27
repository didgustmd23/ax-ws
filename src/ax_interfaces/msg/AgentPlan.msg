# 중앙 Agent → D. 전체 계획 제출. 발신 필드는 Agent 어댑터가 채우며 LLM이 만들지 않는다.
string sender_agent_id
string agent_session_id
uint64 agent_receive_generation
string plan_id
uint64 plan_revision
uint64 snapshot_id
uint32 schema_version
string team_id
string task_id
PlanStep[] steps
string explanation              # 기록용. 검증 근거로 쓰지 않는다
