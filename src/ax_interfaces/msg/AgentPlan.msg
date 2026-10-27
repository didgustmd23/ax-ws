# 중앙 Agent → D. 전체 계획 제출. 발신 필드는 Agent 어댑터가 채우며 LLM이 만들지 않는다.
string sender_agent_id          # 발신 Agent ID. 어댑터가 채운다(예: agent_central)
string agent_session_id         # Agent 프로세스 세션 ID. 재시작마다 새로 발급
uint64 agent_receive_generation # 어댑터가 알고 있는 수신 세대. D가 현재 세대와 대조
string plan_id                  # 계획 ID. 같은 계획 재전송은 같은 ID
uint64 plan_revision            # 같은 계획의 수정 순번
uint64 snapshot_id              # 판단 근거로 쓴 snapshot ID
uint32 schema_version           # 메시지 계약 버전(D-14)
string team_id                  # 대상 협업 그룹 ID
string[] task_ids               # 계획 대상 그룹 작업 ID 목록(교착·재배정처럼 여러 작업에 걸칠 수 있음)
PlanStep[] steps                # 계획 단계 목록(의존 관계 포함)
string explanation              # 기록용. 검증 근거로 쓰지 않는다
