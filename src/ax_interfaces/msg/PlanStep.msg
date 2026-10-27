# 중앙 계획의 단계 1개(설계 5.2절). 등록 지점·물품·역할 ID만 참조하고 좌표를 넣지 않는다.
string task_id                  # 이 단계가 속한 작업(AgentPlan.task_ids 중 하나). 실행 ID·예산은 이 작업에 귀속. 없거나 목록 밖이면 계획 거절
string step_key                 # 단계 식별자. execution_id 생성에 쓰므로 계획에 영속 저장
string[] depends_on             # 먼저 끝나야 하는 step_key 목록
string operation                # D-11 허용 operation 중 하나
string robot_role               # 수행 역할(MANIPULATOR, TRANSPORTER)
string robot_id                 # 비어 있으면 D가 배정
string object_id                # 대상 물체 ID
string location_id              # 대상 등록 지점 ID
PlanCondition[] preconditions   # 이 단계의 전제조건 목록
string on_failure               # 실패 시 이동할 step_key 또는 FAIL_TERMINATE(이 계획에 후속 분기가 없다는 표시). 실패 종료는 D가 본설계 9.5·9.6절로 판정하며 비어 있으면 계획 등록 거절
bool terminal                   # 계획의 마지막 단계인지
