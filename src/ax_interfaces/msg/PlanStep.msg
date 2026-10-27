# 중앙 계획의 단계 1개(설계 5.2절). 등록 지점·물품·역할 ID만 참조하고 좌표를 넣지 않는다.
string step_key
string[] depends_on
string operation                # 허용 operation 목록 안에서만
string robot_role
string robot_id                 # 비어 있으면 D가 배정
string object_id
string location_id
PlanCondition[] preconditions
string on_failure               # 실패 분기 step_key 또는 FAIL_TERMINATE
bool terminal
