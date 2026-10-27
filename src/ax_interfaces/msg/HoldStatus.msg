# D → 관제·Agent. 보류 1건의 수명주기(설계 9.6절).
uint8 HOLD_PENDING=0
uint8 HELD=1
uint8 RECHECK=2
uint8 RELEASED=3
uint8 FAILED_ISOLATED=4
string hold_id
uint8 state
string reason_code              # EVIDENCE_CONFLICT, PLAN_PRECONDITION_UNCLEAR 등
string task_id
string[] scope_robot_ids
string[] scope_object_ids
string recovery_context_id
