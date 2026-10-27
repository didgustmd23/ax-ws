# A → D. 로봇 로컬 실행 단계. 주행 단계는 D가 소유하므로 여기서 다루지 않는다.
StateMeta meta
string robot_id
ExecutionRef execution
string local_phase              # 예: LOCKING, PICKING, STOWING, RELEASING, RETRACTING
bool paused
ResultStatus last_result
