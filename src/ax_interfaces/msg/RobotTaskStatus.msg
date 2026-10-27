# A → D. 로봇 로컬 실행 단계. 주행 단계는 D가 소유하므로 여기서 다루지 않는다.
StateMeta meta                  # 상태 출처·시각·revision·유효성
string robot_id                 # 대상 로봇 ID
ExecutionRef execution          # 현재 실행 식별자 묶음
string local_phase              # 예: LOCKING, PICKING, STOWING, RELEASING, RETRACTING
bool paused                     # 로컬 실행 일시 정지 여부
ResultStatus last_result        # 마지막 로컬 작업 결과
