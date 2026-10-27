# 논리 실행 식별자 묶음(설계 7.1·7.9절). execution_id는 D가 정한 규칙으로만 만든다.
string team_id
string task_id
uint64 task_revision
string assignment_id            # 재배정·재시작 구분. 이전 배정 결과가 현재 배정을 덮어쓰지 못하게 검사
uint64 assignment_generation
string transfer_id              # 인계 관련 실행일 때만
string root_execution_id
string parent_execution_id      # 중첩 호출일 때만
string execution_id             # derive(root_execution_id, step_key, operation, attempt_index)
string derive_rule_version      # 예: derive-v1
string step_key                 # 계획에 영속 저장된 단계 식별자
string operation                # Goal enum 상수 이름 문자열(예: PICK_AND_STOW, PICK)
uint32 attempt_index            # 승인된 새 물리 시도만 증가
