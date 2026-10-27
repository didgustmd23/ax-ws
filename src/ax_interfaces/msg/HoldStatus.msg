# D → 관제·Agent. 보류 1건의 수명주기(설계 9.6절).
uint8 HOLD_PENDING=0            # 접수됨. 다음 단계 차단을 이미 적용하고 안전 경계 도달을 기다림(보수적 0값)
uint8 HELD=1                    # 보류 중. 영향 범위의 일반 작업은 다음 단계 시작 금지
uint8 RECHECK=2                 # 조건 재판정 중. 허용된 관측·복구만 수행하고, 안전 경계 미확인 동안은 일반 실행·복구 이동 금지
uint8 RELEASED=3                # 이 hold_id만 종료. 구동·인계·통로 잠금 해제가 아니다
uint8 FAILED_ISOLATED=4         # 자동 복구 실패로 종료(그룹 작업 FAILED). 잠금 해제가 아니다
string hold_id                  # 보류 ID
uint8 state                     # 위 상수 중 보류 상태
string reason_code              # 보류 사유 코드(설계 9.6절 사유, 예: EVIDENCE_CONFLICT, PLAN_PRECONDITION_UNCLEAR)
string task_id                  # 보류 대상 그룹 작업 ID
string[] scope_robot_ids        # 보류 영향 로봇 ID 목록
string[] scope_object_ids       # 보류 영향 물체 ID 목록
string precondition_id          # 보류 원인이 된 계획 조건 ID. 없으면 빈 문자열
string failure_event_id         # 보류 원인이 된 실패 사건 ID. 없으면 빈 문자열
string recovery_context_id      # 관련 복구 문맥 ID
