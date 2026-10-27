# D → Agent·로그. 제안·계획 검증 결과.
uint8 UNSPECIFIED=0             # 채우지 않은 결정. 수락으로 해석하지 않는다
uint8 ACCEPTED=1                # 검증 통과. D가 실행에 반영(실행 성공은 아님)
uint8 REJECTED=2                # 검증 실패로 거절
uint8 RECORDED_ONLY=3           # ADVISORY·RECOVERY_SHADOW에서 기록만 함
string sender_agent_id          # 제안·계획을 보낸 Agent ID(예: agent_central)
string proposal_id              # 대상 제안 ID. 계획 결정이면 빈 문자열
string plan_id                  # 대상 계획 ID. 제안 결정이면 빈 문자열
uint64 plan_revision            # 대상 계획 revision. 제안 결정이면 0
uint64 snapshot_id              # 검증에 쓴 snapshot ID
uint8 decision                  # 위 상수 중 결정 결과
string reason_code              # 결정 사유 코드(계약 D-19 Agent 결정 사유 목록)
