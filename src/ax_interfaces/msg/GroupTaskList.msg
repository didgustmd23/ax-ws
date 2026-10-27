# D → A·C4·E·관제. 활성 그룹 작업 전체. transfer_state가 NONE·UNLOCKED가 아닌 작업은 작업 상태와 관계없이 D가 잠금 해제를 확인할 때까지 목록에 남긴다. 적용 규칙은 계약 4.3절. keep_last 1로도 늦게 접속한 노드가 모든 작업을 받는다.
StateMeta meta                  # 목록 전체의 출처·시각·revision·유효성
GroupTaskStatus[] tasks         # 그룹 작업 목록
