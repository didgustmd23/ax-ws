# D → Agent. 영역별 출처·시각·revision·유효성을 묶은 관측. 누락 관측을 정상으로 채우지 않는다.
# 안쪽 항목의 meta·영역 meta가 바깥 meta보다 우선한다. 바깥 meta가 무효면 전체를 무효로 본다.
StateMeta meta                  # header.stamp: 생성 시각(ROS sim time). revision 등은 StateMeta 규칙을 따른다
uint64 snapshot_id              # D가 발급하는 snapshot 순번. 마지막 값을 원장에 저장해 재시작 후에도 이어 증가하고, 내용이 같은 재발행은 같은 ID
OperationMode mode              # 현재 운영 모드
RobotSummary[] robots           # 로봇별 요약
GroupTaskStatus[] tasks         # 그룹 작업 상태 목록
InventorySnapshot inventory     # 재고와 등록 지점 상태
CorridorState[] corridors       # 통로 상태 목록
CandidateSet[] candidate_sets   # 게시 중인 조작 후보 묶음
FailureEvent[] open_failures    # 종료되지 않은 실패 사건(상한 32). 제안의 fail-… 근거는 여기에 있어야 한다
HoldStatus[] holds              # 보류 상태 목록
string[] missing_sources        # 만료·누락된 관측의 source_id 목록. 크기 상한으로 잘린 영역은 TRUNCATED:<영역>(예: TRUNCATED:inventory)
