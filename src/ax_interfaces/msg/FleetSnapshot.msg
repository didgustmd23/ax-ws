# D → Agent. 영역별 출처·시각·revision·유효성을 묶은 관측. 누락 관측을 정상으로 채우지 않는다.
# 안쪽 항목의 meta·영역 meta가 바깥 meta보다 우선한다. 바깥 meta가 무효면 전체를 무효로 본다.
StateMeta meta                  # header.stamp: 생성 시각(ROS sim time), revision: snapshot 순번
uint64 snapshot_id
OperationMode mode
RobotSummary[] robots
GroupTaskStatus[] tasks
InventorySnapshot inventory
CorridorState[] corridors
CandidateSet[] candidate_sets
HoldStatus[] holds
string[] missing_sources        # 이번 snapshot에서 만료·누락된 관측 출처
