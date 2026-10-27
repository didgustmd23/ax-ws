# D → Agent. 영역별 출처·시각·revision·유효성을 묶은 관측. 누락 관측을 정상으로 채우지 않는다.
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
