# 등록 지점(설계 7.12절). 의미가 같은 지점은 지도 변경 시에도 ID를 유지한다.
uint8 UNKNOWN=0
uint8 AVAILABLE=1
uint8 RESERVED=2
uint8 OCCUPIED=3
uint8 INACTIVE=4
string location_id
uint64 location_revision
uint64 map_revision
string purpose                  # RECEIPT, STORAGE, HANDOFF, WAIT, SHIPPING 등
string zone_id
uint32 capacity
string[] allowed_object_classes
float32 max_load_kg
uint8 state
string reservation_id
geometry_msgs/PoseStamped pose          # map 기준 작업 기준 pose
geometry_msgs/PoseStamped approach_pose # 접근 pose
string wait_location_id                 # 대응 대기 지점
RobotStopPose[] stop_poses              # 역할별 정차 자세
