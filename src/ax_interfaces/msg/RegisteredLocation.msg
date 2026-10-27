# 등록 지점(설계 7.12절). 의미가 같은 지점은 지도 변경 시에도 ID를 유지한다.
uint8 UNKNOWN=0                 # 지점 상태 불명. 사용 가능으로 보지 않는다
uint8 AVAILABLE=1               # 사용 가능
uint8 RESERVED=2                # 예약됨
uint8 OCCUPIED=3                # 점유됨
uint8 INACTIVE=4                # 사용 중지
string location_id              # 등록 지점 ID
uint64 location_revision        # 지점 정보가 바뀔 때마다 증가(D 원장 영속, 세션과 무관)
uint64 map_revision             # 이 지점이 기준으로 삼은 지도 revision(D 원장 영속, 세션과 무관)
string purpose                  # 지점 용도: RECEIPT(입고), STORAGE(보관), STATION(작업대, 예: station_src·station_dst), HANDOFF(인계), WAIT(대기), SHIPPING(출고) 중 하나
string zone_id                  # 소속 구역 ID
uint32 capacity                 # 수용 가능 물품 수
string[] allowed_object_classes # 허용 물체 종류 목록. 빈 목록은 모든 종류 배치 불가이며 제한이 없으면 "*" 하나를 넣는다
float32 max_load_kg             # 최대 허용 하중, kg
uint8 state                     # 위 상수 중 지점 상태
string reservation_id           # 현재 예약 ID. 없으면 빈 문자열
geometry_msgs/PoseStamped pose  # map 기준 작업 기준 pose
geometry_msgs/PoseStamped approach_pose # 접근 pose
string wait_location_id         # 대응 대기 지점
RobotStopPose[] stop_poses      # 역할별 정차 자세
