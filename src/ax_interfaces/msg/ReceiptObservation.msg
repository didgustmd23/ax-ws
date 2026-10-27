# B/C → D. 등록 입고 구역의 물품 관측. 관측 사실만 보내고 작업 생성은 D가 판단한다.
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_SENSOR=1
uint8 SOURCE_GROUND_TRUTH=2     # 시뮬레이터 정답 기반. 인식 성능 평가에 쓰지 않는다
StateMeta meta
string observation_id
string zone_id                  # 등록 입고 구역 location_id
string object_id                # 식별 실패면 빈 문자열
string object_class
float32 identity_confidence     # 0~1
geometry_msgs/PoseStamped pose  # map 기준 물체 pose
uint8 source
