# B/C → D. 등록 입고 구역의 물품 관측. 관측 사실만 보내고 작업 생성은 D가 판단한다.
uint8 SOURCE_UNKNOWN=0          # 관측 출처 불명
uint8 SOURCE_SENSOR=1           # 센서 인식 기반
uint8 SOURCE_GROUND_TRUTH=2     # 시뮬레이터 정답 기반. 인식 성능 평가에 쓰지 않는다
StateMeta meta                  # 관측 출처·시각·revision·유효성
string observation_id           # 관측 ID
string zone_id                  # 등록 입고 구역 location_id
string object_id                # 식별 실패면 빈 문자열
string object_class             # 물체 종류(예: box_small)
float32 identity_confidence     # 0~1
geometry_msgs/PoseStamped pose  # map 기준 물체 pose
uint8 source                    # 위 SOURCE_ 상수 중 관측 출처
