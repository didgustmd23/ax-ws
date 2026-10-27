# 물품 1개의 재고·그룹 물체 상태(설계 7.8·7.12절).
uint8 UNKNOWN=0
uint8 AT_SOURCE=1
uint8 HELD_BY_MANIPULATOR=2
uint8 STOWED_ON_TRANSPORTER=3
uint8 AT_DESTINATION=4
uint8 LOST=5
string object_id
uint64 revision                 # 이 물품 상태가 바뀔 때마다 증가
string receipt_event_id
string location_id              # 확정 위치. 이동 중이면 빈 문자열
string carrier_robot_id         # 들고 있거나 적재한 로봇
uint64 assignment_generation    # carrier 로봇의 배정 세대
uint8 state
bool isolated                   # 실패로 격리된 물품
