# 물품 1개의 재고·그룹 물체 상태(설계 7.8·7.12절).
uint8 UNKNOWN=0                 # 물품 상태 불명
uint8 AT_SOURCE=1               # 출발 지점에 있음
uint8 HELD_BY_MANIPULATOR=2     # 조작 로봇 공구가 들고 있음
uint8 STOWED_ON_TRANSPORTER=3   # 운송 로봇 적재함에 있음
uint8 AT_DESTINATION=4          # 도착 지점에 배치됨
uint8 LOST=5                    # 운반 중 이탈·낙하가 확인됨. 이탈 여부를 확인할 수 없으면 UNKNOWN
string object_id                # 물품 ID
uint64 revision                 # 이 물품 상태가 바뀔 때마다 증가(D 원장 영속, 세션과 무관)
string receipt_event_id         # 이 물품을 등록한 입고 사건 ID
string location_id              # 확정 위치. 이동 중이면 빈 문자열
string carrier_robot_id         # 들고 있거나 적재한 로봇
uint64 assignment_generation    # carrier 로봇이 이 물품을 다룬 배정의 세대
uint8 state                     # 위 상수 중 물품 상태
bool isolated                   # 실패로 격리된 물품. false만으로 정상 판단하지 않고 state·hold로 확인한다
