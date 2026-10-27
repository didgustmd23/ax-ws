# D → A·B·Agent. 통로 허가·점유(설계 7.3·9.4절). 활성 허가는 1 이하.
uint8 FREE=0
uint8 RESERVED=1
uint8 OCCUPIED=2
uint8 BLOCKED_UNKNOWN=3
string corridor_id
uint8 state
string owner_robot_id
string active_grant_id
uint64 corridor_revision
