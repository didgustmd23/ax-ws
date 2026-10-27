# D → A·B·Agent. 통로 허가·점유(설계 7.3·9.4절). 활성 허가는 1 이하.
uint8 FREE=0                    # 비어 있음(설계 7.3절 숫자). 목록 meta가 무효·만료·revision 0이면 UNKNOWN이며 허가 근거가 아니다
uint8 RESERVED=1                # 허가됨, 아직 진입 전
uint8 OCCUPIED=2                # 점유 중. 신규 허가 없음
uint8 BLOCKED_UNKNOWN=3         # 점유 불확실. 신규 허가 없음
string corridor_id              # 통로 ID
uint8 state                     # 위 상수 중 현재 상태
string owner_robot_id           # 허가·점유 중인 로봇 ID. 없으면 빈 문자열
string active_grant_id          # 활성 허가 ID. 없으면 빈 문자열
uint64 corridor_revision        # 통로 상태가 바뀔 때마다 증가(D 원장 영속, 세션과 무관)
