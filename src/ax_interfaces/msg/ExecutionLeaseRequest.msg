# C → A. 실행 lease 요청. 첫 물리 시도 전에 완료해야 한다.
uint8 UNSPECIFIED=0             # 채우지 않은 요청. A는 거절
uint8 FIRST=1                   # 이 Goal의 lease 요청(첫 요청 또는 응답 유실·만료 뒤 재동기화). A는 goal_id로 판정하고, 재동기화는 이전 세대 차단·A 측 수락 기록 확인 뒤에만 발급
uint8 RENEW=2                   # 보유 중인 유효 lease의 갱신. 만료·철회된 세대면 A는 거절. 응답을 잃으면 C는 기존 기한까지만 실행한다
uint8 kind                      # 위 상수 중 요청 종류
string goal_id                  # C가 받은 ManipulateObject Goal의 UUID(소문자 16진수 32자). A3는 현재 활성 Goal과 같을 때만 처리
string c_session_id             # C 프로세스 세션 ID(기록·진단용). Goal 구분은 goal_id로 한다
string request_id               # 요청 ID. 요청·갱신마다 새로 발급(요청 시각 tC와 짝). 같은 ID 재전송에는 A가 기존 응답만 돌려준다
string execution_id             # 물리 시도를 할 실행 ID
string budget_grant_id          # 사용할 예산 ID
uint64 lease_generation         # FIRST면 Goal의 lease_generation(기록용), RENEW면 현재 보유 세대
