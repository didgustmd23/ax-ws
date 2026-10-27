# D 내부·관제. 반복 관측과 신규 입고를 구분한 입고 사건.
uint8 PENDING_IDENTIFICATION=0  # 물체 식별 대기(보수적 0값)
uint8 TASK_CREATED=1            # 작업 생성됨
uint8 DUPLICATE=2               # 기존 물품의 반복 관측
uint8 REJECTED=3                # 등록 불가로 거절
string receipt_event_id         # 입고 사건 ID
string object_id                # 식별된 물체 ID
string zone_id                  # 입고 구역 location_id
string[] observation_ids        # 이 사건에 묶인 관측 ID 목록
uint8 state                     # 위 상수 중 사건 상태
string task_id                  # 생성된 작업 ID. 없으면 빈 문자열
