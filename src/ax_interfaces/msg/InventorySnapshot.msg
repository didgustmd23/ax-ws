# D → Agent·관제. 재고와 등록 지점의 버전 있는 상태.
StateMeta meta                  # 재고 전체의 출처·시각·revision·유효성
uint64 map_revision             # 등록 지점이 기준으로 삼은 지도 revision(D 원장 영속, 세션과 무관)
RegisteredLocation[] locations  # 등록 지점 목록
InventoryEntry[] items          # 물품 목록
