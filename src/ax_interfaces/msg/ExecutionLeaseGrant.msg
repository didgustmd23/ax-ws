# A → C. lease 응답. 기한은 C의 요청 시각 tC + lease_s이며 0 < lease_s <= min(상한, tA + H - 현재).
string request_id
string execution_id
uint64 lease_generation
bool granted
float32 lease_s                 # L
string reason_code
