# 실행 결과 공통 형식. 완료·실패·취소·시간초과·미확정을 구분한다(DEC-06).
uint8 UNSPECIFIED=0             # 채우지 않은 결과. 소비자는 UNCERTAIN과 같이 취급한다
uint8 SUCCEEDED=1               # 완료 확인
uint8 FAILED=2                  # 실패
uint8 CANCELED=3                # 취소
uint8 TIMED_OUT=4               # 시간초과
uint8 UNCERTAIN=5               # 실제 결과를 확인할 수 없음. 성공으로 처리하지 않는다
uint8 outcome                   # 위 상수 중 결과
string error_code               # 오류 코드(계약 D-13: 설계 9.1절 + 추가 코드)
string failure_event_id         # 실패 사건으로 등록했을 때만
string detail                   # 사람이 읽는 상세 설명(판정에 쓰지 않음)
