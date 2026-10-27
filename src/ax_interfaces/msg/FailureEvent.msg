# 실패 사건(설계 9.1·9.5·9.6절). 사람 응답을 기다리지 않고 종료·격리·알림을 기록한다.
uint8 UNSPECIFIED=0             # 분류 절차 전
uint8 CLASSIFIED=1              # 오류 코드로 분류 완료
uint8 UNCLASSIFIED=2            # 사건 소유자가 분류 불가로 확정
string failure_event_id         # 실패 사건 ID. 사건 소유자가 발급
string owner_domain             # 사건 소유 영역: A, B, C, D(공용 감독기는 A). E는 사건 소유자가 아니다
string code                     # 오류 코드(계약 D-13: 설계 9.1절 + 추가 코드)
uint8 classification            # 위 상수 중 분류 상태
string task_id                  # 관련 그룹 작업 ID
string object_id                # 관련 물체 ID
string robot_id                 # 관련 로봇 ID
string recovery_context_id      # 관련 복구 문맥 ID
bool recovery_exhausted         # 자동 복구 예산 소진 여부
bool post_review_required       # POST_REVIEW_REQUIRED 알림 대상
builtin_interfaces/Time occurred_at # ROS sim time
string detail                   # 사람이 읽는 상세 설명(판정에 쓰지 않음)
