# B3 W1 통로·작업대 영역 정의

B3 김성민 담당의 통로·작업대·대기 영역 장부다. 다른 담당자의 지도·제어·허가 코드는 바꾸지 않는다.
현재는 W1 영역 정의와 순수 기하 검사까지이며 센서 관측 발행은 W2 후속 작업이다.

## 파일과 의미

- `config/ax_test_regions.json`: 실제 `ax_test.world` 작업대의 collision 경계를 기록한다.
  통로·대기 구역은 B3가 제안한 논리 영역이며 물리 통로나 안전한 주차 위치로 승인한 것이 아니다.
- `config/aws_warehouse_regions.json`: 채택한 운영 지도가 없어 좌표를 비워 둔다.
  B2의 odom 경유점을 map 좌표로 복사하지 않는다.
- `ax_environment/regions.py`: 파일 형식·경계 검사와 전체 로봇 외곽 기반 판정 함수.

두 설정 모두 `approved=false`다. `observe_corridor`는 미승인 설정에서 항상 UNKNOWN을 반환한다.
`--require-approved` 검사는 종료 코드 2로 운영 연결을 차단한다.

## 사용

```bash
ros2 run ax_environment check_regions
ros2 run ax_environment check_regions --config src/ax_environment/config/ax_test_regions.json
```

경계 배열은 `[xmin, ymin, xmax, ymax]`(m)다. 로봇 중심이 아니라 접힌 팔·적재물을 포함한
전체 외곽의 축 정렬 포위 사각형을 사용하며 위치 불확실성만큼 넓힌다.
경계에 닿거나 조금이라도 겹치면 INSIDE이고, 전체 외곽이 떨어졌을 때만 OUTSIDE다.
좌표계·map revision 불일치, 위치 만료, 외곽 미확인은 UNKNOWN이다.
회전한 외곽의 포위 사각형은 보수적이므로 실제 비점유 상태도 INSIDE로 나올 수 있다.
APPROACHING 판정·센서 연결·Topic 발행·허가·안전 게이트는 여기서 구현하지 않는다.

## 운영 연결 전에 필요한 공동 확인

1. B2: 채택 지도·revision과 map↔odom 변환.
2. A1/A2/A4: robot_1 접힌 팔과 robot_2 적재물까지 포함한 외곽·위치 여유.
3. B3/D1/D3/A4: 실제 통로 경계·진입선·접근 영역·대기 지점·완전 이탈 기준 합의.
4. D2: 작업대·등록 지점 ID와 용량·접근 조건 확인.

승인은 파일의 bool을 바꾸는 것만으로 성립하지 않는다. 위 확인과 관측 시험 근거가 필요하다.
