# 모델 설정과 생성 보고서

| 파일 | 역할·단위 | 수정 후 처리 |
|---|---|---|
| `arm_joint_limits.yaml` | 팔 5축 위치 rad, 속도 rad/s, 가속도 rad/s², effort N·m | 자산 manifest 갱신·제어/시험 검증 |
| `ax1_asset_v2.yaml` | 원본→링크 로컬 변환. xyz m, rpy rad, 질량 kg, 출력 OBJ mm | 자산 생성 후 물리 속성 검사/갱신 |
| `epick.yaml` | 개발용 본체/컵 질량 kg, 형상·ToF·TCP 길이 m | Xacro 재전개와 TCP/ToF 검증 |
| `sensor_mounts.yaml` | 부모 링크 기준 xyz m/rpy rad, ToF 범위·lip 오프셋 m | 장착·시야·자기 차단 확인 |
| `sensor_contract.yaml` | 토픽·타입·QoS·유효성 계약 | 생산자/소비자 함께 검토 |
| `a1_validation.yaml` | 시험 하중 kg, 시간 s, 오차 rad, 정지 속도 rad/s | 변경된 시험 조건으로 재검증 |

질량·모터·컵 값은 실측 제품 사양이 아닌 개발용 가정이다.
제어기 설정은 이 폴더가 아니라 A2 `src/ax_control/config`에 있다.

## JSON 보고서 읽기

`physical_properties.json`, `ax_2_physical_properties.json`은
`tools/generate_physical_properties.py`의 생성물이다. JSON에는 주석을 직접 넣지 않는다.

- `mesh_directory`, `mesh_sha256`: 계산에 사용한 메시와 원본 일치 확인.
- `pivot_m`, `mesh_bounds_link_m`: 링크 좌표 기준 피벗·형상 경계(m).
- `mass_kg`, `center_of_mass_m`: 링크 질량(kg)과 해당 링크 기준 COM(m).
- `inertia_kg_m2`: COM 기준 3×3 관성 텐서(kg·m²). 비대각 성분도 포함한다.
- `mass_parts`: 가상 배터리/모터/튜브/구조물에 배분한 질량·위치·크기.
- `collision_count`, `all_component_vertices_enclosed`: 충돌 근사 개수와 메시 정점 포함 검사 결과.

구조물 질량은 근사 상자 부피 비율로 배분한다. 빈 공간까지 채우는 근사일 수 있으므로
표면 메시에서 실제 재질 밀도를 계산한 결과로 해석하지 않는다.
전체 로봇 COM은 관절 변환까지 합산해야 하며 팔 자세와 선택 공구에 따라 달라진다.

V2 `manifest.json`은 메시 폴더에 있으며 원본/출력 해시, 재질 해시, 관절 프레임과 설정 해시를 기록한다.
생성 파일은 직접 고치지 않고 생성기/명세를 수정한다. 전체 읽는 순서는
[팀원용 코드 안내](../../../docs/guides/CODE_GUIDE.md)를 참고한다.

## 교체 공구 TCP

두 공구의 공통 작업 기준은 `tool_tcp`이며 기존 계약 이름 `epick_tcp`와 위치·방향이 같다.
실행 중에는 `robot_1/` 프레임 접두사가 붙는다. TCP의 +Z는 공구에서 물체로 향하는 접근 방향이다.

| 공구 | 기준 위치 | gripper_base_link 기준 xyz (m) |
|---|---|---|
| 다중 흡착판 · 12컵 | 변형 전 컵 접촉면 배열 중앙 | `0.15034361 0 0` |
| 손가락 집게 · Robotiq 2F-85 | 기본 열린 자세의 양쪽 안쪽 패드 높이 중앙 | `0.15034361 0 0` |

정의는 `urdf/ax.urdf.xacro`와 `urdf/epick_adapter.xacro`에 있다. 집게 형상·장착 위치는 유지한다.
흡착판 장착 원점을 `0.02 0 0`에서 `0.03304361 0 0`으로 13.04361 mm 옮겨
같은 팔 자세에서 두 TCP가 일치하게 한다. 실제 제작 시 이 오프셋에 맞는 어댑터가 필요하다.
집게 값은 현재 collision 메시에서 정한 명목 기준이고 개폐에 따른 패드 이동·실물 보정은 포함하지 않는다.
하중 시험은 TCP 위치 변경의 영향을 받으므로 기존 집게 하중 시험 결과를 그대로 재사용하지 않는다.

다중 흡착판 ToF는 헤드 바깥 `0.105 0 0.095` m에 있고 광선을 안쪽으로 20° 기울인다.
TCP 중심에 정렬되고 컵 면과 평행한 폭 0.2 m 상자를 접촉~0.3 m 간격에서 볼 수 있도록 한 개발용 배치다.
원시 Range는 광선상 거리이므로 컵 평면 기준 법선 간격은
`(range - tof_lip_offset_m) * tof_normal_cosine`이다. 상자 면이 기울어졌거나 정렬 오차가 크면
이 식만으로 표면 간격을 확정할 수 없다. 접촉면에서 광선은 상자 가장자리 안쪽 약 3.1 mm를 지나므로
가로 위치 오차가 있으면 다시 놓칠 수 있다. 실물 센서 치수·시야각·장착 공차는 별도 확인해야 한다.
