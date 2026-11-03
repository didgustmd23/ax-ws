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
