# AX_2 적재함 교체 검증 — 2026-11-10

ROS 2 Jazzy / Gazebo Harmonic / Ubuntu 24.04 WSL에서 현재 소스로
`ax_description`, `ax_control`을 새 설치 경로에 빌드해 확인했다.

## 반영 내용

- 새 CAD의 7개 부품과 재질을 다시 추출하고 원본·출력 SHA-256을 맞췄다.
- 적재함 충돌체 53개, 무게중심, 관성을 재생성했다. 차체는 27개, 전방 보조 외장은 3개 충돌체다.
- 새 CAD에 없는 옛 상부 LiDAR 외장은 활성 모델에서 제외하고 이전 자산 폴더에 보관했다.
  전후 주행 LiDAR는 유지한다.
- AX_2 무적재 총질량은 19.6kg, COM은 base_link 기준
  (0.0161244704, -0.0005910439, 0.1058547086)m다.
  질량 예산은 기존 가상 가정을 유지했으며 실물 측정이나 정격 확정이 아니다.
- 주행 외곽은 센서·새 적재함을 포함해 약 30mm 여유를 둔
  X[-0.63,0.67], Y[-0.53,0.53]m로 갱신했다.
- 깊이 카메라의 자동 투영·픽셀 중심 0.5px 보정으로 판 바깥 골격 오인을 수정했다.
  빈 판의 약 0.021283m 고정 돌출부는 점군과 일치한다.
- 실행 월드는 빈 바닥과 두 창고 두 개만 남겼다. 메뉴·설치·검사에서 AWS 선택을 제거했다.
- 부품 PNG·표·명칭 사전과 AX_1/AX_2별 Markdown·JSON 스펙을 갱신했다.

## 결과

| 검사 | 결과 |
|---|---|
| 두 패키지 새 경로 빌드 | 통과 |
| CAD 분리 및 물리 생성물 재생성 일치 | 통과 |
| 두 모델 질량·COM·관성·관절·메시 포함 검사 | 통과, 차체 미포함 정점 0개 |
| 모델·자산·센서·높이·영역·데스크톱·교체 회귀 단위 시험 | 41개 통과 |
| AX_1 주행 자세 및 무적재 주행 | 6개 방향·회전·곡선 시험 통과 |
| AX_2 0/0.5/1/2kg 주행·수동 적재 유지 | 하중별 6개, 총 24개 통과 |
| 빈 월드의 AX_2 생성·제어기·높이 허가 | 통과 |
| 창고 월드의 AX_2 생성·제어기·높이 허가 | 통과 |
| 창고 GUI에서 두 로봇 동시 생성·제어기·전후 LiDAR·관절 상태 | 통과 |

전체 주행 30개 시험에서 최대 차체 기울기는 약 6.30e-6rad,
적재 상자 최대 수평 이동은 약 10.73mm였다. 기존 기준(기울기 0.15rad 미만,
적재 이동 20mm 미만)을 만족했다. 시험 상자는 20cm 정육면체로 적재판 중앙에
수동으로 놓았으며 고정 관절로 붙이지 않았다. 최대 2kg까지 확인한 사실은
정격 2kg이나 모든 상자·배치·노면의 안정성을 뜻하지 않는다.

높이 시험은 물리 접촉 영향 없이 광학 자극용 외형을 놓아 확인했다.
두 월드 모두 빈 판 약 2.13cm, 30cm 물체는 허가, 60cm 물체는 거절했다.
상태 토픽과 `loading/check` 서비스 응답을 함께 확인했다.
이 검사는 적재 허가 인터페이스까지이며 실제 자동 적재 실행기는 아직 연결되지 않았다.

## 재실행

빌드한 ROS 환경을 source한 뒤 저장소 루트에서 실행한다.

```bash
python3 tools/update_ax2_parts.py --check
python3 tools/generate_physical_properties.py --check
python3 tools/generate_robot_specs.py
python3 tools/audit_robot_geometry.py
python3 -m unittest discover -s tools/tests -p test_w1_models.py
python3 -m unittest discover -s tools/tests -p test_a1_assets.py
python3 -m unittest discover -s tools/tests -p test_cargo_height.py
python3 -m unittest discover -s tools/tests -p test_b3_regions.py
python3 -m unittest discover -s tools/tests -p test_desktop_session.py
python3 -m unittest discover -s tools/tests -p test_ax2_revision.py
python3 tools/check_portability.py
python3 tools/verify_a1_envelope.py --mobility
python3 tools/check_ax2_revision_runtime.py
python3 tools/generate_parts_preview.py
```

상세 로컬 로그: `log/ax_review_checks.txt`, `log/ax_review_revision.txt`,
`log/repair/model_geometry.json`, `log/a1/mobility_summary.json`,
`log/ax_revision_runtime/result.json`, `log/ax_review_final.txt`,
`log/ax_revision_runtime/two_robot_warehouse.json` (로그는 Git 제외).

현재 두 창고의 새 지도·자동 운반 경로는 별도 작성 대상이다.
삭제된 AWS 월드의 지도·경로는 현재 환경에 적용하지 않는다.

## 후속 정리와 전체 테스트 확인

기존 Git 추적 테스트 파일 27개가 모두 남아 있음을 확인했다. 새 테스트도 유지한다.
`tools/tests`의 23개 테스트 파일을 파일별 독립 프로세스로 실행해 **358개 모두 통과**했다.
위치 추정 launch 테스트는 첫 실행 때 모델 전용 overlay에 패키지가 없어 실패했으나,
위치 추정·환경·인식·주행·작업 실행 패키지를 빌드한 뒤 재실행해 통과했다.
과거 AWS 경로에 남아 있던 옛 AX_2 외곽 치수는 현재 모델과 맞췄으며 테스트를 완화하지 않았다.

미사용 AWS 메시·텍스처 99개는 `log/removed_aws_assets/models` 백업으로 옮겨
소스·설치 대상에서 제외했다. 출처·라이선스와 회귀 시험에 쓰는 지도·설정은 유지한다.
옛 AWS 지도 작성 실행기는 삭제된 월드를 실행하려 하면 중단 사유를 안내한다.
상세 결과는 `log/ax_all_unit_tests.json`, `log/cleanup_test_*.txt`,
`log/ax_cleanup_checks.txt`에 기록했다.

## 2026-11-11 실행 맵 단일화

사용자 요청으로 빈 `ax_test.world`를 삭제하고 직접 만든 창고 맵만 남겼다.
창고 파일 내용은 변경하지 않았으며 기본 launch·Windows 메뉴·설치 검사도 창고 하나로 맞췄다.
기존 주행 시험의 빈 바닥은 `tools/empty_test_fixture.py`로 실행 시에만 생성한다.
테스트 파일은 삭제하지 않고 새 구성을 검증하도록 갱신했다.
관련 단위 시험 27개와 설치 자산 검사가 통과했고, 공백이 있는 임시 경로에서 생성한
빈 바닥으로 Gazebo 모델 생성이 되는 것도 확인했다.
상세 로그: `log/single_world_checks.txt`, `log/single_world_fixture_smoke.txt`.

## 2026-11-11 적재 높이 카메라 제거

사용자 요청으로 공중에 배치했던 cargo_rgbd 카메라와 광학 프레임, ROS 브리지,
높이 감시 노드의 설치·자동 실행을 제거했다. 전방 RGB-D와 접촉 센서는 유지한다.
현재 50cm는 설계 기준으로만 남으며 자동 높이 측정·적재 금지 신호·허가 서비스는 없다.
과거 높이 계산 코드와 테스트는 비활성 회귀 검증 자료로 보관한다.

카메라 질량 0.15kg을 제외해 총질량 19.45kg, COM은 base_link 기준
(0.0163041386, -0.0005727268, 0.0969004887)m로 갱신했다.
관련 단위 시험 25개와 실제 Gazebo의 카메라·감시 노드 부재, 전방 카메라·접촉 센서·
주행 제어기 유지 검사가 통과했다. 스펙 JSON/Markdown과 부품 이미지도 재생성했다.
로그: `log/camera_removal_checks.txt`, `log/ax_revision_runtime/camera_removed.json`.
