# ax-ws

**ax-ws는 입고 물품을 자동 확인하고, 중앙 LLM이 전체 운반 계획을 수립하며 Fleet이 검증·실행하는 가상 창고 운영 무인 시스템을 개발하는 프로젝트다.** ROS 2 Jazzy와 Gazebo Harmonic을 사용한다.

목표 동작은 다음과 같다. B·C가 입고를 관측하면 D가 중복 없이 작업을 생성한다. 중앙 LLM은 보관 규칙·가용 등록 위치·로봇 상태를 보고 목적지와 전체 계획을 생성하고, D가 용량·예약·권한·통로·예산을 검사해 실행을 조정한다. 조작 로봇이 집기·적재·하역하고 운송 로봇이 적재함으로 이동한다. 같은 역할의 검증된 예비 로봇 재배정도 목표 범위다.

기능 E에는 중앙·로봇별 Agent를 두며 전체 계획 책임은 중앙에 있다(ASSISTED 모드). D는 Nav2 주행을 직접 조정하고 A의 로컬 실행기에 작업을 전달한다. A는 최종 구동·차단, B는 위치·환경·위험, C는 조작 전략·보유 판정을 맡는다. 네 사람은 A~E의 같은 번호 세부 작업을 분담하며 기능과 개인을 일대일 고정하지 않는다.

최상위 목표는 완전 무인 운영이며, 설계에서는 이를 **운영 무인**으로 정의한다. 검증된 자동 복구가 실패·소진되면 작업을 종료·격리·알림 처리하고, 안전 조건을 확인한 독립 후속 작업만 계속한다. 사람은 사후에 원인 수정·물품 정리·재등록을 수행한다. 정상 입고는 물품별 사람 명령·승인을 요구하지 않는다.

규칙 기반 자동 입고·공동 운반부터 검증하고, SROS2+D 권한 검사 후 ADVISORY에서 중앙 계획을 평가한 뒤 ASSISTED로 확장한다. LLM 장애·계획 거절 시 유효한 규칙 경로가 있으면 사용하고, 없으면 유한 복구·실패 절차를 따른다. 관제 화면은 통합 1개이며 디자인은 추후 구체화한다.

[통합 상세설계](docs/design/DETAILED_DESIGN.md) · [팀 공통 일정·A1~E4 분담](docs/management/TEAM_SCHEDULE.md) · [전체 문서 안내](docs/README.md)

프로젝트·GitHub 저장소 이름: **`ax-ws`**. ROS 모델 이름은 `ax`, 로컬 작업공간은 `~/ax_ws`로 사용한다.

로컬 작업 지침(`CLAUDE.md`, `.claude/`, `AGENTS.md`, `.agents/`, `.codex/`, `.codex-harness/`)은 로컬 작업 환경 전용이며 Git에 포함하지 않는다.

## 현재 저장소 상태

현재 저장소는 **설계·일정 단계**다. 작업트리에는 기능 구현 소스가 없고 설계·일정 문서, 공유 구성도, 모델 자산, 공통 인터페이스 타입 초안(`src/ax_interfaces`)만 있다.

이전 개발 구성(AX_1·AX_2 모델, 로봇별 ros2_control 기본 제어, Gazebo 센서, 개발용 관절 보호 `arm_command_guard`)과 그 인터페이스·검증·팔 사양 문서는 커밋 [`4f85007`](https://github.com/didgustmd23/ax-ws/tree/4f8500755151b2fbcbf35ab9cbca13fb1d71cea9)까지의 Git 이력으로만 남긴다. 이전 구현은 G1(안전 게이트·최종 구동 차단)을 충족하지 않았으며, 이력의 코드·수치·시험 결과를 새 구현의 통과 근거로 쓰지 않는다. 경위와 조회 방법은 [이전 구현 기록](docs/management/TEAM_SCHEDULE.md#이전-구현-기록)을 따른다.

```bash
git show 4f85007:docs/guides/INTERFACES.md      # 이전 인터페이스 조회
git checkout 4f85007 -- src/ax_control          # 필요한 부분만 복원
```

## 전체 시스템 구성도

### 목표 구성

아래는 입고부터 재고 갱신까지 기능 A~E가 어떻게 연결되는지 보여 주는 **목표 설계의 간략 구조**다. 블록별 세부는 상세 기능 구성도를 본다.

![AX 전체 시스템 — 입고부터 재고 갱신까지](https://raw.githubusercontent.com/didgustmd23/ax-ws/main/docs/assets/diagrams/ax-system.webp?v=89df46995ea9)

[인터랙티브 전체 구성도](https://didgustmd23.github.io/ax-ws/overview.html) · [이미지 크게 보기 · WebP](docs/assets/diagrams/ax-system.webp) · [상세 기능 구성도](https://didgustmd23.github.io/ax-ws/functions.html) ([HTML](docs/assets/diagrams/ax-functions.html))

웹 링크는 main 반영과 GitHub Pages 최초 배포 후 사용할 수 있다. 배포 전에는 [전체 구성도 HTML](docs/assets/diagrams/ax-system.html)을 내려받아 브라우저로 연다.

<details>
<summary>구성도 웹 배포 설정</summary>

저장소의 **Settings → Pages → Build and deployment → Source**를 **GitHub Actions**로 선택한다. 이후 main에서 배포 파일이 변경되면 `Publish AX diagrams` 워크플로가 사이트를 갱신한다. 수동 실행은 main을 선택한다.

공개 대상은 `docs/assets/diagrams/`에서 명시한 구성도 HTML 7개와 투명 WebP 7개다. `.archify/`의 작업 원본·JSON·검증 자료·글꼴은 Git과 Pages 배포에서 제외한다.

</details>

목표 구성도는 기능 A~E의 책임을 표시한다. 번호별 담당은 1 이종수·2 양현승·3 김성민·4 엄승연이며 [작업자 배정표](docs/management/TEAM_SCHEDULE.md#번호별-작업자-배정)를 따른다. 기간은 2026-10-28 ~ 2027-01-28(개발 2개월·시험 1개월)이며 주차 계획은 [달력 일정](docs/management/TEAM_SCHEDULE.md#달력-일정--2026-10-28--2027-01-28)을 따른다.

### 공동 운반 흐름

[![자동 입고·공동 운반·실패 복구 — 목표 설계](docs/assets/diagrams/ax-transfer.webp)](https://didgustmd23.github.io/ax-ws/transfer.html)

[인터랙티브 구성도](https://didgustmd23.github.io/ax-ws/transfer.html) · [HTML 내려받기](docs/assets/diagrams/ax-transfer.html) · [이미지 크게 보기 · WebP](docs/assets/diagrams/ax-transfer.webp)

팔 동작 중 조작 로봇 주행과 적재·하역 중 운송 로봇 주행을 차단하는 것이 목표다. 두 로봇의 대형 유지 이동 여부는 미정이다.

[최종 차단 권고 구조](docs/assets/diagrams/ax-safety.html) · [전체 구성도 목록](docs/README.md#구성도-모음)

## 구성

| 경로 | 용도 |
|---|---|
| [docs/design/DETAILED_DESIGN.md](docs/design/DETAILED_DESIGN.md) | 목표 설계·결정·계약·시험 요구 |
| [docs/management/TEAM_SCHEDULE.md](docs/management/TEAM_SCHEDULE.md) | A1~E4 작업 분해·담당·의존성·단계·이전 구현 기록 |
| [docs/design/INTERFACE_CONTRACT.md](docs/design/INTERFACE_CONTRACT.md) | 공통 인터페이스 계약 초안(K0) |
| `src/ax_interfaces/` | 공통 메시지·서비스·액션 타입 초안(55개). `colcon build --packages-select ax_interfaces` |
| [docs/management/GIT_UPLOAD_POLICY.md](docs/management/GIT_UPLOAD_POLICY.md) | Git 포함·제외 기준, 초안·이전 구현의 정리 원칙 |
| `docs/assets/diagrams/` | 공유 구성도 HTML·투명 WebP |
| `src/ax_description/meshes/` | AX_1·AX_2·상자 원본 OBJ·MTL과 분리 메시. [분리 메시 README](src/ax_description/meshes/parts/README.md) |
| `src/robotiq_description/` | 외부 Robotiq 모델 subset·LICENSE·원본 해시. [패키지 안내](src/robotiq_description/README.md) |
| `maps/warehouse_map.yaml`, `.pgm` | 기존 지도. 대응하는 창고 월드는 포함하지 않음 |

## 전달 파일

`docs/`, `src/`의 모델 자산과 공통 인터페이스 타입 초안(`src/ax_interfaces`), `maps/`, README를 전달한다. 지도 사용 시 YAML과 PGM을 함께 전달한다. `build/`, `install/`, `log/`, `legacy_before_rename/`, `__pycache__/`, `.bak`, `Zone.Identifier` 부가 파일은 제외한다. 외부 라이선스·manifest를 함께 전달한다.

Archify는 `docs/assets/diagrams/`의 공유 HTML·WebP만 포함하고 `.archify/` 원본은 제외한다.
