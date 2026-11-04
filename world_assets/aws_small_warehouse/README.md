# 택배 물류 시험 후보 — AWS Small Warehouse

선반·통로·팔레트 운반 장비 등 창고 배경을 제공하는 외부 월드다. 지붕 없는 원본을 사용해 위에서 로봇을 관찰하기 쉽게 구성했다.
기존 `ax_test.world`와 계획 창고를 유지하며 **AX 월드 선택**의 세 번째 항목에서 AWS 물류 창고를 선택할 수 있다.

## 보관 내용

- `upstream/`: 원본 모델·월드·README·MIT-0 라이선스. 원본은 수정하지 않았다.
- `SOURCE.json`: 원본 저장소·고정 commit·다운로드 아카이브 해시.
- `aws_warehouse_harmonic.world`: 지붕 없는 월드의 Harmonic용 변환본. 원본 모델을 월드에 펼쳐 넣고 배경을 모두 정적 장애물로 고정했다. 원본 바닥 관성은 삼각 부등식을 만족하지 않아 Harmonic 로딩에 실패하므로 정적 배경의 불필요한 관성 요소를 제외했다. 원본 메시·텍스처는 그대로 참조하며 Physics·UserCommands·SceneBroadcaster 시스템과 조명을 설정했다.
- `open_world.sh`: 파일 위치에서 상대적으로 모델 경로를 찾는 별도 미리보기 실행기.

출처: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ee0af733315e78432408c3cd98d378ecee5f767c

## Ubuntu에서 월드만 미리보기

ROS 2 Jazzy와 Gazebo Harmonic이 설치된 Ubuntu에서 저장소 루트로 이동한 뒤 실행한다.

```bash
bash world_assets/aws_small_warehouse/open_world.sh
```

이 명령은 창고만 연다. AX 로봇 두 대와 제어기를 함께 실행하려면 빌드 후 다음 명령을 사용하거나 **AX 월드 선택 → AWS 물류 창고**를 선택한다.

```bash
ros2 launch ax_control simulation.launch.py world:=aws_warehouse.world
```

`ax_description` 빌드 시 외부 자산도 함께 설치한다. launch가 설치 위치에서 모델을 찾아 연결하므로 개인 절대경로를 기록하지 않는다. 센서·IMU·접촉 시스템을 추가했고 내부 월드 이름은 기존 AX 연동과 같은 `ax_test`다. 리모컨의 공구 교체·초기화도 선택한 월드를 유지한다.
미리보기를 마치면 실행 터미널에서 Ctrl+C로 종료한다.

## 택배 로봇 시험에 사용할 때

선반과 통로는 주행·장애물 회피 시험의 배경으로 쓸 수 있다. 기존 AX 로봇의 폭·팔 작업 공간과 통로 여유는 별도로 확인해야 한다.
배경 모델은 집어서 운반하는 택배 상자와 구분한다. 실제 집기·운반 시험에는 동적 상자, 입고·출고 작업대, 적재·하역 위치를 따로 구성해야 한다.
기존 `maps/warehouse_map.yaml`이 이 월드와 맞는다는 뜻은 아니며 주행용 지도는 별도로 작성·검증해야 한다.
원본은 Gazebo Classic용으로, Harmonic에서 재질 표현이 원본과 다를 수 있다. 전체 AX 운반 동작의 검증 완료를 뜻하지 않는다.
