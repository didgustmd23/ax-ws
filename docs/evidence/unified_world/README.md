# 통합 월드 실행 검증

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic에서 기존 build/install 없이 소스를 별도 디렉터리에 복사해 빌드했다.
기존 workspace 환경을 제거하고 ROS 기본 환경만 source했다. rosdep check, 13개 검사,
공구 5종×로봇 2종의 114개 메시 참조, 지도 상대경로 검사가 통과했다.
통합 월드 GUI에서 두 로봇·테이블 2개·상자가 생성됐고 제어기 5개 활성 및 LiDAR 4개 수신을 확인했다.

[기계 판독 결과](summary.json). 다른 물리 PC 또는 다른 GPU에서 직접 시험한 결과는 아니다.
집게 개폐 제어·TCP·일반 명령 충돌 차단 등 기존 제한을 해결했다는 의미도 아니다.
