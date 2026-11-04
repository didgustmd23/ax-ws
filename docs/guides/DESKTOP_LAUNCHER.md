# 바탕화면에서 한 번에 실행·종료

Windows 저장소 폴더의 **`remote_controller/install.cmd`를 더블클릭**한다.
바탕화면에 **AX 월드 선택**, **AX 리모컨**, **AX 종료**, **AX 화면 복구** 바로가기를 만든다. 기존 AX 실행 바로가기는 월드 선택으로 통합해 제거한다. 최초 ROS 설치·빌드는 자동 수행하지 않는다.

Windows PowerShell에서 저장소 루트 기준:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\remote_controller\install-shortcuts.ps1
```

Ubuntu 배포판과 `~/ax-play-ws` 또는 `~/ax-ws` 작업 폴더를 자동으로 찾으며 신규 바로가기의 기본 공구는 다중 흡착판 · 12컵이다. 실행값 `epick`은 호환 이름이다. 기존 개인 설정이 있으면 유지하며 리모컨에서 손가락 집게 · Robotiq 2F-85와 교체할 수 있다.
다른 PC의 폴더가 다르면 설치할 때 지정한다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\remote_controller\install-shortcuts.ps1 -Distro Ubuntu-24.04 -Workspace ~/ax-ws -Tool epick
```

Ubuntu 작업 폴더에는 빌드된 `install/setup.bash`와 리모컨 파일이 있어야 한다.
설치·빌드는 [다른 PC 실행 안내](RUN_ON_ANOTHER_PC.md)를 따른다.
Windows 쪽 저장소는 WSL에서 접근 가능한 로컬 드라이브에 둔다.

- **AX 월드 선택**: 기존 시험 월드(30×30 m), 창고(20×15 m), AWS 물류 창고 중 하나를 골라 Ubuntu 24.04 터미널·Gazebo·로봇 2대를 연다. 같은 월드는 재사용하고 다른 월드는 해당 AX 세션을 정리한 뒤 전환한다. 취소하면 현재 세션을 유지한다. 창고 선택에는 외부 자산을 포함한 최신 `ax_description` 빌드가 필요하다. 리모컨은 별도로 실행한다.
- **AX 리모컨**: 리모컨 서버와 창만 연다. Gazebo를 새로 실행하거나 종료하지 않는다. 로봇 조종에는 별도로 실행된 Gazebo가 필요하다.
- 이미 해당 작업 폴더의 세션이 있으면 재사용한다. 중복 클릭은 잠금으로 순차 처리한다.
- **AX 종료**: 해당 작업 폴더의 AX 런치·Gazebo·조종 서버·별도 리모컨 창·AX 전용 Ubuntu 터미널을 종료한다. Ubuntu 전체나 별도로 연 터미널을 종료하지 않는다.
- 기존 키보드 조종기는 리모컨과 충돌하지 않도록 실행 시 종료한다.
- 리모컨 앱 제목과 정확히 일치하는 전용 창만 닫는다. 일반 브라우저 탭으로 연 리모컨은 직접 닫는다.

설정과 실행 로그는 각각 `log/desktop_launcher/`, Ubuntu 작업 폴더의 `log/desktop_session/`에 저장한다.
개인 PC 설정·바로가기는 Git에 넣지 않는다. 저장소를 옮기면 바로가기 생성 스크립트를 다시 실행한다.
WSLg 자체에 문제가 있으면 프로세스가 실행돼도 Gazebo 창이 안 보일 수 있다.
이때 AX 화면 복구로 확인 후 WSL 전체를 재시작할 수 있다. 일반 실행·종료에서는 WSL 전체를 재시작하지 않는다.
