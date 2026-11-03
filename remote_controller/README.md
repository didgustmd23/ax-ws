# AX 실행·리모컨

**이 폴더의 `install.cmd`를 더블클릭하면 바탕화면에 바로가기 3개가 생성됩니다.**
Git 다운로드만으로 바로가기가 자동 생성되지는 않습니다. 내려받은 뒤 설치 파일을 한 번 실행하세요.

| 파일 / 바탕화면 바로가기 | 동작 |
|---|---|
| `install.cmd` | 바탕화면 바로가기 생성·갱신 |
| `start.cmd` / **AX 실행** | Ubuntu 24.04 터미널 + Gazebo + 로봇 두 대 실행 |
| `remote_controller.cmd` / **AX 리모컨** | 리모컨 서버와 별도 창만 실행 |
| `stop.cmd` / **AX 종료** | 해당 로봇 세션·Gazebo·리모컨 서버·리모컨 전용 창 종료 |

`start.cmd`, `remote_controller.cmd`, `stop.cmd`도 바로 사용할 수 있습니다.
설정 파일이 없으면 첫 실행 때 바로가기 생성도 함께 수행합니다.
파일 위치를 기준으로 경로를 계산하므로 다른 사용자 이름·다운로드 위치에서도 사용할 수 있습니다.
**이 폴더만 떼어내지 말고 저장소 전체를 내려받으세요.** ROS 연동용 내부 코드는 `tools/`를 사용합니다.

## 처음 받는 사람

1. 저장소의 `su` 브랜치를 Windows의 로컬 폴더에 내려받습니다.
2. Ubuntu 24.04 WSL2, ROS 2 Jazzy와 Ubuntu 작업공간을 준비하고 최초 빌드를 완료합니다.
   [다른 PC 설치·빌드 안내](../docs/guides/RUN_ON_ANOTHER_PC.md)를 따르세요.
3. 이 폴더의 **`install.cmd`**를 더블클릭합니다.
4. 바탕화면의 **AX 실행**을 더블클릭합니다.

설치기는 Ubuntu 배포판과 `~/ax-play-ws`, `~/ax-ws` 작업 폴더를 자동으로 찾습니다.
여러 폴더가 있으면 `ax-play-ws`를 우선합니다. 기존 설정이 있으면 해당 배포판·작업 폴더를 유지합니다.
다른 이름의 작업 폴더를 사용한다면 이 폴더에서 Windows PowerShell을 열어 지정하세요.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\install-shortcuts.ps1 -Distro Ubuntu-24.04 -Workspace ~/my-ax-ws
```

자동 감지는 작업 폴더를 찾는 기능이며, Ubuntu·ROS 설치나 빌드를 대신하지 않습니다.
처음 실행할 때 Ubuntu 시작과 로봇 생성에 시간이 걸릴 수 있으니 기다리세요.
이미 실행 중인 세션은 재사용하며, 중복 클릭으로 로봇을 추가 생성하지 않습니다.
실행 중 PowerShell 창이 잠깐 보일 수 있지만 작업이 끝나면 자동으로 닫힙니다.
AX 종료는 AX 전용 Ubuntu 터미널도 함께 종료합니다. 별도로 연 다른 터미널은 유지합니다.
**AX 실행은 리모컨을 열지 않습니다.** 리모컨이 필요할 때 **AX 리모컨**을 별도로 실행하세요.

## 파일 정리

- 사용자용 진입점: 이 폴더의 `.cmd` 파일 네 개.
- Windows 실행·설치 처리: `ax-launch.ps1`, `install-shortcuts.ps1`.
- ROS 세션·리모컨 내부 구현: 저장소의 `tools/`.
- 개인 설정: `log/desktop_launcher/config.json` — Git 제외.
- 실행 로그: Ubuntu 작업공간의 `log/desktop_session/` — Git 제외.
- 바탕화면 `.lnk`는 각 PC에서 생성하며 Git에 넣지 않습니다.

저장소 폴더를 옮기면 `install.cmd`를 다시 실행해 바로가기를 갱신하세요.
조작법은 [리모컨 안내](../docs/guides/REMOTE_CONTROL.md), 상세 설정은
[바탕화면 실행 안내](../docs/guides/DESKTOP_LAUNCHER.md)를 참고하세요.
