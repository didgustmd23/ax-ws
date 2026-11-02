#!/usr/bin/env python3
"""Update the existing README/schedule with the verified A1 scope only."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
readme = root/'README.md'
text = readme.read_text(encoding='utf-8-sig')
text = re.sub(r'\n<!-- robot-drive-repair -->.*?<!-- /robot-drive-repair -->\n', '\n', text, flags=re.S)
state = ('현재 작업트리에는 **A1 모델·센서·시험 구성**이 구현돼 있다. AX_1 V2 관절 로컬 메시, '
         '질량·관성 생성기, 고정 commit EPick description, AX_2 적재함 접촉 센서, 로봇별 namespace·TF와 공통 시험 launch를 제공한다. '
         '관절 범위·자기 충돌·0~2 kg 관성 치구·유지·Action 취소 정지와 기하 TCP 보정을 검증했다. '
         '[A1 구현·제한·재현](docs/guides/A1_MODEL_ASSETS.md) · [검증 결과](docs/evidence/a1/). '
         '실제 흡착 부착/해제·B1 보정/동기화·Nav2·Agent·A4 안전 게이트와 G1 관문은 별도 구현·검증 대상이다.')
text = re.sub(r'현재 (?:저장소는|작업트리에는).*?(?=\n\n)', state, text, count=1, flags=re.S)
if '<!-- a1-model-entry -->' not in text:
    section = '''<!-- a1-model-entry -->
## A1 모델 실행·검증

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select epick_description ax_description ax_interfaces robotiq_description
source install/setup.bash
ros2 launch ax_description robot_test.launch.py
```

기본 공구는 EPick이며 `tool:=suction`으로 기존 가상 4컵 헤드를 비교할 수 있다.
원본 CAD·라이선스를 보존하고 [A1 안내](docs/guides/A1_MODEL_ASSETS.md)의 생성·시험 명령을 따른다.
[바퀴·시험 월드·런치](docs/guides/WHEEL_AND_LAUNCH_REPAIR.md) · [A1 시험 증거](docs/evidence/a1/).

'''
    text = text.replace('## 전체 시스템 구성도', section + '## 전체 시스템 구성도', 1)
readme.write_text(text, encoding='utf-8')
schedule = root/'docs/management/TEAM_SCHEDULE.md'
text = schedule.read_text(encoding='utf-8-sig')
text = text.replace('A1 최종 EPick·하중·기구 검증은 계속 필요하다.', 'A1 추가 검증 결과는 아래 A1 상태를 따른다.')
if '<!-- a1-verified-status -->' not in text:
    status = '''<!-- a1-verified-status -->
## A1 모델 산출물 검증 상태 — 2026-11-03

- AX_1 V2 메시 11개: 기존 CAD 표면·면 연결 보존, 관절 로컬 원점·manifest·충돌/관성·제한 동기화.
- EPick description 선별 도입: 고정 commit·BSD-3-Clause·원본/수정 해시, 가상 진공 관절·드라이버 제외, Jazzy 빌드.
- 범위 205점·결합 자세 128개 검사, 무부하 양단 직전 10개 목표, 0·0.5·1·2 kg 치구의 36개 추종/유지와 4개 취소 정지.
- 기하 TCP/ToF 평면 거리 3조건 검증. 제품 질량·최종 컵·실물 보정은 미확정으로 구분.
- 범위·명령·수치는 [A1 안내](../guides/A1_MODEL_ASSETS.md), 실제 결과는 [A1 증거](../evidence/a1/)를 따른다.
  시험용 충돌 선별·고정 하중 결과를 A2 흡착 성공이나 A4 런타임 안전 게이트·G1 통과로 해석하지 않는다.

'''
    text = text.replace('## 완료 관문', status + '## 완료 관문', 1)
schedule.write_text(text, encoding='utf-8')
index = root/'docs/README.md'
text = index.read_text(encoding='utf-8-sig')
text = re.sub(r'- 저장소는 \*\*설계·일정 단계\*\*다\..*?(?=\n)', '- A1 모델·센서·시험 구현이 추가됐다. [A1 안내](guides/A1_MODEL_ASSETS.md)와 [검증 증거](evidence/a1/)를 따르며 G1·운반·안전·Agent 완료와 구분한다.', text, count=1)
if '## A1 모델 안내' not in text:
    text = text.replace('## 전체 시스템 구성도', '## A1 모델 안내\n\n[A1 메시·EPick·관절 검증](guides/A1_MODEL_ASSETS.md) · [바퀴·런치](guides/WHEEL_AND_LAUNCH_REPAIR.md) · [증거](evidence/a1/).\n\n## 전체 시스템 구성도', 1)
index.write_text(text, encoding='utf-8')
print('README and existing A1 schedule updated')
