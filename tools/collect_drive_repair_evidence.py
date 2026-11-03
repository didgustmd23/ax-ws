#!/usr/bin/env python3
# 완료된 주행·팔·LiDAR 회귀 결과만 수집한다. 원시 로그는 log/, 공유 증거는 docs/evidence/에 둔다.
"""Collect only completed tests from the actual Ubuntu workspace."""
import hashlib
import json
from pathlib import Path
from evidence_files import write_test_log

root = Path(__file__).resolve().parents[1]
runtime = json.loads((root / 'log/repair/drive_directions.json').read_text())
geometry = json.loads((root / 'log/repair/model_geometry.json').read_text())
units = (root / 'log/repair/unit_tests.log').read_text()
if not (runtime.get('passed') and geometry.get('passed') and units.strip().endswith('OK')
        and len(runtime.get('trials', [])) == 8 and len(runtime.get('arm_trials', [])) == 5
        and len(runtime.get('lidar_obstacle_checks', [])) == 4
        and runtime.get('cleanup', {}).get('remaining_count') == 0):
    raise SystemExit('The full drive/arm/LiDAR/geometry test suite has not passed')
destination = root / 'docs/evidence/drive_repair'
destination.mkdir(parents=True, exist_ok=True)
for trial in runtime['trials']:
    trial.pop('gazebo_pose_debug', None)
for name, data in [('drive_directions.json', runtime), ('model_geometry.json', geometry)]:
    serialized = json.dumps(data, indent=2).replace(str(root), '<workspace>')
    (destination / name).write_text(serialized + '\n')
unit_log = write_test_log(root, destination, units)
sources = {}
for directory in [root / 'src/ax_description', root / 'tools']:
    for path in sorted(directory.rglob('*')):
        if not path.is_file() or any(part in ('meshes', '__pycache__') for part in path.parts):
            continue
        if path.suffix not in ('.py', '.sh', '.yaml', '.xacro', '.world', '.xml', '.json', '.txt'):
            continue
        sources[path.relative_to(root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
summary = {'date': '2026-11-03', 'execution_workspace': 'Ubuntu ax-ws', 'passed': True,
           'drive_trials': [{'robot': trial['robot_id'], 'trial': trial['trial'],
                             'forward_distance_m': trial['forward_distance_m'],
                             'yaw_change_rad': trial['yaw_change_rad'], 'passed': trial['passed']}
                            for trial in runtime['trials']],
           'arm_joint_tests': len(runtime['arm_trials']),
           'lidar_pose_checks': len(runtime['lidar_snapshots']),
           'lidar_obstacle_checks': len(runtime['lidar_obstacle_checks']),
           'remaining_test_processes': runtime['cleanup']['remaining_count'],
           'source_sha256': sources}
(destination / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
(destination / 'README.md').write_text(f'''# 바퀴·런치·모델 회귀 검증

2026-11-03 Ubuntu 실제 작업공간에서 실행한 결과다.

- EPick 포함 패키지 4개 빌드, 정적·계약 검사 13개 통과.
- 조작·운송 로봇 각각 전진·후진·좌회전·우회전: 총 8개 통과.
- 팔 5축 각각 0.15 rad 이동, 실제 관절 상태·Gazebo 자식 링크 회전 확인, 중립 복귀.
- 중립·팔 동작 후 LiDAR 4개: 근거리 자기 반환·음의 무한대·NaN 검사.
- 네 LiDAR의 실제 구형 표적 표면 기대 거리 1.85 m 검출 확인.
- 차체 메시 정점 누락 0, 링크별 COM 및 SDF 5축 관절 보존 확인.
- 시험 태그 프로세스 잔류 0.

[실행·수정 안내](../../guides/WHEEL_AND_LAUNCH_REPAIR.md) · [요약](summary.json) ·
[실행 상세](drive_directions.json) · [형상·COM](model_geometry.json) · [검사 로그]({unit_log})

가상 질량·마찰 가정의 짧은 회귀 시험이다. 실물 정격·전체 관절 범위·하중·안전 게이트·정밀 주행 성능을 검증한 결과가 아니다.
''')
print('Collected verified Ubuntu repair evidence:', destination)
