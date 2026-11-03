#!/usr/bin/env python3
# 이미 완료된 A1 시험 JSON과 로그를 docs/evidence로 모은다. 이 파일 자체가 시험을 실행하지는 않는다.
import hashlib
import json
from pathlib import Path
from evidence_files import write_test_log

root = Path(__file__).resolve().parents[1]
logs = root/'log/a1'
destination = root/'docs/evidence/a1'
payloads = [json.loads((logs/f'payload_{mass}.json').read_text()) for mass in (0.0, .5, 1.0, 2.0)]
tcp = json.loads((logs/'tcp_calibration.json').read_text())
geometry = json.loads((logs/'range_audit.json').read_text())
units = (root/'log/repair/unit_tests.log').read_text()
if not all(item['passed'] for item in payloads+[tcp,geometry]) or not units.strip().endswith('OK'):
    raise SystemExit('A1 checks have not all passed')
destination.mkdir(parents=True, exist_ok=True)
for path in [logs/'range_audit.json', logs/'tcp_calibration.json'] + [logs/f'payload_{mass}.json' for mass in (0.0,.5,1.0,2.0)]:
    (destination/path.name).write_text(path.read_text().replace(str(root),'<workspace>'))
unit_log = write_test_log(root, destination, units)
summary = {'date':'2026-11-03','passed':True,'scope':'A1 configured simulation model; fixed payload fixture',
    'asset_version':'ax1-joint-local-v2','epick_commit':'d07391307d2d16cacf88daa168ce02839589397e',
    'nominal_axis_samples':sum(len(item['samples']) for item in geometry['joints'].values()),
    'coupled_samples':geometry['coupled_configurations_sampled'],
    'blocked_coupled_cases':len(geometry['unsafe_coupled_configurations']),
    'range_endpoint_cases':sum(len(item['range_cases']) for item in payloads),
    'load_hold_cases':sum(len(item['load_cases']) for item in payloads),
    'payloads_kg':[item['payload_kg'] for item in payloads],
    'stop_cases':len(payloads),'tcp_plane_cases':len(tcp['cases']),
    'max_hold_error_rad':max(case['hold_max_error_rad'] for item in payloads for case in item['range_cases']+item['load_cases']),
    'max_stop_velocity_rad_s':max(item['stop']['max_stopped_velocity_rad_s'] for item in payloads),
    'remaining_test_processes':sum(item['cleanup']['remaining_count'] for item in payloads+[tcp]),
    'source_sha256':{}}
local_helpers = {'deploy_ubuntu_repair.py','collect_w1_evidence.py','verify_w1_runtime.py',
                 'w1_build.sh','w1_refresh.sh','w1_runtime.sh','w1_verify_all.sh','update_ubuntu_repair_docs.py'}
for directory in (root/'src/ax_description',root/'src/epick_description',root/'tools'):
    for path in sorted(directory.rglob('*')):
        if not path.is_file() or '__pycache__' in path.parts:
            continue
        if path.parent == root/'tools' and path.name in local_helpers:
            continue
        if 'meshes' in path.parts and path.name!='manifest.json':
            continue
        if path.suffix not in ('.py','.sh','.yaml','.json','.xacro','.world','.xml','.txt'):
            continue
        summary['source_sha256'][path.relative_to(root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
(destination/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(destination/'README.md').write_text(f'''# A1 모델·EPick·관절 검증

2026-11-03, Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic.

- 관절 좌표계에 맞춘 V2 메시 11개, 원본·출력·재질·관절 프레임·제한 해시 기록.
- 고정 commit EPick description, BSD-3-Clause 원본·해시 보존, 가상 진공 관절/하드웨어 드라이버 제외.
- 축별 범위 205점, 결합 자세 128개 검사. 충돌 경로는 시험에서 차단.
- 무부하 5축 양쪽 한계 직전 10개 목표 구동·복귀.
- 0·0.5·1·2 kg 관성 치구에서 3개 자세×3회: 추종·유지 총 36개.
- 각 하중에서 Action 취소 후 실제 정지·유지: 4개.
- 컵 접촉면 기준 ToF/TCP 평면 거리: 3개.

[구현·제한·재현 방법](../../guides/A1_MODEL_ASSETS.md) · [수치 요약](summary.json) · [검사 로그]({unit_log})

시험 하중은 고정 관성 치구이며 흡착 성공의 증거가 아니다. 질량·컵 구성은 가상 설정이다.
범위 검사는 단축/샘플 조건이며 전체 결합 공간·실물 정격·A4 안전 게이트의 인증이 아니다.
''')
print(json.dumps({key:value for key,value in summary.items() if key!='source_sha256'}, indent=2))
