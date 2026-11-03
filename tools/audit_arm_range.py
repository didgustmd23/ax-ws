#!/usr/bin/env python3
# 팔 5축의 단축 범위와 시드 고정 결합 자세를 검사하고 log/a1에 결과를 저장한다.
import json
from pathlib import Path
import numpy as np
from arm_geometry import ArmGeometry, ARM_JOINTS

WORKSPACE = Path(__file__).resolve().parents[1]

model = ArmGeometry()
report = {'scope': 'conservative independent-axis sweep at neutral; coupled paths require screening',
          'home_collisions': model.collisions([0]*5), 'joints': {}}
for index, joint in enumerate(ARM_JOINTS):
    low, high = model.limits[joint]
    cases = []
    for value in np.linspace(low, high, 41):
        positions = [0.0]*5
        positions[index] = float(value)
        collisions = model.collisions(positions)
        cases.append({'position_rad': float(value), 'collisions': collisions})
    report['joints'][joint] = {'nominal_limits_rad': [low, high], 'samples': cases,
                              'collision_free_samples': sum(not case['collisions'] for case in cases)}
    print(joint, report['joints'][joint]['collision_free_samples'], '/', len(cases), flush=True)
report['passed'] = not report['home_collisions']
rng = np.random.default_rng(20261103)
coupled = []
for _ in range(128):
    positions = [float(rng.uniform(*model.limits[joint])) for joint in ARM_JOINTS]
    collisions = model.collisions(positions)
    if collisions:
        coupled.append({'positions_rad': positions, 'collisions': collisions,
                        'screened_path': model.screen_path([0]*5, positions)})
report['coupled_configurations_sampled'] = 128
report['unsafe_coupled_configurations'] = coupled
report['passed'] = report['passed'] and all(not case['screened_path']['allowed'] for case in coupled)
output = WORKSPACE / 'log/a1/range_audit.json'
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(report, indent=2) + '\n')
print('home collisions:', report['home_collisions'])
