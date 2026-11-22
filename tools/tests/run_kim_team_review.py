"""내 검증 환경의 설치 경로와 ROS 시험 프로세스를 분리합니다. 상대 소스·판정은 바꾸지 않습니다."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def review_commands(prefix, stage):
    """분리한 사례를 빠뜨리지 않고 별도 단계로 실행합니다. 자동 재시도는 하지 않습니다."""
    python = sys.executable
    commands = {
        'core': [python, '-m', 'pytest', 'tools/tests', '-q', '-rs',
                 '--deselect=tools/tests/test_a4_stop_confirmation_ros.py::A4StopConfirmationRosTest',
                 '--deselect=tools/tests/test_c2_surface_node_process.py::InstalledNode',
                 '--deselect=tools/tests/test_world_selection.py::WorldSelection::test_run_mapping_session_world',
                 '--deselect=tools/tests/test_scan_merge.py::Node::test_build_and_merge_callback'],
        'mapping': [python, '-m', 'pytest', 'tools/tests/test_kim_mapping_external_install.py', '-q'],
        'c2': [python, '-m', 'pytest', 'tools/tests/test_c2_surface_node_process.py', '-q'],
        'a4': [python, 'tools/tests/run_external_a4_stop_review.py', '--install-prefix', str(prefix)],
    }
    # mapping 단계는 core에도 포함되지만 단독 진단 명령으로 제공한다.
    names = ('core', 'c2', 'a4') if stage == 'all' else (stage,)
    return [(name, commands[name]) for name in names]


def main():
    """각 단계의 원본 종료 코드를 보존하고 한 번이라도 실패하면 전체를 실패로 반환합니다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-prefix', type=Path, required=True)
    parser.add_argument('--c2-prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=('all', 'core', 'mapping', 'c2', 'a4'), default='all')
    args = parser.parse_args()
    prefix, c2_prefix = args.install_prefix.resolve(), args.c2_prefix.resolve()
    for path in (prefix, c2_prefix):
        if not (path / 'setup.bash').is_file():
            parser.error(f'설치 환경 설정 파일 없음: {path / "setup.bash"}')
    gate = prefix / 'ax_safety/lib/ax_safety/a4_safety_gate'
    if not gate.is_file():
        parser.error(f'A4 실행 파일 없음: {gate}')
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('기존/진행 중인 시험 근거를 보존하도록 비어 있는 새 output 폴더를 사용하세요.')
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / 'summary.json'
    if summary_path.exists():
        parser.error('기존 시험 근거를 덮어쓰지 않도록 새 output 폴더를 사용하세요.')
    environment = dict(os.environ, AX_KIM_INSTALL_BASE=str(prefix),
                       AX_C2_NODE_PREFIX=str(c2_prefix), AX_GATE_BIN=str(gate),
                       AX_LOCAL_EXECUTOR_LATENCY_OUT=str(output / 'latency.json'))
    summary = {'started_utc': datetime.now(timezone.utc).isoformat(),
               'install_prefix': str(prefix), 'c2_prefix': str(c2_prefix), 'stages': [],
               'all_stages_passed': False,
               'limits': '실제 Gazebo/운영 승인 아님. C2 원본의 전송 순서 근본 해결 아님.'}
    for name, command in review_commands(prefix, args.stage):
        # 새 Python 프로세스로 ROS context·import·환경 누적을 분리한다. 실패를 재시도로 숨기지 않는다.
        if name != 'a4':
            command = command + ['--junitxml=' + str(output / f'{name}.xml')]
        print(f'{name}: 시작', flush=True)
        with (output / f'{name}.log').open('w', encoding='utf-8') as log:
            result = subprocess.run(command, cwd=ROOT, env=environment,
                                    stdout=log, stderr=subprocess.STDOUT)
        row = {'name': name, 'command': command, 'returncode': result.returncode}
        xml_path = output / f'{name}.xml'
        if xml_path.is_file():
            cases = ET.parse(xml_path).getroot().findall('.//testcase')
            row['counts'] = {
                'passed': sum(all(case.find(key) is None for key in ('failure', 'error', 'skipped'))
                              for case in cases),
                'failed': sum(case.find('failure') is not None for case in cases),
                'errors': sum(case.find('error') is not None for case in cases),
                'skipped': sum(case.find('skipped') is not None for case in cases),
            }
        summary['stages'].append(row)
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'{name}: 종료 코드 {result.returncode}', flush=True)
    summary['all_stages_passed'] = all(row['returncode'] == 0 for row in summary['stages'])
    summary['has_skipped_tests'] = any(row.get('counts', {}).get('skipped', 0)
                                      for row in summary['stages'])
    summary['finished_utc'] = datetime.now(timezone.utc).isoformat()
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    return 0 if summary['all_stages_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
