"""기존 1000회 기준을 유지해 A3 지연을 여러 회차 측정하고 원본·상세 결과를 저장합니다."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def main():
    """동일 시험을 별도 프로세스에서 실행해 설정과 회차 결과를 보존합니다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--temp-root', type=Path)
    parser.add_argument('--cpu-workers', type=int, default=0)
    parser.add_argument('--affinity-cores', type=int, default=0)
    arguments = parser.parse_args()
    if not 1 <= arguments.rounds <= 5 or not 0 <= arguments.cpu_workers <= 4:
        parser.error('rounds=1..5, cpu-workers=0..4 required')
    if arguments.affinity_cores < 0:
        parser.error('affinity-cores must be nonnegative')
    arguments.output = arguments.output.resolve()
    arguments.output.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ)
    if arguments.temp_root:
        arguments.temp_root.mkdir(parents=True, exist_ok=True)
        # 공용 ROS domain 잠금은 계속 기본 /tmp를 사용하고 원장 위치만 바꿉니다.
        environment['AX_A3_LEDGER_TEMP_ROOT'] = str(arguments.temp_root.resolve())
    selected_cpus = sorted(os.sched_getaffinity(0))
    if arguments.affinity_cores:
        selected_cpus = selected_cpus[:arguments.affinity_cores]
        os.sched_setaffinity(0, selected_cpus)
    workers = []
    results = []
    try:
        # 자기 시험 프로세스와 같은 CPU 집합에만 부하를 줍니다. 다른 노드 설정은 건드리지 않습니다.
        for _ in range(arguments.cpu_workers):
            workers.append(subprocess.Popen(
                [sys.executable, '-c', 'while True: pass'], stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL))
        for number in range(arguments.rounds):
            directory = arguments.output / f'round_{number + 1}'
            directory.mkdir(exist_ok=True)
            environment.update(
                AX_LOCAL_EXECUTOR_LATENCY_OUT=str(directory / 'summary.json'),
                AX_LOCAL_EXECUTOR_LATENCY_DUMP=str(directory / 'samples.json'),
                AX_A3_PROFILE_OUT=str(directory / 'profile.json'))
            with (directory / 'pytest.log').open('w') as log:
                completed = subprocess.run([
                    sys.executable, '-m', 'pytest',
                    'tools/tests/test_a3_local_task_executor.py::'
                    'LocalTaskExecutorTests::test_lease_latency_under_action_load', '-q', '-s',
                ], cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT, timeout=120)
            summary_path = directory / 'summary.json'
            summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
            result = {'round': number + 1, 'exit_code': completed.returncode, 'summary': summary}
            results.append(result)
            print(json.dumps(result), flush=True)
    finally:
        for worker in workers:
            worker.terminate()
        for worker in workers:
            worker.wait(timeout=10)
    report = {'cpu_workers': arguments.cpu_workers, 'affinity_cpus': selected_cpus,
              'temp_root': environment.get('AX_A3_LEDGER_TEMP_ROOT', '/tmp'),
              'scope': 'dev_allow·모의 Action; 실제 D4/Gazebo 운영 검증 아님', 'rounds': results}
    (arguments.output / 'runs.json').write_text(json.dumps(report, indent=2))
    return 0 if all(row['exit_code'] == 0 for row in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
