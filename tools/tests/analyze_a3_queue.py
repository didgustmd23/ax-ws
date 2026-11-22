"""같은 요청 ID의 서버 시각과 잠금 대기를 대조해 대기 중 추월 근거를 요약합니다."""
import argparse
import json
from pathlib import Path


def analyze(directory):
    """다른 요청이 먼저 끝난 사례를 보수적 여유 1ms로 대조합니다."""
    samples = json.loads((directory / 'samples.json').read_text())
    profile = json.loads((directory / 'profile.json').read_text())
    timing = {row[2]: (row[1], row[1] + row[0]) for row in samples['server']}
    rows = []
    for row in profile['requests']:
        if row['request_id'] not in timing:
            continue
        start, end = timing[row['request_id']]
        # 잔여 시간 전체를 잠금 전 시간으로 잡아도 뒤 요청이 추월했는지 확인한다.
        before_bound = row.get('inside_without_sql_or_ledger_wait_s', 0.0)
        rows.append(dict(request_id=row['request_id'], start=start, end=end,
                         queue_by=start + before_bound, wait=row.get('ledger_lock_wait_s', 0.0)))
    evidence = []
    for waiter in rows:
        overtakers = [other for other in rows
                      if other['start'] > waiter['queue_by'] + 0.001
                      and other['end'] < waiter['start'] + waiter['wait'] - 0.001]
        if overtakers:
            evidence.append(dict(request_id=waiter['request_id'], wait_s=waiter['wait'],
                                 overtakers=len(overtakers)))
    return dict(directory=str(directory), requests=len(rows),
                overtaken_waiters=len(evidence), overtaking_completions=sum(
                    row['overtakers'] for row in evidence),
                examples=sorted(evidence, key=lambda row: row['wait_s'], reverse=True)[:5])


def main():
    """계측된 동일 회차를 읽고 원본을 바꾸지 않은 채 근거를 출력합니다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', type=Path, nargs='+')
    args = parser.parse_args()
    print(json.dumps([analyze(path) for path in args.directories], indent=2))


if __name__ == '__main__':
    main()
