#!/usr/bin/env python3
"""D2 원장의 ETA 주행 표본을 CSV로 내보낸다(원장은 읽기 전용으로 연다, ROS 불필요).

- 기본: mode=DEVELOPMENT_MIXED, provenance=LIVE, status=SUCCEEDED 표본만 내보낸다(학습 표본).
  MOCK 표본은 학습에 쓰지 않는다.
- --include-mock: 모드·출처 필터를 끈다(개발 확인용, 출처 NULL 포함).
- --include-failed: 실패·취소·시간초과·미확정 표본도 넣는다.
- 항상 제외: 종료 시각(finished_boottime_s)이 NULL이거나 duration_s·duration_sim_s가 음수·비유한인 행.
- 첫 줄은 열 이름이다. 단위는 시각·시간 s, 길이 m(README 참고). 결측 열은 빈 칸이다.

실행: python3 tools/export_d2_eta_samples.py <원장.sqlite> [-o 출력.csv]
      [--include-mock] [--include-failed]
"""
import argparse
import csv
import math
import sqlite3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))

from ax_fleet_manager.task_store import ETA_SAMPLE_FIELDS  # noqa: E402

MIN_SCHEMA_VERSION = 4
TRAINING_MODE = 'DEVELOPMENT_MIXED'
TRAINING_PROVENANCE = 'LIVE'
TRAINING_STATUS = 'SUCCEEDED'


class ExportError(Exception):
    """원장을 읽을 수 없거나 표본 표가 없다."""


# 원장을 읽기 전용 URI로 열고 schema 버전을 확인한다
def open_read_only(path):
    if not Path(path).is_file():
        raise ExportError(f'원장 파일 없음: {path}')

    uri = Path(path).resolve().as_uri() + '?mode=ro'
    db = sqlite3.connect(uri, uri=True)

    try:
        row = db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()
    except sqlite3.Error as error:
        db.close()
        raise ExportError(f'원장 meta 읽기 실패: {error}') from error

    if row is None or not row[0].isdigit() or int(row[0]) < MIN_SCHEMA_VERSION:
        db.close()
        raise ExportError(f'schema {MIN_SCHEMA_VERSION} 이상 원장만 내보낸다(현재 {row})')

    return db


# 시간 값이 쓸 수 있는지(NULL은 허용하지 않는 쪽은 호출자가 정한다): 유한한 0 이상
def _valid_duration(value):
    return isinstance(value, (int, float)) and math.isfinite(value) and value >= 0.0


# 내보낼 행인지: 종료 시각이 있고 시간 값이 유효하며 필터 조건을 만족
def keep(row, include_mock=False, include_failed=False):
    if row['finished_boottime_s'] is None or not _valid_duration(row['duration_s']):
        return False

    if row['duration_sim_s'] is not None and not _valid_duration(row['duration_sim_s']):
        return False

    if not include_mock and (row['mode'] != TRAINING_MODE
                             or row['provenance'] != TRAINING_PROVENANCE):
        return False

    if not include_failed and row['status'] != TRAINING_STATUS:
        return False

    return True


# 원장의 표본 행을 필터해 돌려준다(실행 ID 순)
def load_rows(path, include_mock=False, include_failed=False):
    db = open_read_only(path)

    try:
        cursor = db.execute(
            f'SELECT {", ".join(ETA_SAMPLE_FIELDS)} FROM eta_samples ORDER BY execution_id')
        rows = [dict(zip(ETA_SAMPLE_FIELDS, values)) for values in cursor.fetchall()]
    except sqlite3.Error as error:
        raise ExportError(f'표본 읽기 실패: {error}') from error
    finally:
        db.close()

    return [row for row in rows if keep(row, include_mock, include_failed)]


# CSV 쓰기: 첫 줄 열 이름, NULL은 빈 칸
def write_csv(rows, stream):
    writer = csv.writer(stream, lineterminator='\n')
    writer.writerow(ETA_SAMPLE_FIELDS)

    for row in rows:
        writer.writerow(['' if row[name] is None else row[name] for name in ETA_SAMPLE_FIELDS])


# 진입점. 오류는 종료 코드 2
def main(argv=None):
    parser = argparse.ArgumentParser(description='D2 ETA 주행 표본 CSV 내보내기(읽기 전용)')
    parser.add_argument('store', help='D2 원장 SQLite 경로')
    parser.add_argument('-o', '--output', help='출력 CSV 경로(없으면 표준 출력)')
    parser.add_argument('--include-mock', action='store_true',
                        help='모드·출처 필터 해제(개발 확인용, 학습 표본 아님)')
    parser.add_argument('--include-failed', action='store_true',
                        help='SUCCEEDED 외 종료 상태도 포함')
    args = parser.parse_args(argv)

    try:
        rows = load_rows(args.store, args.include_mock, args.include_failed)
    except ExportError as error:
        print(f'[export_d2_eta_samples] {error}', file=sys.stderr)
        return 2

    if args.output:
        with open(args.output, 'w', encoding='utf-8', newline='') as stream:
            write_csv(rows, stream)
    else:
        write_csv(rows, sys.stdout)

    print(f'[export_d2_eta_samples] {len(rows)}행', file=sys.stderr)

    return 0


if __name__ == '__main__':
    sys.exit(main())
