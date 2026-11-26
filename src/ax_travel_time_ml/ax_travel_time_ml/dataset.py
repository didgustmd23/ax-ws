"""D2 ETA 표본 CSV 읽기·검사·필터·해시.

CSV 열은 D2 원장 `eta_samples` 표(`ax_fleet_manager.task_store.ETA_SAMPLE_FIELDS`)와
`tools/export_d2_eta_samples.py` 출력 헤더를 따른다. 이 모듈은 `ax_fleet_manager`를 import하지 않고
열 이름만 계약으로 둔다(어긋나면 시험 `test_eta_ml_dataset.py`가 잡는다).
"""
import csv
import hashlib
import io
import math
from collections import Counter

# 원장 eta_samples 열 순서(계약). 모두 있어야 하며 추가 열은 무시한다
EXPECTED_COLUMNS = (
    'execution_id', 'task_id', 'step_key', 'robot_id', 'role', 'attempt_index',
    'mode', 'live_kinds', 'provenance',
    'status', 'outcome', 'error_code',
    'from_location_id', 'to_location_id', 'to_location_revision', 'map_revision',
    'object_state', 'carrying_object',
    'sent_boottime_s', 'sent_sim_s', 'accepted_boottime_s', 'accepted_sim_s',
    'finished_boottime_s', 'finished_sim_s', 'duration_s', 'duration_sim_s',
    'path_length_m', 'turn_count', 'corridor_wait_s', 'obstacle_density',
)

INT_COLUMNS = frozenset({
    'attempt_index', 'outcome', 'to_location_revision', 'map_revision', 'object_state',
    'carrying_object', 'turn_count',
})
FLOAT_COLUMNS = frozenset({
    'sent_boottime_s', 'sent_sim_s', 'accepted_boottime_s', 'accepted_sim_s',
    'finished_boottime_s', 'finished_sim_s', 'duration_s', 'duration_sim_s',
    'path_length_m', 'corridor_wait_s', 'obstacle_density',
})

# 학습 목표값(sim 시간 기준 주행 소요, s)
TARGET = 'duration_sim_s'

# 기본 필터: export 기본값과 같다(혼합 개발 모드의 LIVE 출처, 성공 표본)
DEFAULT_MODE = 'DEVELOPMENT_MIXED'
DEFAULT_PROVENANCE = 'LIVE'
DEFAULT_STATUS = 'SUCCEEDED'

# 합성 표본 표시(실제 출처와 섞이지 않게 mode·provenance 모두 이 값)
SYNTHETIC = 'SYNTHETIC'


class DatasetError(Exception):
    """CSV를 읽을 수 없거나 열·값이 계약과 다르다."""


# 빈 칸은 None, 정수·실수 열은 형 변환한다. 변환 실패는 DatasetError
def _parse_value(name, text, line_no):
    if text == '':
        return None

    if name not in INT_COLUMNS and name not in FLOAT_COLUMNS:
        return text

    try:
        number = float(text)
    except ValueError as error:
        raise DatasetError(f'{line_no}행 {name} 값 해석 실패: {text!r}') from error

    if name in FLOAT_COLUMNS:
        return number

    if not math.isfinite(number) or not number.is_integer():
        raise DatasetError(f'{line_no}행 {name}은 정수여야 한다: {text!r}')

    return int(number)


# CSV 바이트를 행 사전 목록으로 바꾼다. 헤더에 계약 열이 하나라도 없으면 오류
def parse_csv_bytes(data):
    try:
        text = data.decode('utf-8')
    except UnicodeDecodeError as error:
        raise DatasetError(f'UTF-8 CSV가 아니다: {error}') from error

    reader = csv.DictReader(io.StringIO(text))
    header = reader.fieldnames or []

    missing = [name for name in EXPECTED_COLUMNS if name not in header]
    if missing:
        raise DatasetError(f'필수 열 없음: {", ".join(missing)}')

    rows = []
    for line_no, raw in enumerate(reader, start=2):
        row = {
            name: _parse_value(name, raw[name] if raw[name] is not None else '', line_no)
            for name in EXPECTED_COLUMNS
        }
        rows.append(row)

    seen = Counter(row['execution_id'] for row in rows)
    duplicated = sorted(key for key, count in seen.items() if count > 1)
    if duplicated:
        raise DatasetError(f'execution_id 중복: {", ".join(map(str, duplicated[:5]))}')

    return rows


# CSV 파일을 읽어 (행 목록, 원본 바이트 SHA-256)을 돌려준다
def read_csv(path):
    try:
        with open(path, 'rb') as stream:
            data = stream.read()
    except OSError as error:
        raise DatasetError(f'CSV 읽기 실패: {path}: {error}') from error

    return parse_csv_bytes(data), hashlib.sha256(data).hexdigest()


# 목표값이 학습에 쓸 수 있는지(유한한 양수)
def _valid_target(value):
    return isinstance(value, float) and math.isfinite(value) and value > 0.0


# 필터. (남은 행, 제외 사유별 개수)를 돌려준다. 조건 값이 None이면 그 조건을 보지 않는다
def filter_rows(rows, mode=DEFAULT_MODE, provenance=DEFAULT_PROVENANCE, status=DEFAULT_STATUS):
    kept = []
    excluded = Counter()

    for row in rows:
        if mode is not None and row['mode'] != mode:
            excluded['MODE_MISMATCH'] += 1
        elif provenance is not None and row['provenance'] != provenance:
            excluded['PROVENANCE_MISMATCH'] += 1
        elif status is not None and row['status'] != status:
            excluded['STATUS_MISMATCH'] += 1
        elif row['role'] is None or row['to_location_id'] is None:
            excluded['MISSING_KEY_FIELD'] += 1
        elif not _valid_target(row[TARGET]):
            excluded['INVALID_TARGET'] += 1
        else:
            kept.append(row)

    return kept, dict(sorted(excluded.items()))


# 값 하나를 정규화 문자열로(None은 빈 칸, 실수는 repr로 왕복 가능하게)
def _canonical(value):
    if value is None:
        return ''

    if isinstance(value, float):
        return repr(value)

    return str(value)


# 행 순서·추가 열과 무관한 정규화 SHA-256(execution_id 순, 계약 열만)
def normalized_sha256(rows):
    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator='\n')
    writer.writerow(EXPECTED_COLUMNS)

    ordered = sorted(rows, key=lambda row: str(row['execution_id']))
    for row in ordered:
        writer.writerow([_canonical(row[name]) for name in EXPECTED_COLUMNS])

    return hashlib.sha256(buffer.getvalue().encode('utf-8')).hexdigest()


# 데이터 출처 판정: 합성 표시만 있으면 synthetic, 섞였으면 mixed, 없으면 live
def data_origin(rows):
    synthetic = sum(1 for row in rows if row['provenance'] == SYNTHETIC or row['mode'] == SYNTHETIC)

    if synthetic == 0:
        return 'live'

    if synthetic == len(rows):
        return 'synthetic'

    return 'mixed'


# 행 목록을 계약 열 CSV 문자열로(합성 생성기·시험용)
def rows_to_csv_text(rows):
    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator='\n')
    writer.writerow(EXPECTED_COLUMNS)

    for row in rows:
        writer.writerow(['' if row.get(name) is None else row[name] for name in EXPECTED_COLUMNS])

    return buffer.getvalue()
