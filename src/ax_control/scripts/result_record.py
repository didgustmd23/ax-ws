"""A2 시험 도구 공용 결과 기록. 중단돼도 result.json이 깨지지 않게 저장한다.

시험 기록 형식(status·error 등)은 각 도구가 정한다. 이 모듈은 파일 쓰기만 맡는다.
"""

import json
import os


# 임시 파일에 쓴 뒤 바꿔 넣는다. 쓰는 도중 중단돼도 이전 파일이 깨지지 않는다.
def write_json_atomic(path, data):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, default=float) + "\n", encoding="utf-8")

    os.replace(temporary, path)
