# 로봇 1·2의 작업 결과와 실행 번호가 정해진 규칙에 맞는지 확인하는 파일

# 1. 검사에 필요한 기능 준비
import hashlib
import re
from pathlib import Path
from uuid import UUID

import rfc8785

# 메시지 항목이 적혀 있는 schema 폴더 위치
SCHEMA = Path(__file__).resolve().parents[2] / "ax_interfaces" / "msg"

# 현재 A3가 사용하는 두 로봇의 번호
ROBOT_IDS = ("robot_1", "robot_2")


# 2. 메시지에 어떤 항목이 있는지 읽기
# 입력: 메시지 이름 / 결과: 항목 이름과 종류
def fields(message_type):
    name = message_type.rsplit("/", 1)[-1]
    result = {}
    for line in (SCHEMA / (name + ".msg")).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line and "=" not in line:
            field_type, field_name = line.split()
            result[field_name] = field_type
    return result


# 3. 성공·실패 등에 정해진 번호 읽기
# 입력: 메시지 이름 / 결과: 상태 이름과 번호
def constants(message_type):
    result = {}
    for line in (SCHEMA / (message_type + ".msg")).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if "=" in line:
            _, definition = line.split()
            name, value = definition.split("=")
            result[name] = int(value)
    return result


# A3에서도 사용하는 ResultStatus의 결과 번호
OUTCOMES = constants("ResultStatus")
OUTCOME_NAMES = {value: name for name, value in OUTCOMES.items()}


# 4. 받은 데이터의 형식 검사
# 입력: 받은 데이터, 메시지 이름 / 확인: 빠진 항목이나 잘못된 값
def check_message(value, message_type="RobotTaskStatus", path="status"):
    expected = fields(message_type)
   
    # 필요한 항목이 모두 들어왔는지 검사
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ValueError(path + ": 필요한 항목이 빠졌거나 다른 항목이 들어왔어요")

    for name, field_type in expected.items():
        item = value[name]
        label = path + "." + name
     
        # 글자·참거짓·숫자가 각각 맞는 형식인지 검사
        if field_type == "string":
            if type(item) is not str:
                raise ValueError(label + ": 글자를 입력해야 해요")
    
        elif field_type == "bool":
            if type(item) is not bool:
                raise ValueError(label + ": 참(True) 또는 거짓(False)으로 입력해야 해요")
    
        elif re.fullmatch(r"u?int(8|32|64)", field_type):
            bits = int(re.search(r"\d+", field_type).group())
            unsigned = field_type.startswith("u")
            low = 0 if unsigned else -(2 ** (bits - 1))
            high = 2 ** bits - 1 if unsigned else 2 ** (bits - 1) - 1
            if type(item) is not int or not low <= item <= high:
                raise ValueError(label + ": 숫자가 아니거나 허용된 범위를 벗어났어요")
        else:
            # 항목 안에 또 다른 데이터가 있으면 그 안도 검사
            check_message(item, field_type, label)
 
 
    # 시간값이 정상 범위인지 검사
    if message_type.rsplit("/", 1)[-1] == "Time":
        if value["nanosec"] >= 1_000_000_000:
            raise ValueError(path + ": 시간의 나노초 값이 너무 커요")





# 5. 작업 실행 번호 만들기
# 입력: 처음 실행 번호, 작업 단계, 동작 종류, 시도 횟수
def make_execution_id(root_execution_id, step_key, operation, attempt_index):
    # 시도 횟수가 올바른 숫자인지 검사
    if type(attempt_index) is not int or not 0 <= attempt_index < 2**32:
        raise ValueError("시도 횟수는 0 이상의 정해진 범위 안 숫자여야 해요")
 
    # 실행 번호를 만드는 데 필요한 글자가 있는지 검사
    if not all(type(v) is str and v for v in (root_execution_id, step_key, operation)):
        raise ValueError("실행 번호를 만들 때 필요한 글자 정보가 빠졌어요")
    data = dict(root_execution_id=root_execution_id, step_key=step_key,
                operation=operation, attempt_index=attempt_index)
    return hashlib.sha256(rfc8785.dumps(data)).hexdigest()[:32]


# 6. A3의 실행 결과 검사
# 입력: A3가 돌려준 결과 / 확인: 결과 항목과 상태값
def check_a3_action_result(result):
    # 이 결과 자체는 RobotTaskStatus 메시지가 아니다.
    # 결과가 항목별로 정리된 데이터인지 검사
    if not isinstance(result, dict):
        raise ValueError("A3 결과가 항목별로 정리된 형태가 아니에요")
    required = {
        "status", "object_hold_state", "load_state", "usage",
        "need_reobservation_move", "observation_reason", "observation_target",
    }


    # A3 결과에 필요한 항목이 빠지거나 더 들어왔는지 검사
    if set(result) != required:
        raise ValueError("A3 결과에 필요한 항목이 빠졌거나 다른 항목이 들어왔어요")


    # 성공·실패 결과값이 정해진 형식과 번호인지 검사
    check_message(result["status"], "ResultStatus", "result.status")
    if result["status"]["outcome"] not in OUTCOME_NAMES:
        raise ValueError("A3가 보낸 성공·실패 결과가 정해진 값이 아니에요")
 
 
    # A3가 반환하는 적재 상태: UNKNOWN=0, STOWED=1, EMPTY=2, LOST=3
    if type(result["load_state"]) is not int or result["load_state"] not in (0, 1, 2, 3):
        raise ValueError("상자가 실려진 상태가 정해진 번호가 아니에요")


    # 물체를 잡은 상태가 올바른 번호인지 검사
    if type(result["object_hold_state"]) is not int or not 0 <= result["object_hold_state"] <= 255:
        raise ValueError("물체를 잡은 상태의 번호가 올바르지 않아요")


    # 작업에 사용한 기록이 목록인지 검사
    if not isinstance(result["usage"], list):
        raise ValueError("사용 기록은 여러 항목을 담는 목록이어야 해요")
 
 
    # 다시 관찰해야 하는지와 그 이유·위치 정보 검사
    if type(result["need_reobservation_move"]) is not bool:
        raise ValueError("다시 확인하러 이동할지 참·거짓으로 표시해야 해요")
    if type(result["observation_reason"]) is not str:
        raise ValueError("다시 확인하는 이유는 글자로 적어야 해요")
    if not isinstance(result["observation_target"], dict):
        raise ValueError("다시 확인할 위치 정보가 올바른 형태가 아니에요")
    return result


# 7. 로봇 작업 상태 전체 검사
# 입력: 로봇 번호, 실행 번호, 작업 결과 등이 담긴 상태 정보
def check_status(status):
    check_message(status)
    execution = status["execution"]
 
 
    # 로봇 번호와 메시지 규칙 버전 검사
    if status["robot_id"] not in ROBOT_IDS:
        raise ValueError("로봇 번호가 robot_1 또는 robot_2가 아니에요")
    if status["meta"]["schema_version"] != 1:
        raise ValueError("메시지 규칙 버전이 현재 지원하는 버전이 아니에요")
    if execution["derive_rule_version"] != "derive-v1":
        raise ValueError("실행 번호를 만드는 규칙 버전이 달라요")
 
 
    # 작업 정보로 다시 계산한 실행 번호와 받은 번호 비교
    expected = make_execution_id(execution["root_execution_id"],
                                 execution["step_key"], execution["operation"],
                                 execution["attempt_index"])
    if execution["execution_id"] != expected:
        raise ValueError("받은 실행 번호가 계산한 번호와 달라요")
 
 
    # 작업·배정·최초 실행 번호가 정해진 모양인지 검사
    for name, prefix in [("task_id", "task-"), ("assignment_id", "asg-"),
                         ("root_execution_id", "exec-")]:
        raw = execution[name]
        if not raw.startswith(prefix):
            raise ValueError(name + ": 번호 앞에 붙는 글자가 올바르지 않아요")
        parsed = UUID(raw[len(prefix):])
        if parsed.version != 4 or str(parsed) != raw[len(prefix):]:
            raise ValueError(name + ": 번호의 고유 ID 형식이 올바르지 않아요")



    # 팀 번호 형식 검사
    if not re.fullmatch(r"team_[0-9]+", execution["team_id"]):
        raise ValueError("team_id 형태가 올바르지 않아요")


    # 마지막 작업 결과가 정해진 성공·실패 값인지 검사
    if status["last_result"]["outcome"] not in OUTCOME_NAMES:
        raise ValueError("성공·실패 결과가 정해진 값이 아니에요")
    return status
