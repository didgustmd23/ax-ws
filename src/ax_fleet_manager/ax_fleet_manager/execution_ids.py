"""D-03 실행 ID 규칙(derive-v1)의 최소 구현.

규칙: 키 정렬·공백 없는 UTF-8 JSON {attempt_index, operation, root_execution_id, step_key}
(RFC 8785 JCS)의 SHA-256 소문자 16진수 앞 32자.

ax_d4.common_contract.make_execution_id와 같은 결과를 내야 한다. 그 함수를 직접 쓰지 않는 이유:
ax_d4는 rfc8785 외부 모듈을 필요로 하지만 package.xml에 rosdep 키가 없고 ROS 시스템
python에는 설치돼 있지 않다. 입력이 ASCII 고정 키·문자열 3개·uint32 정수 하나뿐이므로
표준 json으로 JCS와 같은 바이트를 만들 수 있다(시험에서 기존 함수와 고정 벡터를 대조).
"""
import hashlib
import json

DERIVE_RULE_VERSION = 'derive-v1'


# 문자열이 JCS와 표준 json 직렬화 결과가 같아지는 범위인지 확인한다
def _check_text(name, value):
    if type(value) is not str or not value:
        raise ValueError(f'{name}는 비어 있지 않은 문자열이어야 한다')

    # 짝 없는 surrogate는 UTF-8로 만들 수 없어 JCS도 거부한다
    try:
        value.encode('utf-8')
    except UnicodeEncodeError as error:
        raise ValueError(f'{name}에 UTF-8로 바꿀 수 없는 글자가 있다') from error


# root_execution_id·step_key·operation·attempt_index로 하위 execution_id를 만든다
def make_execution_id(root_execution_id, step_key, operation, attempt_index):
    if type(attempt_index) is not int or not 0 <= attempt_index < 2**32:
        raise ValueError('attempt_index는 0 이상 2^32 미만 정수여야 한다')

    for name, value in (('root_execution_id', root_execution_id),
                        ('step_key', step_key),
                        ('operation', operation)):
        _check_text(name, value)

    # JCS: 키 정렬, 구분자 공백 없음, 비ASCII 원문 유지, 제어 문자는 json 표준 escape
    data = {
        'attempt_index': attempt_index,
        'operation': operation,
        'root_execution_id': root_execution_id,
        'step_key': step_key,
    }
    encoded = json.dumps(data, sort_keys=True, separators=(',', ':'),
                         ensure_ascii=False).encode('utf-8')

    return hashlib.sha256(encoded).hexdigest()[:32]
