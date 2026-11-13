"""D4 W0 기본 검증 테스트: 실제 A3/로봇 연결 없이 모의 상태를 사용함.

위치: tools/tests/test_d4_w0.py
실행: python3 tools/tests/test_d4_w0.py
"""

import copy
import sqlite3
import sys
import tempfile
from contextlib import closing
from pathlib import Path
from uuid import uuid4

# 공용 테스트 폴더에서 D4 코드 위치를 찾아 불러옴
ROOT = Path(__file__).resolve().parents[2]
D4_PATH = ROOT / "src" / "ax_d4" / "ax_d4"
sys.path.insert(0, str(D4_PATH))

from common_contract import OUTCOMES, check_status, make_execution_id
from ledger import accept_status, read_latest


def make_status(revision=1, session="session-1"):
    """팀의 RobotTaskStatus.msg 형식에 맞춘 연습용 입력을 생성함."""
    root_id = "exec-" + str(uuid4())
    execution = {
        "team_id": "team_1",
        "task_id": "task-" + str(uuid4()),
        "task_revision": 1,
        "assignment_id": "asg-" + str(uuid4()),
        "assignment_generation": 1,
        "transfer_id": "",
        "root_execution_id": root_id,
        "parent_execution_id": "",
        "execution_id": "",
        "derive_rule_version": "derive-v1",
        "step_key": "load_pick_and_stow",
        "operation": "PICK_AND_STOW",
        "attempt_index": 0,
    }
    execution["execution_id"] = make_execution_id(
        root_id, execution["step_key"], execution["operation"], 0
    )
    return {
        "meta": {
            "header": {
                "stamp": {"sec": 100, "nanosec": 0},
                "frame_id": "",
            },
            "schema_version": 1,
            "source_id": "a3_executor",
            "source_session_id": session,
            "revision": revision,
            "valid": True,
            "validity_reason": "",
        },
        "robot_id": "robot_1",
        "execution": execution,
        "local_phase": "STOWING",
        "paused": False,
        "last_result": {
            "outcome": OUTCOMES["SUCCEEDED"],
            "error_code": "",
            "failure_event_id": "",
            "detail": "",
        },
    }


def send(status, db, now=100, boot=100):
    return accept_status(
        status, db, now_sim_s=now, received_boottime_s=boot
    )


def count_rows(db, table="history"):
    with closing(sqlite3.connect(db)) as conn:
        return conn.execute("SELECT COUNT(*) FROM " + table).fetchone()[0]


def run_checks():
    total = 0

    def confirm(title, func):
        nonlocal total
        try:
            func()
        except Exception as e:
            print(f"[실패] {title}: {type(e).__name__}: {e}")
            raise
        else:
            total += 1
            print(f"[통과] {title}")

    with tempfile.TemporaryDirectory(prefix="d4_w0_test_") as tmp:
        folder = Path(tmp)

        def normal():
            db = folder / "normal.db"
            status = make_status()
            check_status(status)
            assert send(status, db) == "새 상태 기록 완료"
            assert count_rows(db) == 1
            assert read_latest(db, now_sim_s=100, now_boottime_s=100)[0][1] == "CURRENT"

        confirm("정상 입력 → 한 번 저장, 현재 상태 유지", normal)

        def invalid():
            status = make_status()
            del status["robot_id"]
            try:
                check_status(status)
            except ValueError:
                return
            raise AssertionError("잘못된 입력을 통과시킴")

        confirm("잘못된 입력 → 오류 감지", invalid)

        def duplicate():
            db = folder / "duplicate.db"
            status = make_status()
            send(status, db)
            answer = send(copy.deepcopy(status), db, boot=101)
            assert answer.startswith("중복 수신"), answer
            assert count_rows(db) == 1

        confirm("동일 상태 재수신 → 중복 기록하지 않음", duplicate)

        def out_of_order():
            db = folder / "order.db"
            current = make_status(revision=3)
            send(current, db)
            old = copy.deepcopy(current)
            old["meta"]["revision"] = 2
            assert send(old, db).startswith("무시: 이전 revision")
            assert count_rows(db) == 1

        confirm("이전 버전 수신 → 무시", out_of_order)

        def conflict():
            db = folder / "conflict.db"
            status = make_status()
            send(status, db)
            changed = copy.deepcopy(status)
            changed["local_phase"] = "RELEASING"
            try:
                send(changed, db)
            except ValueError as e:
                assert "내용이 달라" in str(e), str(e)
            else:
                raise AssertionError("같은 버전의 내용 충돌을 잡지 못함")
            assert count_rows(db) == 1

        confirm("같은 버전·다른 내용 → 충돌 감지", conflict)

        def old_session():
            db = folder / "session.db"
            first = make_status(revision=1, session="session-A")
            assert send(first, db) == "새 상태 기록 완료"
            next_session = copy.deepcopy(first)
            next_session["meta"]["source_session_id"] = "session-B"
            assert send(next_session, db) == "새 상태 기록 완료"
            replay = copy.deepcopy(first)
            replay["meta"]["revision"] = 2
            assert send(replay, db).startswith("무시: 이미 교체된 이전 세션")
            assert count_rows(db) == 2

        confirm("이미 교체된 이전 세션 → 무시", old_session)

        def expired():
            db = folder / "expired.db"
            status = make_status()
            assert send(status, db, now=104).startswith("UNKNOWN")
            # 별도 DB에 정상 저장한 다음 4초 뒤 최신성 검사
            assert send(status, db, now=100) == "새 상태 기록 완료"
            state = read_latest(db, now_sim_s=104, now_boottime_s=104)[0][1]
            assert state == "UNKNOWN", state

        confirm("4초 지난 상태 → UNKNOWN", expired)

    print(f"\n결과: {total}/7개 테스트 통과")


def test_d4_w0():
    """기존 W0 검사를 바꾸지 않고 pytest가 실행할 수 있게 감싼다."""
    run_checks()


if __name__ == "__main__":
    run_checks()
