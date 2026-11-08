# A3에서 받은 로봇 1·2의 상태를 검사하고 DB에 기록하는 파일


import copy
import json
import sqlite3
from pathlib import Path

from common_contract import check_status, OUTCOMES



# daccept_status(상태 받기): status(작업 상태), database_path(DB 저장 경로)
# *(뒤의 값은 이름을 지정해 전달), now_sim_s(현재 ROS 시각), received_boottime_s(데이터를 받은 시각)
def accept_status(status, database_path, *, now_sim_s, received_boottime_s):
    # 1. check_status(상태 검사)로 데이터 형식과 받은 시간 확인함

    check_status(status)
    meta = status["meta"]
    stamp = meta["header"]["stamp"]

    stamp_s = stamp["sec"] + stamp["nanosec"] / 1_000_000_000
    if not meta["valid"] or meta["revision"] == 0:
        return "UNKNOWN: 유효한 상태가 아님"

    if not 0 <= now_sim_s - stamp_s <= 3.0:
        return "UNKNOWN: 표본 시각 확인 필요"



    # 2. stamp(기록된 시간)를 제외하고 중복 비교할 내용 준비함
    body = copy.deepcopy(status)
    body["meta"]["header"]["stamp"] = {"sec": 0, "nanosec": 0}

    body_text = json.dumps(body, ensure_ascii=False, sort_keys=True)
    payload = json.dumps(status, ensure_ascii=False)

    key = (status["robot_id"], meta["source_id"])
    session = meta["source_session_id"]

    if not session:
        raise ValueError("source_session_id가 비어 있음")
    revision = meta["revision"]



    # 3. database_path(저장 위치)에 지정된 DB 폴더가 없으면 생성함
    Path(database_path).parent.mkdir(parents=True, exist_ok=True)



    # 4. execute(명령 실행)로 DB에 기록 표를 만들고 로봇별 기존 기록 조회함
    with sqlite3.connect(database_path) as conn:
        conn.execute("CREATE TABLE IF NOT EXISTS latest (robot TEXT, source TEXT, "
                     "session TEXT, revision TEXT, body TEXT, payload TEXT, "
                     "received REAL, PRIMARY KEY(robot, source))")
        conn.execute("CREATE TABLE IF NOT EXISTS sessions (robot TEXT, source TEXT, "
                     "session TEXT, PRIMARY KEY(robot, source, session))")
        conn.execute("CREATE TABLE IF NOT EXISTS history (robot TEXT, source TEXT, "
                     "session TEXT, revision TEXT, payload TEXT, "
                     "PRIMARY KEY(robot, source, session, revision))")

        conn.execute("BEGIN IMMEDIATE")
        # ?는 빈칸이 아니라 실행할 때 key의 값으로 자동 채워짐
        old = conn.execute("SELECT session, revision, body FROM latest "
                           "WHERE robot=? AND source=?", key).fetchone()

     
     
        # 5. revision(변경 번호)과 session(연결 구간)을 비교해 중복·이전 기록 검사함
        if old and old[0] != session:
            seen = conn.execute("SELECT 1 FROM sessions WHERE robot=? AND source=? "
                                "AND session=?", (*key, session)).fetchone()
            if seen:
                return "무시: 이미 교체된 이전 세션"

        if old and old[0] == session:
            if revision < int(old[1]):
                return "무시: 이전 revision"
            if revision == int(old[1]):
                if body_text != old[2]:
                    raise ValueError("같은 세션·revision의 내용이 달라 확인 필요")
                # UPDATE(기록 수정)의 ?도 아래 전달한 값이 순서대로 들어감
                conn.execute("UPDATE latest SET payload=?, received=? "
                             "WHERE robot=? AND source=?",
                             (payload, received_boottime_s, *key))
                return "중복 수신: 기존 내용 유지, 시각만 갱신"

    
    
        # 6. INSERT(새 기록 추가)로 로봇 상태를 DB에 저장함
        # VALUES의 ?는 실행할 때 아래 괄호의 실제 값이 순서대로 들어가는 자리임
        # 직접 ?를 바꿀 필요 없으며, 로봇 정보 등이 들어오면 자동으로 저장됨
        conn.execute("INSERT OR IGNORE INTO sessions VALUES (?,?,?)", (*key, session))
        conn.execute("INSERT INTO history VALUES (?,?,?,?,?)",
                     (*key, session, str(revision), payload))
        conn.execute("INSERT OR REPLACE INTO latest VALUES (?,?,?,?,?,?,?)",
                     (*key, session, str(revision), body_text, payload,
                      received_boottime_s))


    if status["last_result"]["outcome"] in (OUTCOMES["UNSPECIFIED"], OUTCOMES["UNCERTAIN"]):
        return "기록 완료: 결과 미확정, 성공으로 처리하지 않음"
    return "새 상태 기록 완료"



def read_latest(database_path, *, now_sim_s, now_boottime_s):
    # 7. CURRENT(현재 유효)와 UNKNOWN(상태 확인 필요)으로 구분함
    with sqlite3.connect(database_path) as conn:
        rows = conn.execute("SELECT payload, received FROM latest").fetchall()


    result = []
    for payload, received in rows:
        status = json.loads(payload)
        stamp = status["meta"]["header"]["stamp"]

        age = now_sim_s - stamp["sec"] - stamp["nanosec"] / 1_000_000_000
        silence = now_boottime_s - received
        known = 0 <= age <= 3.0 and 0 <= silence <= 3.0

        result.append((status, "CURRENT" if known else "UNKNOWN"))
    return result
