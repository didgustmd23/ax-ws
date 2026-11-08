# A3에서 받은 로봇 1·2의 작업 상태를 검사하고 DB에 기록하는 메인 파일




# pathlib(파이썬 기본 기능)에서 파일·폴더 경로를 다루는 Path를 가져옴
from pathlib import Path
# D4(승연)가 만든 input_source.py에서 A3의 로봇 상태를 받는 기능을 가져옴
import input_source
# D4가 만든 common_contract.py에서 상태 검사 함수와 결과 번호·이름 정보를 가져옴
from common_contract import check_status, OUTCOME_NAMES
# D4가 만든 ledger.py에서 상태 저장 함수와 최신 기록 조회 함수를 가져옴
from ledger import accept_status, read_latest

# DB는 **데이터베이스(Database)**의 줄임말


def main():
    # record_file(DB 저장 위치): 이 파일과 같은 폴더에 기록을 저장함
    record_file = Path(__file__).with_name("w0_ledger.sqlite3")

    def on_status(status, *, now_sim_s, received_boottime_s):
        # 1. check_status(상태 검사): 받은 로봇 정보와 실행 번호를 검사함
        status = check_status(status)
        print("로봇 ID:", status["robot_id"])
        print("실행 번호:", status["execution"]["execution_id"])
        outcome = status["last_result"]["outcome"]
        print("실행 결과:", outcome, OUTCOME_NAMES[outcome])


        # 2. accept_status(상태 기록): 중복·이전 기록을 검사하고 DB에 저장함
        print(accept_status(
            status, record_file,
            now_sim_s=now_sim_s,
            received_boottime_s=received_boottime_s
        ))


        # 3. read_latest(최신 기록 조회): 저장된 상태가 아직 유효한지 확인함
        # DB 파일이 있을 때만 기록을 읽음
        if record_file.exists():
            for saved_status, freshness in read_latest(
                record_file,
                now_sim_s=now_sim_s,
                now_boottime_s=received_boottime_s
            ):
                print("저장된 결과:", saved_status["last_result"]["outcome"])
                print("상태 최신성:", freshness)


    # input_source.run(결과 수신 시작): 로봇 상태가 들어올 때마다 on_status를 실행함
    input_source.run(on_status)




# 이 파일을 직접 실행했을 때만 아래 main() 함수를 시작함
if __name__ == "__main__":
    main()
