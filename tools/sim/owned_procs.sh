# 격리 검증 스크립트용 소유 프로세스 추적(이 파일을 source해서 쓴다).
# 기록은 own_procs.py가 맡는다. 소유 조건: (1) own_start마다 루트 PID를 커널 시작 시각과 함께 직접
# 등록한다(명령이 환경을 비워도(env -i) 동작한다). (2) 모든 명령에 고유 AX_OWN_TOKEN을 붙이므로,
# 곧바로 떨어져 나가는 자손(새 세션·프로세스 그룹, 부모가 이미 종료)도 토큰으로 찾는다. (3) 소유
# 프로세스의 살아 있는 자손. 기록은 (pid, start_time) 쌍이고 신호는 시작 시각을 다시 확인한 뒤
# pidfd로 보내므로 재사용된 PID에는 신호가 가지 않는다. own_init 전부터 있던 프로세스는 제외한다.
OWN_HELPER="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/own_procs.py"

# 상태 폴더와 고유 토큰을 만들고, 시작 전부터 있던 프로세스를 기록한다.
own_init() {
  OWN_STATE=$(mktemp -d "${TMPDIR:-/tmp}/ax_own_XXXXXX")
  OWN_TOKEN=$(python3 -c 'import uuid; print(uuid.uuid4().hex)')
  python3 "$OWN_HELPER" init "$OWN_STATE"
}

# 사용: own_start LOGFILE CMD...  토큰을 붙인 새 세션으로 명령을 시작하고 루트 PID를 등록한다
# (OWN_LAST_PID를 설정한다).
own_start() {
  local log=$1
  shift
  # 비대화형 셸의 백그라운드 작업은 그룹 리더가 아니므로 setsid가 그 자리에서 exec하고,
  # $!는 명령 자신의 PID다(env -i도 그 자리에서 exec한다).
  AX_OWN_TOKEN="$OWN_TOKEN" setsid "$@" > "$log" 2>&1 &
  OWN_LAST_PID=$!

  # 루트를 (pid, start_time)으로 등록한다. 종료 코드 2 = 이미 종료함(자손은 토큰·부모 PID로
  # 계속 추적), 1 = PID 재사용 방지 조건으로 거부됨.
  python3 "$OWN_HELPER" register "$OWN_STATE" "$OWN_LAST_PID" "$BASHPID"
  [ $? -eq 1 ] && echo "own_start: root $OWN_LAST_PID refused" >&2
  own_snapshot
}

# 토큰을 가진 프로세스와 소유 프로세스의 자손을 소유 기록에 더한다.
own_snapshot() { python3 "$OWN_HELPER" snapshot "$OWN_STATE" "$OWN_TOKEN"; }

# 살아 있는 소유 프로세스를 'pid start cmd'로 출력한다.
own_alive() { python3 "$OWN_HELPER" alive "$OWN_STATE"; }

# 소유 프로세스를 SIGINT → SIGTERM → SIGKILL 순으로 정지하고 상태 폴더를 지운다.
own_cleanup() {
  own_snapshot
  python3 "$OWN_HELPER" signal "$OWN_STATE" SIGINT > /dev/null
  sleep 6
  own_snapshot
  python3 "$OWN_HELPER" signal "$OWN_STATE" SIGTERM > /dev/null
  sleep 3
  own_snapshot
  python3 "$OWN_HELPER" signal "$OWN_STATE" SIGKILL > /dev/null
  rm -rf "$OWN_STATE"
}
