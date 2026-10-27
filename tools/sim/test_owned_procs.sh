#!/bin/bash
# 무해한 sleep 프로세스로 owned_procs.sh / own_procs.py를 회귀 시험한다.
# 이 시험이 만드는 모든 프로세스는 (pid, start_time)으로 기록하며, 생존 확인과 정리는 그 기록만
# 쓴다(test_proc.py, pidfd). 명령줄로 pkill/pgrep 하지 않으므로 동시에 실행한 시험끼리 서로의
# 프로세스를 건드리지 않는다. 확인이 실패해도 trap이 기록한 프로세스를 모두 정지한다.
# 선택: $1 = 이번 실행의 "pid start" 기록을 받을 파일(동시 실행 시험기가 사용).
HERE=$(cd "$(dirname "$0")" && pwd)
T=$(mktemp -d "$HERE/owned_test_XXXX")
fail=0
TP="python3 $HERE/test_proc.py"
RECORDS=()

# 통과 줄을 출력한다.
pass() { echo "PASS $1"; }

# 실패 줄을 출력하고 실패로 표시한다.
bad() { echo "FAIL $1"; fail=1; }

# 사용: track VAR PID  (pid, start_time)을 RECORDS에 기록하고 VAR에 "pid start"를 넣는다.
# 반드시 현재 셸에서 실행해야 한다($(...) 안에서 실행하면 기록을 잃어 정리되지 않는다).
track() {
  local var=$1 pid=$2 s
  printf -v "$var" '%s' ''
  [ -n "$pid" ] && s=$($TP stamp "$pid") || return 1
  RECORDS+=("$pid $s")
  printf -v "$var" '%s' "$pid $s"
}

# 인자 "pid start"인 프로세스가 살아 있는지 확인한다.
alive() { [ -n "$1" ] && $TP alive $1; }

# PID 파일이 채워질 때까지 최대 5초 기다린 뒤 내용을 출력한다.
wait_pidfile() { for _ in $(seq 50); do [ -s "$1" ] && { cat "$1"; return; }; sleep 0.1; done; }

# 기록한 시험 프로세스를 모두 정지하고(정확한 pid + 시작 시각, pidfd 사용) 임시 폴더를 지운다.
test_cleanup() {
  local r
  for r in "${RECORDS[@]}"; do $TP kill $r SIGTERM; done
  sleep 0.3
  for r in "${RECORDS[@]}"; do $TP kill $r SIGKILL; done
  sleep 0.1
  [ -n "${OWN_STATE:-}" ] && rm -rf "$OWN_STATE"
  rm -rf "$T"
}

trap test_cleanup EXIT
export GZ_PARTITION=ax_owned_test

setsid sleep 501 > /dev/null 2>&1 & track DECOY $!             # 시작 전부터 있음, 같은 partition
sleep 0.2

# 빠르게 떨어져 나가는 경우의 이전 부모 PID 전용 추적기(비교용; 남긴 프로세스는 이 시험이 정리)
( source "$HERE/owned_procs_v1_ppid.sh"; own_init
  own_start "$T/v1.log" bash -c "setsid sleep 512 & echo \$! > '$T/v1.pid'; exit 0"
  sleep 1; own_cleanup ) > /dev/null 2>&1
track LEGACY "$(wait_pidfile "$T/v1.pid")"
if [ -n "$LEGACY" ] && alive "$LEGACY"; then
  echo "INFO previous parent-PID tracker missed the fast-detaching child (expected)"
else
  echo "INFO previous tracker caught the child this time (timing dependent)"
fi

source "$HERE/owned_procs.sh"
own_init
# 떨어져 나간 뒤 부모가 종료함
own_start "$T/a.log" bash -c "setsid sleep 502 & echo \$! > '$T/a.pid'; exit 0"
track DETACHED "$(wait_pidfile "$T/a.pid")"
own_start "$T/b.log" sleep 503
track ORDINARY "$OWN_LAST_PID"
own_start "$T/c.log" env -i sleep 505                                          # 토큰 없는 루트
track ENVLESS "$OWN_LAST_PID"
setsid sleep 504 > /dev/null 2>&1 & track UNRELATED $!                     # 토큰 없음, 소유 아님
sleep 0.5

# 시험 훅: 여기서 중단해 실패 후에도 EXIT trap이 정리함을 보인다
[ "${AX_OWN_TEST_ABORT:-}" = 1 ] && {
  echo "ABORT (test hook) with ${#RECORDS[@]} recorded processes"
  exit 3
}
tr '\0' '\n' < /proc/${ENVLESS% *}/environ | grep -q AX_OWN_TOKEN \
  && bad "env -i root still has the token (test premise broken)" \
  || pass "env -i root has no token (premise)"

# 부모가 이 셸이 아닌 PID(부모가 바뀐 이전 추적기의 자식)는 등록을 거부해야 한다
python3 "$HERE/own_procs.py" register "$OWN_STATE" "${LEGACY% *}" "$BASHPID" 2> /dev/null \
  && bad "register accepted a foreign PID" \
  || pass "register refused a PID that is not this shell's child"

# 낡은 기록: PID가 재사용된 것처럼 UNRELATED의 PID에 다른 시작 시각을 기록한다
echo "${UNRELATED% *} $(( ${UNRELATED#* } - 1 ))" >> "$OWN_STATE/owned.txt"
own_cleanup

alive "$DECOY" \
  && pass "pre-existing same-partition process kept" \
  || bad "pre-existing process was signalled"
alive "$ORDINARY" \
  && bad "ordinary owned process still alive" \
  || pass "ordinary owned process stopped"
alive "$ENVLESS" \
  && bad "env -i root still alive" \
  || pass "env -i root stopped (registered root)"
alive "$DETACHED" \
  && bad "fast-detaching grandchild still alive" \
  || pass "fast-detaching grandchild stopped (token)"
alive "$UNRELATED" \
  && pass "stale (pid, start_time) record did not signal the unrelated PID" \
  || bad "unrelated PID was signalled"
[ ${#RECORDS[@]} -ge 6 ] \
  && pass "all ${#RECORDS[@]} test processes recorded" \
  || bad "only ${#RECORDS[@]} test processes recorded"
[ -n "${1:-}" ] && printf '%s\n' "${RECORDS[@]}" > "$1"

# 정리 후 기록한 시험 프로세스가 남지 않았는지 확인한다.
test_cleanup
trap - EXIT
left=0
for r in "${RECORDS[@]}"; do alive "$r" && left=$((left + 1)); done
[ $left -eq 0 ] \
  && pass "no recorded test process left after cleanup" \
  || bad "$left recorded test processes left"
exit $fail
