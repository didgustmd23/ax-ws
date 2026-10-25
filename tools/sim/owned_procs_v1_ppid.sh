# 격리 검증 스크립트용 소유 프로세스 추적(이 파일을 source해서 쓴다). 이전 부모 PID 방식이다.
# `own_start`로 시작한 프로세스와 그 자손에게만 신호를 보낸다.
# 자손은 `own_snapshot`으로 반복 수집하므로, 프로세스 그룹이나 세션을 바꾸는 자식(예: ruby
# 래퍼 아래의 gz sim)도 부모가 종료한 뒤 알고 있다. `own_init` 전부터 있던 PID(예: 다른 사용자의
# gz sim)에는 신호를 보내지 않는다.
OWN_ROOTS=()
declare -A OWN_SET=()
declare -A OWN_PRE=()

# 시작 전부터 있던 PID를 기록한다.
own_init() { for p in $(ps -eo pid=); do OWN_PRE[$p]=1; done; }

# 사용: own_start LOGFILE CMD...  새 세션으로 명령을 시작하고 루트 PID를 소유로 기록한다.
own_start() {
  local log=$1
  shift
  setsid "$@" > "$log" 2>&1 &
  OWN_ROOTS+=($!)
  OWN_SET[$!]=1
}

# 이미 소유한 PID의 살아 있는 자손(부모 PID 기준)을 모두 소유에 더한다.
own_snapshot() {
  local changed=1 pid ppid
  while [ $changed -eq 1 ]; do
    changed=0
    while read -r pid ppid; do
      if [ -n "${OWN_SET[$ppid]:-}" ] && [ -z "${OWN_SET[$pid]:-}" ] &&
          [ -z "${OWN_PRE[$pid]:-}" ]; then
        OWN_SET[$pid]=1
        changed=1
      fi
    done < <(ps -eo pid=,ppid=)
  done
}

# 시작 전 PID가 아니고 살아 있는 소유 PID를 출력한다.
own_alive() {
  local p
  for p in "${!OWN_SET[@]}"; do
    [ -z "${OWN_PRE[$p]:-}" ] && kill -0 "$p" 2>/dev/null && echo "$p"
  done
}

# 소유 프로세스를 SIGINT → SIGTERM → SIGKILL 순으로 정지한다.
own_cleanup() {
  own_snapshot
  local p
  for p in $(own_alive); do kill -INT "$p" 2>/dev/null; done
  sleep 6
  own_snapshot
  for p in $(own_alive); do kill -TERM "$p" 2>/dev/null; done
  sleep 3
  for p in $(own_alive); do kill -KILL "$p" 2>/dev/null; done
}
