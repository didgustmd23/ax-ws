#!/usr/bin/env bash
# B2 세션 프로세스 관리(run_mapping_session.sh가 source한다). Gazebo 없이 단독 시험할 수 있게 분리했다.
#
# - session_start NAME LOG CMD...: CMD를 setsid로 새 프로세스 그룹에 띄우고 그룹 대표 PID를 SESSION_PIDS에 넣는다.
# - session_owned: 소유 표식(환경 변수 AX_SESSION_OWNER=값)을 가진 현재 사용자 프로세스 PID 목록(호출 셸 제외).
# - session_cleanup WAIT_S: 그룹마다 SIGINT → WAIT_S 초 대기 → 남은 소유 프로세스 SIGKILL → 잔여 수를 SESSION_REMAINING에 둔다.
#
# 소유 표식은 session_start 전에 export해야 자식 프로세스 환경에 들어간다(셸 자신의 /proc environ에는 들어가지 않는다).

SESSION_PIDS=()
SESSION_REMAINING=
SESSION_REMAINING_PIDS=

# CMD를 새 프로세스 그룹으로 띄운다. 표준 입력은 막고 출력은 LOG 파일에 남긴다.
# 비대화형 셸의 `&` 자식은 SIGINT·SIGQUIT를 무시한 채 시작하므로, 정리 때 보내는 SIGINT로 정지 절차를 밟을 수 있게
# 기본 동작으로 되돌린 뒤 실행한다(env --default-signal).
session_start() {
  local name="$1" log="$2"; shift 2
  setsid env --default-signal=INT,QUIT "$@" > "$log" 2>&1 < /dev/null &
  SESSION_PIDS+=("$!")
  eval "SESSION_PID_${name}=$!"
}

# /proc/<pid>/stat의 세션 ID. 읽지 못하면 빈 문자열.
session_sid() {
  local stat
  stat="$(cat "/proc/$1/stat" 2>/dev/null)" || return 0
  stat="${stat##*) }"
  # shellcheck disable=SC2086
  set -- $stat
  echo "${4:-}"
}

# /proc/<pid>/stat의 부모 PID. 읽지 못하면 빈 문자열.
session_ppid() {
  local stat
  stat="$(cat "/proc/$1/stat" 2>/dev/null)" || return 0
  stat="${stat##*) }"
  # shellcheck disable=SC2086
  set -- $stat
  echo "${2:-}"
}

# 살아 있는(좀비가 아닌) 프로세스면 0.
session_alive() {
  local stat
  stat="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
  stat="${stat##*) }"
  [ "${stat%% *}" != Z ]
}

# pid가 이 셸($$)의 실제 자손이면 0. 부모를 따라 올라가며 확인한다(최대 64단계, init에서 멈춤).
session_is_descendant() {
  local pid="$1" depth=0

  while [ -n "$pid" ] && [ "$pid" -gt 1 ] && [ "$depth" -lt 64 ]; do
    pid="$(session_ppid "$pid")"
    [ "$pid" = "$$" ] && return 0
    depth=$((depth + 1))
  done

  return 1
}

# 소유 표식을 가진 같은 사용자 프로세스를 찾는다. 다른 사용자·권한 없는 /proc는 건너뛴다.
# 소유 판단은 표식(AX_SESSION_OWNER 값 일치)으로만 한다. 부모 관계는 소유 근거로 쓰지 않고,
# 정리 명령 자신의 보조 명령(tr·grep·wc·명령 치환 subshell 등: 이 셸과 같은 SID이면서 이 셸의 실제 자손)을
# 빼는 데에만 쓴다. 그래서 같은 SID에 남은 고아(부모가 바뀐 ros2 daemon 등)와 다른 세션으로 빠진 자손은 모두 찾는다.
session_owned() {
  [ -n "${AX_SESSION_OWNER:-}" ] || return 0
  local pid environ own_sid
  own_sid="$(session_sid "$$")"

  for environ in /proc/[0-9]*/environ; do
    pid="${environ#/proc/}"; pid="${pid%/environ}"
    [ "$pid" = "$$" ] && continue
    [ -O "$environ" ] || continue
    { tr '\0' '\n' < "$environ"; } 2>/dev/null | grep -qx "AX_SESSION_OWNER=$AX_SESSION_OWNER" || continue
    [ -d "/proc/$pid" ] || continue

    if [ "$(session_sid "$pid")" = "$own_sid" ] && session_is_descendant "$pid"; then
      continue
    fi

    echo "$pid"
  done
}

# 그룹 대표 PID에 SIGINT를 보내고 대표 프로세스가 끝날 때까지 최대 WAIT_S 초 기다린다(정지 절차 우선).
# 끝났으면 0, 시간 안에 끝나지 않았으면 1을 돌려준다(이후 session_cleanup이 강제 정리한다).
session_stop_group() {
  local pid="$1" wait_s="${2:-90}" deadline
  kill -0 "$pid" 2>/dev/null || return 0
  kill -INT -- "-$pid" 2>/dev/null
  deadline=$((SECONDS + wait_s))

  while kill -0 "$pid" 2>/dev/null; do
    [ "$SECONDS" -ge "$deadline" ] && return 1
    sleep 0.2
  done

  return 0
}

# 띄운 그룹을 정리하고 소유 프로세스 잔여 수를 남긴다.
session_cleanup() {
  local wait_s="${1:-5}" pid deadline

  for pid in "${SESSION_PIDS[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null
  done

  # 정지 절차가 끝나도록 기다리되, 소유 프로세스가 모두 끝나면 바로 넘어간다.
  deadline=$((SECONDS + wait_s))

  while [ "$SECONDS" -lt "$deadline" ] && [ -n "$(session_owned)" ]; do
    sleep 0.5
  done

  # 남은 그룹을 강제로 끝내고, 그룹 밖으로 빠졌거나 정리 중 새로 생긴 소유 프로세스가 없어질 때까지 반복한다.
  for pid in "${SESSION_PIDS[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null
  done

  local pass

  for pass in 1 2 3 4 5; do
    local owned
    owned="$(session_owned)"
    [ -z "$owned" ] && break

    for pid in $owned; do
      kill -KILL "$pid" 2>/dev/null
    done

    sleep 0.3
  done

  # 이 셸의 백그라운드 작업(같은 SID 자손이라 표식 검색에서 빠지는 것: 예 로그 tail)도 정리하고 잔여에 넣는다.
  for pid in $(jobs -p); do
    kill -KILL "$pid" 2>/dev/null
  done

  sleep 0.2
  local remaining
  remaining="$( { session_owned; for pid in $(jobs -p); do session_alive "$pid" && echo "$pid"; done; } | sort -u )"
  SESSION_REMAINING=$(printf '%s' "$remaining" | grep -c .)
  SESSION_REMAINING_PIDS="$(echo $remaining)"
}

# 로봇 ID에 맞는 제어기 설정 파일 이름(robot_test.launch.py CONTROLLER_FILES·simulation.launch.py MANIPULATORS와 같음).
# 팔 로봇(robot_1·robot_3)은 ax1_controllers.yaml, 운송 로봇(robot_2·robot_4)은 ax2_controllers.yaml. 그 밖은 빈 문자열.
session_controller_file() {
  case "$1" in
    robot_1|robot_3) echo ax1_controllers.yaml ;;
    robot_2|robot_4) echo ax2_controllers.yaml ;;
    *) echo "" ;;
  esac
}

# 실제 launch가 읽는 설정(SLAM·로봇별 제어기·공통 제어기·센서 장착·외곽 기준)의 사본과 SHA-256을 회차에 남긴다.
# 사용: session_record_configs <session 폴더> <robot_id>. 경로는 launch와 같은 ament_index(get_package_share_directory,
# 현재 AMENT_PREFIX_PATH 순서)로 찾으므로 다른 설치 overlay에서도 실제로 읽히는 파일을 기록한다.
# 사본은 <session>/config/, 기록은 <session>/config_files.json. 패키지·파일 탐색·복사·해시 중 하나라도 실패하면
# 기록 파일을 쓰지 않고 1을 돌려준다(시뮬레이션 전에 실패). 평가 도구는 이 기록만 쓰고 현재 파일로 채우지 않는다.
session_record_configs() {
  local session="$1" robot="$2" controller shares name package share sum entries="" status=0
  controller="$(session_controller_file "$robot")"

  if [ -z "$controller" ]; then
    echo "실행 설정 기록 실패: 제어기 설정을 정할 수 없는 로봇 ID: $robot" >&2
    return 1
  fi

  # launch와 같은 함수로 share 경로를 찾는다(패키지가 없으면 실패).
  shares="$(python3 -c 'import sys
from ament_index_python.packages import get_package_share_directory as share
for name in sys.argv[1:]:
    print(name + "=" + share(name, print_warning=False))' ax_localization ax_control ax_description)" || {
    echo "실행 설정 기록 실패: ament share 경로를 찾지 못함(AMENT_PREFIX_PATH 확인)" >&2
    return 1
  }

  mkdir -p "$session/config" || return 1
  local -a files=("slam_mapping.yaml:ax_localization" "$controller:ax_control" "common_controllers.yaml:ax_control"
                  "sensor_mounts.yaml:ax_description" "a1_validation.yaml:ax_description")

  for item in "${files[@]}"; do
    name="${item%%:*}"; package="${item#*:}"
    share="$(printf '%s\n' "$shares" | sed -n "s|^$package=||p")"

    if ! cp -L "$share/config/$name" "$session/config/$name" 2> /dev/null; then
      echo "실행 설정 기록 실패: 복사 불가 $share/config/$name" >&2
      status=1
      continue
    fi

    if ! sum="$(sha256sum "$session/config/$name" 2> /dev/null | cut -d' ' -f1)" || [ ${#sum} -ne 64 ]; then
      echo "실행 설정 기록 실패: 해시 계산 불가 $session/config/$name" >&2
      status=1
      continue
    fi

    entries="$entries${entries:+, }\"$name\": {\"package\": \"$package\", \"source\": \"$share/config/$name\", \"copy\": \"session/config/$name\", \"sha256\": \"$sum\"}"
  done

  [ "$status" = 0 ] || return 1
  printf '{"robot_id": "%s", "controller_file": "%s", "files": {%s}}\n' "$robot" "$controller" "$entries" \
    > "$session/config_files.json" || return 1
}
