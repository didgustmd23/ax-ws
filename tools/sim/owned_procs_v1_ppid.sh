# Owned-process tracking for isolated verification scripts (source this file).
# Only processes started through `own_start` and their descendants are ever signalled.
# Descendants are collected repeatedly with `own_snapshot` so that a child that changes its
# process group or session (e.g. gz sim under the ruby wrapper) is still known after its parent
# exits. PIDs that existed before `own_init` (e.g. another user's gz sim) are never signalled.
OWN_ROOTS=(); declare -A OWN_SET=(); declare -A OWN_PRE=()
own_init() { for p in $(ps -eo pid=); do OWN_PRE[$p]=1; done; }
own_start() {   # own_start LOGFILE CMD...
  local log=$1; shift
  setsid "$@" > "$log" 2>&1 &
  OWN_ROOTS+=($!); OWN_SET[$!]=1
}
own_snapshot() {   # add every live descendant (by parent PID) of already owned PIDs
  local changed=1 pid ppid
  while [ $changed -eq 1 ]; do
    changed=0
    while read -r pid ppid; do
      if [ -n "${OWN_SET[$ppid]:-}" ] && [ -z "${OWN_SET[$pid]:-}" ] && [ -z "${OWN_PRE[$pid]:-}" ]; then
        OWN_SET[$pid]=1; changed=1; fi
    done < <(ps -eo pid=,ppid=)
  done
}
own_alive() { local p; for p in "${!OWN_SET[@]}"; do [ -z "${OWN_PRE[$p]:-}" ] && kill -0 "$p" 2>/dev/null && echo "$p"; done; }
own_cleanup() {
  own_snapshot
  local p
  for p in $(own_alive); do kill -INT "$p" 2>/dev/null; done; sleep 6
  own_snapshot
  for p in $(own_alive); do kill -TERM "$p" 2>/dev/null; done; sleep 3
  for p in $(own_alive); do kill -KILL "$p" 2>/dev/null; done
}
