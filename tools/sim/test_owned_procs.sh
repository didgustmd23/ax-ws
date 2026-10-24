#!/bin/bash
# Regression checks for owned_procs.sh / own_procs.py with harmless sleep processes.
# Every process this test creates is recorded as (pid, start_time); liveness checks and cleanup use
# only those records (test_proc.py, pidfd) -- no pkill/pgrep by command line -- so concurrent runs
# cannot touch each other's processes. A trap stops all recorded processes even when a check fails.
# Optional: $1 = file that receives this run's "pid start" records (used by the concurrency runner).
HERE=$(cd "$(dirname "$0")" && pwd); T=$(mktemp -d "$HERE/owned_test_XXXX"); fail=0
TP="python3 $HERE/test_proc.py"; RECORDS=()
pass() { echo "PASS $1"; }; bad() { echo "FAIL $1"; fail=1; }
# track VAR PID: record (pid, start_time) in RECORDS and store "pid start" in VAR. It must run in the
# current shell (never inside $(...)), otherwise the record would be lost and not cleaned up.
track() { local var=$1 pid=$2 s; printf -v "$var" '%s' ''
          [ -n "$pid" ] && s=$($TP stamp "$pid") || return 1
          RECORDS+=("$pid $s"); printf -v "$var" '%s' "$pid $s"; }
alive() { [ -n "$1" ] && $TP alive $1; }                                           # arg: "pid start"
wait_pidfile() { for _ in $(seq 50); do [ -s "$1" ] && { cat "$1"; return; }; sleep 0.1; done; }
test_cleanup() {   # stop every recorded test process (exact pid + start time, via pidfd)
  local r; for r in "${RECORDS[@]}"; do $TP kill $r SIGTERM; done; sleep 0.3
  for r in "${RECORDS[@]}"; do $TP kill $r SIGKILL; done; sleep 0.1
  [ -n "${OWN_STATE:-}" ] && rm -rf "$OWN_STATE"; rm -rf "$T"
}
trap test_cleanup EXIT
export GZ_PARTITION=ax_owned_test

setsid sleep 501 > /dev/null 2>&1 & track DECOY $!             # pre-existing, same partition
sleep 0.2

# Previous parent-PID-only tracker on the fast-detach case (comparison; its leftover is ours to clean)
( source "$HERE/owned_procs_v1_ppid.sh"; own_init
  own_start "$T/v1.log" bash -c "setsid sleep 512 & echo \$! > '$T/v1.pid'; exit 0"
  sleep 1; own_cleanup ) > /dev/null 2>&1
track LEGACY "$(wait_pidfile "$T/v1.pid")"
if [ -n "$LEGACY" ] && alive "$LEGACY"; then echo "INFO previous parent-PID tracker missed the fast-detaching child (expected)"
else echo "INFO previous tracker caught the child this time (timing dependent)"; fi

source "$HERE/owned_procs.sh"; own_init
own_start "$T/a.log" bash -c "setsid sleep 502 & echo \$! > '$T/a.pid'; exit 0"   # detaches, parent exits
track DETACHED "$(wait_pidfile "$T/a.pid")"
own_start "$T/b.log" sleep 503;        track ORDINARY "$OWN_LAST_PID"
own_start "$T/c.log" env -i sleep 505; track ENVLESS "$OWN_LAST_PID"               # root without token
setsid sleep 504 > /dev/null 2>&1 & track UNRELATED $!                              # no token, not owned
sleep 0.5
# test hook: abort here to prove the EXIT trap cleans up after a failure
[ "${AX_OWN_TEST_ABORT:-}" = 1 ] && { echo "ABORT (test hook) with ${#RECORDS[@]} recorded processes"; exit 3; }
tr '\0' '\n' < /proc/${ENVLESS% *}/environ | grep -q AX_OWN_TOKEN \
  && bad "env -i root still has the token (test premise broken)" || pass "env -i root has no token (premise)"
# registration must refuse a PID whose parent is not this shell (the re-parented legacy child)
python3 "$HERE/own_procs.py" register "$OWN_STATE" "${LEGACY% *}" "$BASHPID" 2> /dev/null \
  && bad "register accepted a foreign PID" || pass "register refused a PID that is not this shell's child"
# stale record: UNRELATED's PID with a different start time, as if the PID had been recycled
echo "${UNRELATED% *} $(( ${UNRELATED#* } - 1 ))" >> "$OWN_STATE/owned.txt"
own_cleanup
alive "$DECOY"     && pass "pre-existing same-partition process kept" || bad "pre-existing process was signalled"
alive "$ORDINARY"  && bad "ordinary owned process still alive"        || pass "ordinary owned process stopped"
alive "$ENVLESS"   && bad "env -i root still alive"                   || pass "env -i root stopped (registered root)"
alive "$DETACHED"  && bad "fast-detaching grandchild still alive"     || pass "fast-detaching grandchild stopped (token)"
alive "$UNRELATED" && pass "stale (pid, start_time) record did not signal the unrelated PID" || bad "unrelated PID was signalled"
[ ${#RECORDS[@]} -ge 6 ] && pass "all ${#RECORDS[@]} test processes recorded" || bad "only ${#RECORDS[@]} test processes recorded"
[ -n "${1:-}" ] && printf '%s\n' "${RECORDS[@]}" > "$1"
test_cleanup; trap - EXIT
left=0; for r in "${RECORDS[@]}"; do alive "$r" && left=$((left + 1)); done
[ $left -eq 0 ] && pass "no recorded test process left after cleanup" || bad "$left recorded test processes left"
exit $fail
