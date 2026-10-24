# Owned-process tracking for isolated verification scripts (source this file).
# Bookkeeping lives in own_procs.py. Ownership: (1) the root PID of every own_start is registered
# directly with its kernel start time (works even if the command clears its environment, env -i);
# (2) every command carries a unique AX_OWN_TOKEN, so descendants that detach quickly (new session or
# process group, parent already exited) are found by token; (3) live descendants of owned processes.
# Records are (pid, start_time) pairs and signals go through pidfd after re-checking the start time,
# so a recycled PID is never signalled. Processes that existed before own_init are excluded.
OWN_HELPER="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/own_procs.py"
own_init() {
  OWN_STATE=$(mktemp -d "${TMPDIR:-/tmp}/ax_own_XXXXXX")
  OWN_TOKEN=$(python3 -c 'import uuid; print(uuid.uuid4().hex)')
  python3 "$OWN_HELPER" init "$OWN_STATE"
}
own_start() {   # own_start LOGFILE CMD...   (sets OWN_LAST_PID)
  local log=$1; shift
  # A background job of a non-interactive shell is not a group leader, so setsid execs in place
  # and $! is the command's own PID (env -i also execs in place).
  AX_OWN_TOKEN="$OWN_TOKEN" setsid "$@" > "$log" 2>&1 &
  OWN_LAST_PID=$!
  # Register the root by (pid, start_time); exit 2 = it already exited (its descendants are still
  # tracked by token / parent PID), exit 1 = refused by the PID-reuse guard.
  python3 "$OWN_HELPER" register "$OWN_STATE" "$OWN_LAST_PID" "$BASHPID"
  [ $? -eq 1 ] && echo "own_start: root $OWN_LAST_PID refused" >&2
  own_snapshot
}
own_snapshot() { python3 "$OWN_HELPER" snapshot "$OWN_STATE" "$OWN_TOKEN"; }
own_alive() { python3 "$OWN_HELPER" alive "$OWN_STATE"; }
own_cleanup() {
  own_snapshot; python3 "$OWN_HELPER" signal "$OWN_STATE" SIGINT > /dev/null; sleep 6
  own_snapshot; python3 "$OWN_HELPER" signal "$OWN_STATE" SIGTERM > /dev/null; sleep 3
  own_snapshot; python3 "$OWN_HELPER" signal "$OWN_STATE" SIGKILL > /dev/null
  rm -rf "$OWN_STATE"
}
