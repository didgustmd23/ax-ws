#!/usr/bin/env bash
# robot_1 회전 시험 조건 실행기. 조건마다 새 격리 시뮬레이션을 띄우고 r1_rotation.py를 실행한 뒤 자기 프로세스만 정리한다.
# 사용: bash run_conditions.sh <이름>:<자세>:<하중kg>:<slip1 또는 ->:<contact 0|1>[:<회전 회차 수>] ...
set -uo pipefail
R="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/ax_ws"
TMP="<tmp>/r1"
mkdir -p "$TMP" "$R/trials" "$R/sim_logs" "$R/model_diffs"
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
BASE_PREFIX="$AMENT_PREFIX_PATH"

# 바퀴 마찰·접촉 센서 임시 사본을 만들고 원본 대비 차이를 남긴다.
make_overlay() {
  local name="$1" slip="$2" contact="$3" ov="$TMP/overlay_$1"
  rm -rf "$ov"; mkdir -p "$ov/share/ament_index/resource_index/packages"
  cp -rL "$WS/install/ax_description/share/ax_description" "$ov/share/" 2>/dev/null
  touch "$ov/share/ament_index/resource_index/packages/ax_description"
  python3 - "$ov/share/ax_description/urdf/ros2_control.xacro" "$slip" "$contact" <<'EOF'
import sys
from pathlib import Path
p = Path(sys.argv[1]); s = p.read_text(); slip, contact = sys.argv[2], sys.argv[3]
old = '            <fdir1>0 0 1</fdir1>\n'
assert s.count(old) == 1
add = ''
if slip != '-':
    add += f'            <collision><surface><friction><ode><slip1>{slip}</slip1></ode></friction></surface></collision>\n'
if contact == '1':
    add += ('            <sensor name="${link}_contact" type="contact"><always_on>true</always_on><update_rate>100</update_rate>'
            '<contact><collision>${link}_collision</collision></contact></sensor>\n')
p.write_text(s.replace(old, old + add))
EOF
  diff -u "$WS/src/ax_description/urdf/ros2_control.xacro" "$ov/share/ax_description/urdf/ros2_control.xacro" > "$R/model_diffs/$name.diff"
  echo "$ov"
}

# 같은 GZ_PARTITION을 가진 자기 프로세스만 정리한다.
cleanup() {
  local part="$1"
  for pid in $(pgrep -f "ros2|gz sim|ruby|parameter_bridge|robot_state|spawner|suction_tool|r1_rotation"); do
    tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "GZ_PARTITION=$part" && kill -INT "$pid" 2>/dev/null
  done
  sleep 6
  for pid in $(pgrep -f "ros2|gz sim|ruby|parameter_bridge|robot_state|spawner|suction_tool|r1_rotation"); do
    tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "GZ_PARTITION=$part" && kill -9 "$pid" 2>/dev/null
  done
}

for spec in "$@"; do
  IFS=: read -r name pose payload slip contact rotations <<< "$spec"
  export ROS_DOMAIN_ID=87 GZ_PARTITION="ax-r1-$name-$$" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ROS_LOG_DIR="$TMP/roslog_$name"
  export AMENT_PREFIX_PATH="$BASE_PREFIX"

  if [ "$slip" != "-" ] || [ "$contact" = 1 ]; then
    ov=$(make_overlay "$name" "$slip" "$contact" | tail -1)
    export AMENT_PREFIX_PATH="$ov:$BASE_PREFIX"
  fi

  log="$R/sim_logs/$name.launch.log"
  setsid ros2 launch ax_control simulation.launch.py robot_ids:=robot_1 gui:=false sensors:=false tool:=epick \
    world:=ax_test.world payload_mass:="$payload" > "$log" 2>&1 < /dev/null &
  ready=0
  for i in $(seq 1 120); do grep -q "Velocity command timed out" "$log" && { ready=1; break; }; sleep 2; done

  if [ "$ready" = 1 ]; then
    extra=(); [ "$contact" = 1 ] && extra+=(--contact); [ -n "${rotations:-}" ] && extra+=(--rotations-only "$rotations")
    timeout 3000 python3 "$R/r1_rotation.py" "$name" "$pose" "$R/trials/$name.json" "${extra[@]}" > "$R/sim_logs/$name.trial.log" 2>&1
    echo "$name exit $?"
  else
    echo "$name sim not ready"; echo "{\"condition\": \"$name\", \"status\": \"failed\", \"error\": \"시뮬레이션 준비 실패\"}" > "$R/trials/$name.json"
  fi

  cleanup "$GZ_PARTITION"
  echo "$name cleaned: $(for p in $(pgrep -f 'gz sim|ros2'); do tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "GZ_PARTITION=$GZ_PARTITION" && echo $p; done | wc -l) left"
done
