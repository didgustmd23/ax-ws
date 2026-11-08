#!/usr/bin/env bash
# B2 지도 작성 개발 세션을 명령 한 줄로 실행한다.
#   시뮬레이션 → B1 LiDAR 전처리 → SLAM을 격리 환경에서 띄우고, 준비를 확인한 뒤
#   지도 작성 주행(mapping_drive.py) → 정확도 비교(check_map_accuracy.py)를 실행하고 자기 프로세스만 정리한다.
#
# 사용: bash src/ax_localization/scripts/run_mapping_session.sh [옵션]
#   --light          AWS 메시 충돌을 단순화한 임시 사본으로 실행(시각 메시·LiDAR 형상은 같고 물리 충돌만 다름, 검증용 조건)
#   --gui            Gazebo 화면을 띄운다(기본 headless)
#   --software       GPU 대신 CPU 렌더링(llvmpipe)
#   --robot-id ID    기본 robot_2
#   --scan KIND      filtered(B1, 기본) 또는 raw
#   --domain N       ROS_DOMAIN_ID(기본 66). GZ_PARTITION은 실행마다 새로 만든다
#   --keep           주행·비교 뒤 시뮬레이션을 끄지 않고 Enter를 기다린다(RViz 확인용)
#   --waypoints N    앞에서부터 경유점 N개까지만 주행하고 지도는 저장하지 않는다(짧은 확인용)
#   --save-partial   --waypoints와 함께 쓰면 짧은 주행에서도 지도를 저장한다(저장·재로딩 확인용)
#   --wheel-slip1 V  바퀴 축 방향 미끄럼 순응(slip1, m/s/N)을 임시 사본에만 넣는다(A1 반영 전 비교 시험용)
#
# 결과: log/b2_mapping/<KST 시각>/run/(주행·지도·비교 결과), session/(프로세스 로그·조건)
# 격리된 개발 시뮬레이션 전용이다. A4 없이 제어기에 직접 명령하므로 운영 경로가 아니다.

set -uo pipefail

# ── 인자 ─────────────────────────────────────────────────────────────
LIGHT=0; GUI=false; SOFTWARE=0; ROBOT=robot_2; SCAN=filtered; DOMAIN=66; KEEP=0; WAYPOINTS=''; SLIP1=''; SAVE_PARTIAL=0

while [ $# -gt 0 ]; do
  case "$1" in
    --light) LIGHT=1 ;;
    --gui) GUI=true ;;
    --software) SOFTWARE=1 ;;
    --robot-id) ROBOT="$2"; shift ;;
    --scan) SCAN="$2"; shift ;;
    --domain) DOMAIN="$2"; shift ;;
    --keep) KEEP=1 ;;
    --waypoints) WAYPOINTS="$2"; shift ;;
    --wheel-slip1) SLIP1="$2"; shift ;;
    --save-partial) SAVE_PARTIAL=1 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "알 수 없는 인자: $1" >&2; exit 2 ;;
  esac
  shift
done

# ── 경로·환경 ────────────────────────────────────────────────────────
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
STAMP="$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)"
BASE="$WS/log/b2_mapping/$STAMP"
SESSION="$BASE/session"
RUN="$BASE/run"
mkdir -p "$SESSION"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# 모든 프로세스가 같은 값을 쓰도록 여기서 한 번만 정한다.
export ROS_DOMAIN_ID="$DOMAIN"
export GZ_PARTITION="ax-b2-$STAMP-$$"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$SESSION/roslog"

# WSL Mesa는 기본이 CPU 렌더링이므로 NVIDIA D3D12를 지정한다.
if [ "$SOFTWARE" = 1 ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  RENDER=software
else
  export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
  RENDER=d3d12-nvidia
fi

PIDS=()

# 단계 안내를 출력한다.
say() { echo "[session] $*"; }

# 이 세션이 띄운 프로세스 그룹만 정리한다(같은 GZ_PARTITION 확인 포함).
cleanup() {
  trap - EXIT INT TERM
  say "정리 중..."

  for pid in "${PIDS[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null
  done

  sleep 5

  for pid in $(pgrep -f "ros2|gz sim|ruby|parameter_bridge|robot_state|spawner|slam_toolbox|sensor_processing|suction_tool|map_saver" 2>/dev/null); do
    tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "GZ_PARTITION=$GZ_PARTITION" && kill -9 "$pid" 2>/dev/null
  done

  [ -d "$SESSION/overlay" ] && rm -rf "$SESSION/overlay/share"
  say "정리 완료. 결과: $BASE"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

# 명령을 새 프로세스 그룹으로 띄우고 로그를 남긴다.
start() {
  local name="$1"; shift
  setsid "$@" > "$SESSION/$name.log" 2>&1 < /dev/null &
  PIDS+=("$!")
}

# 토픽에서 메시지 하나가 올 때까지 기다린다.
wait_topic() {
  local topic="$1" label="$2" limit="$3" start_time=$SECONDS

  say "$label 기다리는 중 ($topic)"

  until timeout 10 ros2 topic echo --once "$topic" > /dev/null 2>&1; do
    if [ $((SECONDS - start_time)) -ge "$limit" ]; then
      say "실패: $label 이 ${limit}s 안에 오지 않음. 로그: $SESSION"
      exit 1
    fi
  done

  say "$label 확인"
}

# ── 1. 임시 사본(--light, --wheel-slip1) ──────────────────────────────
WORLD_CONDITION=original
WHEEL_CONDITION=original
OV="$SESSION/overlay"

if [ "$LIGHT" = 1 ] || [ -n "$SLIP1" ]; then
  mkdir -p "$OV/share/ament_index/resource_index/packages"
  cp -rL "$WS/install/ax_description/share/ax_description" "$OV/share/" 2>/dev/null
  touch "$OV/share/ament_index/resource_index/packages/ax_description"
  export AMENT_PREFIX_PATH="$OV:$AMENT_PREFIX_PATH"
fi

if [ -n "$SLIP1" ]; then
  say "바퀴 slip1=$SLIP1 사본 만드는 중"
  python3 - "$OV/share/ax_description/urdf/ros2_control.xacro" "$SLIP1" <<'EOF' || exit 1
import sys
from pathlib import Path
p = Path(sys.argv[1]); s = p.read_text()
# <gazebo reference>의 <slip1>은 SDF 변환에서 무시되므로 collision 마찰 블록으로 넣는다.
old = '            <fdir1>0 0 1</fdir1>\n'
assert s.count(old) == 1, '바퀴 마찰 블록을 찾지 못함'
p.write_text(s.replace(old, old + f'            <collision><surface><friction><ode><slip1>{sys.argv[2]}</slip1></ode></friction></surface></collision>\n'))
EOF
  cp "$OV/share/ax_description/urdf/ros2_control.xacro" "$SESSION/ros2_control_slip.xacro"
  WHEEL_CONDITION="slip1=$SLIP1"
fi

if [ "$LIGHT" = 1 ]; then
  say "충돌 단순화 사본 만드는 중"

  python3 - "$OV/share/ax_description/world_assets/aws_small_warehouse/aws_warehouse_harmonic.world" <<'EOF' || exit 1
import re, sys
from pathlib import Path
p = Path(sys.argv[1]); s = p.read_text()
# 바닥 메시 충돌은 평면으로, 나머지 메시 충돌은 제거한다. 시각 메시(LiDAR·카메라가 보는 형상)는 유지한다.
i = s.index('<model name="aws_robomaker_warehouse_GroundB_01_001">'); j = s.index('</model>', i)
ground = re.sub(r'<collision name="([^"]+)">.*?</collision>',
                r'<collision name="\1"><pose>0 0 0.09 0 0 0</pose><geometry><plane><normal>0 0 1</normal>'
                r'<size>14 21</size></plane></geometry></collision>', s[i:j], count=1, flags=re.S)
s = s[:i] + ground + s[j:]
s, n = re.subn(r'<collision name="[^"]+">\s*<geometry>\s*<mesh>.*?</collision>', '', s, flags=re.S)
p.write_text(s)
print(f"[session] 메시 충돌 {n}개 제거, 바닥은 평면")
EOF

  cp "$OV/share/ax_description/world_assets/aws_small_warehouse/aws_warehouse_harmonic.world" "$SESSION/world_light.world"
  WORLD_CONDITION=light_collision
fi

cat > "$SESSION/session.json" <<EOF
{
  "world": "aws_warehouse.world",
  "world_condition": "$WORLD_CONDITION",
  "wheel_condition": "$WHEEL_CONDITION",
  "robot_id": "$ROBOT",
  "scan": "$SCAN",
  "render": "$RENDER",
  "ROS_DOMAIN_ID": "$ROS_DOMAIN_ID",
  "GZ_PARTITION": "$GZ_PARTITION",
  "note": "light_collision·wheel slip1은 임시 사본 조건이며 운영 모델이 아니고 지도 채택(--adopt) 대상이 아니다"
}
EOF

say "환경: ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION 렌더링=$RENDER 월드=$WORLD_CONDITION 바퀴=$WHEEL_CONDITION"

# ── 2. 시뮬레이션 ────────────────────────────────────────────────────
say "시뮬레이션 시작 (로그 $SESSION/sim.log)"
start sim ros2 launch ax_control simulation.launch.py world:=aws_warehouse.world robot_ids:="$ROBOT" gui:="$GUI"
wait_topic "/ax/$ROBOT/diff_drive_controller/odom" "odom" 300

# ── 3. B1 LiDAR 전처리 ───────────────────────────────────────────────
if [ "$SCAN" = filtered ]; then
  say "B1 센서 처리 시작"
  start b1 ros2 launch ax_perception sensors.launch.py robot_ids:="$ROBOT"
  wait_topic "/ax/$ROBOT/sensors/front_lidar/scan_filtered" "B1 scan_filtered" 120
fi

# ── 4. SLAM ─────────────────────────────────────────────────────────
say "SLAM 시작"
start slam ros2 launch ax_localization mapping.launch.py robot_id:="$ROBOT" scan:="$SCAN"
wait_topic "/ax/$ROBOT/map" "SLAM 지도" 180

# ── 5. 주행 → 비교 ───────────────────────────────────────────────────
say "지도 작성 주행 시작. 중단하려면 Ctrl+C (정지 확인 뒤 종료)"
DRIVE_ARGS=(--output "$RUN" --scan "$SCAN")
if [ -n "$WAYPOINTS" ]; then
  DRIVE_ARGS+=(--max-waypoints "$WAYPOINTS")
  [ "$SAVE_PARTIAL" = 1 ] || DRIVE_ARGS+=(--no-save)
fi
python3 "$WS/src/ax_localization/scripts/mapping_drive.py" "${DRIVE_ARGS[@]}"
DRIVE=$?

RESULT=$DRIVE

# 주행이 실패해도 기록이 있으면 궤적 비교는 할 수 있다. 세 상태(drive·trajectory·map)는 비교 도구가 따로 남긴다.
if [ -f "$RUN/recording.npz" ]; then
  say "비교 실행 (주행 종료 코드 $DRIVE)"
  python3 "$WS/src/ax_localization/scripts/check_map_accuracy.py" "$RUN"
  CHECK=$?
  [ "$DRIVE" = 0 ] && RESULT=$CHECK
else
  say "기록(recording.npz)이 없어 비교를 건너뜀. $RUN/drive_result.json 확인"
fi

if [ "$KEEP" = 1 ]; then
  say "시뮬레이션 유지 중. 다른 터미널에서 같은 환경으로 확인:"
  say "  export ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
  read -r -p "[session] Enter를 누르면 정리합니다 " _
fi

exit "$RESULT"
