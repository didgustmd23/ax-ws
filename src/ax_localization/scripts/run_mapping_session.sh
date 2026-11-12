#!/usr/bin/env bash
# B2 지도 작성 개발 세션을 명령 한 줄로 실행한다.
#   시뮬레이션 → B1 LiDAR 전처리 → SLAM을 격리 환경에서 띄우고, 준비를 확인한 뒤
#   지도 작성 주행(mapping_drive.py) → 정확도 비교(check_map_accuracy.py)를 실행하고 자기 프로세스만 정리한다.
#
# 사용: bash src/ax_localization/scripts/run_mapping_session.sh [옵션]
#   --world FILE     실행 월드(ax_description worlds 기준). 지원 목록은 ax_description/config/worlds.yaml
#                    (planned/ax_warehouse.world 기준 질감, planned/ax_warehouse_plain.world 단색 보존, Gazebo 내부 이름 ax_test).
#                    이 옵션과 같은 world를 적은 --route가 함께 필요하다. 없으면 삭제된 AWS 월드 시험기로 보고 거절한다
#   --light          AWS 메시 충돌을 단순화한 임시 사본으로 실행(시각 메시·LiDAR 형상은 같고 물리 충돌만 다름, 검증용 조건)
#   --gui            Gazebo 화면을 띄운다(기본 headless)
#   --software       GPU 대신 CPU 렌더링(llvmpipe)
#   --robot-id ID    기본 robot_2
#   --scan KIND      SLAM 입력. merged(기본, 앞·뒤 B1 scan_filtered를 scan_merge.py로 병합; mapping.launch.py가 병합 노드를 띄움),
#                    filtered(앞 B1만, 비교용) 또는 raw(앞 원시, 비교용). 장애물 감시는 merged·filtered면 앞·뒤 B1 scan_filtered
#   --domain N       ROS_DOMAIN_ID(기본 66). GZ_PARTITION은 실행마다 새로 만든다
#   --keep           주행·비교 뒤 시뮬레이션을 끄지 않고 Enter를 기다린다(RViz 확인용)
#   --waypoints N    앞에서부터 경유점 N개까지만 주행하고 지도는 저장하지 않는다(짧은 확인용)
#   --save-partial   --waypoints와 함께 쓰면 짧은 주행에서도 지도를 저장한다(저장·재로딩 확인용)
#   --wheel-slip1 V  바퀴 축 방향 미끄럼 순응(slip1, m/s/N)을 임시 사본에만 넣는다(A1 반영 전 비교 시험용)
#   --route FILE     지도 작성 경로 yaml(기본: mapping_drive.py 기본 경로). 사본을 session/route.yaml로 남긴다
#   --wait-before-drive  SLAM 준비 뒤 주행 직전에 터미널 Enter를 기다린다(q 입력 시 주행 없이 종료). 터미널이 없으면 실패
#   --gate-dir DIR   mapping_drive.py 관측 관문(경로의 gate: true) 신호·정지 기록 폴더(없으면 관문 없이 연속 주행)
#
# 예(두 창고 월드 중앙 통로 구간): bash src/ax_localization/scripts/run_mapping_session.sh \
#   --world planned/ax_warehouse.world --route src/ax_localization/config/routes/ax_warehouse_center.yaml --gate-dir <폴더>
# 시뮬레이션을 띄우기 전에 경로의 world·gz_world·robot_id가 실행 조건과 같은지 mapping_drive.py --check-only로 검사한다.
#
# 결과: log/b2_mapping/<KST 시각>/run/(주행·지도·비교 결과), session/(프로세스 로그·조건)
# 격리된 개발 시뮬레이션 전용이다. A4 없이 제어기에 직접 명령하므로 운영 경로가 아니다.

set -uo pipefail

# ── 인자 ─────────────────────────────────────────────────────────────
LIGHT=0; GUI=false; SOFTWARE=0; ROBOT=robot_2; SCAN=merged; DOMAIN=66; KEEP=0; WAYPOINTS=''; SLIP1=''; SAVE_PARTIAL=0
ROUTE=''; WAIT_BEFORE_DRIVE=0; GATE_DIR=; WORLD=''
GZ_WORLD=ax_test

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
    --route)
      [ -n "${2:-}" ] || { echo "--route에 경로 yaml 파일이 필요함" >&2; exit 2; }
      ROUTE="$2"; shift ;;
    --wait-before-drive) WAIT_BEFORE_DRIVE=1 ;;
    --world)
      [ -n "${2:-}" ] || { echo "--world에 월드 파일이 필요함" >&2; exit 2; }
      WORLD="$2"; shift ;;
    --gate-dir)
      [ -n "${2:-}" ] || { echo "--gate-dir에 폴더가 필요함" >&2; exit 2; }
      GATE_DIR="$2"; shift ;;
    -h|--help) sed -n '2,29p' "$0"; exit 0 ;;
    *) echo "알 수 없는 인자: $1" >&2; exit 2 ;;
  esac
  shift
done

case "$SCAN" in
  merged|filtered|raw) ;;
  *) echo "--scan은 merged, filtered, raw 중 하나: $SCAN" >&2; exit 2 ;;
esac

# ── 경로·환경 ────────────────────────────────────────────────────────
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
# --world가 없으면 옛 AWS 전용 시험 조건으로 본다. 그 월드는 삭제됐으므로 현재 월드에 잘못 적용하지 않고 거절한다.
if [ -z "$WORLD" ] && [ ! -f "$WS/world_assets/aws_small_warehouse/aws_warehouse_harmonic.world" ]; then
  echo "AWS 월드는 삭제됐습니다. 이 과거 지도 작성 시험기는 현재 창고 월드에 사용할 수 없습니다." >&2
  echo "두 창고 월드는 --world planned/ax_warehouse.world와 그 월드용 --route를 함께 지정하세요." >&2
  exit 2
fi

# 지원 월드와 그 월드에서 검증하지 않은 옵션을 시뮬레이션 전에 거절한다.
if [ -n "$WORLD" ]; then
  # 지원 월드는 ax_description config/worlds.yaml 목록이다(기준 질감 월드, 단색 보존 월드).
  SUPPORTED_WORLDS="$(python3 -c 'import sys, yaml; print(" ".join(yaml.safe_load(open(sys.argv[1]))["worlds"]))' \
    "$WS/src/ax_description/config/worlds.yaml")" || { echo "월드 목록(worlds.yaml)을 읽지 못함" >&2; exit 2; }
  case " $SUPPORTED_WORLDS " in
    *" $WORLD "*) ;;
    *) echo "지원하지 않는 --world: $WORLD (지원: $SUPPORTED_WORLDS)" >&2; exit 2 ;;
  esac

  [ -n "$ROUTE" ] || { echo "--world $WORLD에는 같은 world를 적은 --route가 필요함(기본 AWS 경로 사용 불가)" >&2; exit 2; }
  [ "$LIGHT" = 0 ] || { echo "--light는 AWS 월드 사본 전용이라 --world $WORLD와 함께 쓸 수 없음" >&2; exit 2; }
  [ -z "$SLIP1" ] || { echo "--wheel-slip1은 --world $WORLD에서 검증하지 않아 함께 쓸 수 없음" >&2; exit 2; }
fi

# 시뮬레이션을 띄우기 전에 선택 인자를 검사한다(경로 파일 존재, 대기에 필요한 터미널).
if [ -n "$ROUTE" ]; then
  [ -f "$ROUTE" ] || { echo "--route 파일이 없음: $ROUTE" >&2; exit 2; }
  ROUTE="$(cd "$(dirname "$ROUTE")" && pwd)/$(basename "$ROUTE")"
fi

if [ "$WAIT_BEFORE_DRIVE" = 1 ] && ! [ -t 0 ]; then
  echo "--wait-before-drive는 터미널(Enter 입력)이 필요함: 표준 입력이 터미널이 아님" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# 새 월드: 설치된 월드 파일과 경로의 world·gz_world·robot_id 일치를 시뮬레이션 전에 확인한다.
if [ -n "$WORLD" ]; then
  WORLD_FILE="$WS/install/ax_description/share/ax_description/worlds/$WORLD"
  [ -f "$WORLD_FILE" ] || { echo "설치된 월드 파일이 없음(ax_description 빌드 필요): $WORLD_FILE" >&2; exit 2; }
  ROUTE_CHECK="$(python3 "$WS/src/ax_localization/scripts/mapping_drive.py" --route "$ROUTE" --world "$WORLD" \
    --gz-world "$GZ_WORLD" --robot-id "$ROBOT" --check-only 2>&1)" || {
    echo "경로 사전검사 실패(시뮬레이션을 띄우지 않음):" >&2
    echo "$ROUTE_CHECK" >&2
    exit 2
  }
fi

STAMP="$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)"
BASE="$WS/log/b2_mapping/$STAMP"
SESSION="$BASE/session"
RUN="$BASE/run"
mkdir -p "$SESSION"

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

# 이 세션이 띄운 프로세스만 찾도록 소유 표식을 둔다. 이후 띄우는 모든 자식 프로세스 환경에 들어간다.
export AX_SESSION_OWNER="b2-mapping-$STAMP-$$"
# shellcheck source=session_procs.sh
source "$WS/src/ax_localization/scripts/session_procs.sh"
DRIVE_PID=
TAIL_PID=

# 단계 안내를 출력한다.
say() { echo "[session] $*"; }

# 주행(기록) 프로세스 그룹이 살아 있으면 SIGINT를 보내 정지 절차를 밟게 하고 끝날 때까지 기다린다(최대 90 s).
stop_drive() {
  [ -n "$DRIVE_PID" ] && kill -0 "$DRIVE_PID" 2>/dev/null || return 0
  say "주행 프로세스에 중단 신호(SIGINT): 정지 확인 뒤 종료를 기다림"
  session_stop_group "$DRIVE_PID" 90
}

# 이 세션이 띄운 프로세스 그룹과 소유 표식 프로세스만 정리하고 잔여 수를 session/cleanup.json에 남긴다.
cleanup() {
  trap - EXIT INT TERM
  say "정리 중..."
  stop_drive
  [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null
  # 세션 셸과 같은 세션에서 도는 앞쪽 명령(check_map_accuracy 등)은 끝난 뒤이므로 그룹·소유 표식만 정리한다.
  session_cleanup 10
  local clean=true
  [ "$SESSION_REMAINING" = 0 ] || clean=false
  printf '{"owner": "%s", "gz_partition": "%s", "ros_domain_id": "%s", "owned_remaining": %s, "remaining_pids": "%s", "clean": %s}\n' \
    "$AX_SESSION_OWNER" "$GZ_PARTITION" "$ROS_DOMAIN_ID" "$SESSION_REMAINING" "$SESSION_REMAINING_PIDS" "$clean" > "$SESSION/cleanup.json"
  [ -d "$SESSION/overlay" ] && rm -rf "$SESSION/overlay/share"

  if [ "$clean" = true ]; then
    say "정리 완료(소유 프로세스 잔여 0). 결과: $BASE"
  else
    say "정리 미완료: 소유 프로세스 잔여 $SESSION_REMAINING (PID $SESSION_REMAINING_PIDS). 결과: $BASE"
  fi
}

trap cleanup EXIT
trap 'exit 130' INT TERM

# 명령을 새 프로세스 그룹으로 띄우고 로그를 남긴다.
start() {
  local name="$1"; shift
  session_start "$name" "$SESSION/$name.log" "$@"
}

# 토픽에서 메시지 하나가 올 때까지 기다린다.
wait_topic() {
  local topic="$1" label="$2" limit="$3" start_time=$SECONDS

  say "$label 기다리는 중 ($topic)"

  # --no-daemon: ROS CLI 데몬(ros2-daemon)을 띄우지 않는다. 데몬은 부모가 바뀐 채 세션 셸과 같은 SID에 남는다(4단계 확인).
  until timeout 10 ros2 topic echo --once --no-daemon "$topic" > /dev/null 2>&1; do
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

# 실제 실행에 쓰는 월드·바퀴 사본 파일과 해시, 코드 버전을 기록한다(map_info.yaml 근거).
if [ -n "$WORLD" ]; then
  # 새 월드는 설치 사본 그대로 실행한다(사본 옵션은 위에서 거절). 실행 조건 사본을 session에 남긴다.
  WORLD_RECORD="install/ax_description/share/ax_description/worlds/$WORLD"
  cp "$WORLD_FILE" "$SESSION/world.world"
elif [ "$WORLD_CONDITION" = original ]; then
  WORLD_FILE="$WS/install/ax_description/share/ax_description/world_assets/aws_small_warehouse/aws_warehouse_harmonic.world"
  WORLD_RECORD="install/ax_description/share/ax_description/world_assets/aws_small_warehouse/aws_warehouse_harmonic.world"
else
  WORLD_FILE="$SESSION/world_light.world"
  WORLD_RECORD="session/world_light.world"
fi

WORLD_SHA=""
[ -f "$WORLD_FILE" ] && WORLD_SHA="$(sha256sum "$WORLD_FILE" | cut -d' ' -f1)"
WHEEL_FILE=""; WHEEL_SHA=""

if [ -n "$SLIP1" ]; then
  WHEEL_FILE="session/ros2_control_slip.xacro"
  WHEEL_SHA="$(sha256sum "$SESSION/ros2_control_slip.xacro" | cut -d' ' -f1)"
fi

# 실제 launch가 읽는 설치본 설정의 사본·해시(평가 도구가 회차 기록으로만 대조한다). 없으면 시뮬레이션 전에 실패한다.
session_record_configs "$SESSION" "$ROBOT" || { say "실행 설정 사본을 남기지 못해 중단(config_files)"; exit 2; }

GIT_COMMIT="$(git -C "$WS" rev-parse HEAD 2>/dev/null)"
GIT_DIRTY=false
[ -n "$(git -C "$WS" status --porcelain --untracked-files=no 2>/dev/null)" ] && GIT_DIRTY=true
# 미커밋 변경도 실행 조건이므로 diff 사본과 해시를 남긴다.
git -C "$WS" diff HEAD > "$SESSION/git_diff.patch" 2>/dev/null
git -C "$WS" status --porcelain > "$SESSION/git_status.txt" 2>/dev/null
GIT_DIFF_SHA="$(sha256sum "$SESSION/git_diff.patch" | cut -d' ' -f1)"
ROUTE_SHA=""

if [ -n "$ROUTE" ]; then
  cp "$ROUTE" "$SESSION/route.yaml"
  ROUTE_SHA="$(sha256sum "$ROUTE" | cut -d' ' -f1)"
fi

# SLAM 입력이 병합 스캔이면 launch가 실행할 설치본 scan_merge.py 위치·해시를 남긴다(찾지 못하면 시뮬레이션 전에 실패).
SCAN_MERGE_RECORD=null

if [ "$SCAN" = merged ]; then
  MERGE_FILE="$(python3 -c 'from ament_index_python.packages import get_package_prefix as p; print(p("ax_localization") + "/lib/ax_localization/scan_merge.py")' 2>/dev/null)"
  MERGE_SHA="$(sha256sum "$MERGE_FILE" 2>/dev/null | cut -d' ' -f1)"

  if [ ${#MERGE_SHA} -ne 64 ]; then
    say "병합 노드(scan_merge.py) 설치본을 찾지 못해 중단: $MERGE_FILE (colcon build 확인)"
    exit 2
  fi

  SCAN_MERGE_RECORD="{\"file\": \"$MERGE_FILE\", \"sha256\": \"$MERGE_SHA\"}"
fi

cat > "$SESSION/session.json" <<EOF
{
  "world": "${WORLD:-aws_warehouse.world}",
  "gz_world": "$GZ_WORLD",
  "route_file": "$ROUTE",
  "route_copy": "session/route.yaml",
  "route_sha256": "$ROUTE_SHA",
  "git_diff_sha256": "$GIT_DIFF_SHA",
  "session_owner": "$AX_SESSION_OWNER",
  "config_files": $(cat "$SESSION/config_files.json"),
  "world_condition": "$WORLD_CONDITION",
  "world_file": "$WORLD_RECORD",
  "world_sha256": "$WORLD_SHA",
  "wheel_condition": "$WHEEL_CONDITION",
  "wheel_override_file": "$WHEEL_FILE",
  "wheel_override_sha256": "$WHEEL_SHA",
  "git_commit": "$GIT_COMMIT",
  "git_dirty": $GIT_DIRTY,
  "robot_id": "$ROBOT",
  "scan": "$SCAN",
  "scan_merge": $SCAN_MERGE_RECORD,
  "render": "$RENDER",
  "ROS_DOMAIN_ID": "$ROS_DOMAIN_ID",
  "GZ_PARTITION": "$GZ_PARTITION",
  "note": "light_collision·wheel slip1은 임시 사본 조건이며 운영 모델이 아니고 지도 채택(--adopt) 대상이 아니다"
}
EOF

say "환경: ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION 렌더링=$RENDER 월드=$WORLD_CONDITION 바퀴=$WHEEL_CONDITION"

# ── 2. 시뮬레이션 ────────────────────────────────────────────────────
say "시뮬레이션 시작 (로그 $SESSION/sim.log)"
start sim ros2 launch ax_control simulation.launch.py world:="${WORLD:-aws_warehouse.world}" robot_ids:="$ROBOT" gui:="$GUI"
wait_topic "/ax/$ROBOT/diff_drive_controller/odom" "odom" 300

# ── 3. B1 LiDAR 전처리 ───────────────────────────────────────────────
if [ "$SCAN" != raw ]; then
  say "B1 센서 처리 시작"
  start b1 ros2 launch ax_perception sensors.launch.py robot_ids:="$ROBOT"
  wait_topic "/ax/$ROBOT/sensors/front_lidar/scan_filtered" "B1 scan_filtered" 120
fi

# ── 4. SLAM ─────────────────────────────────────────────────────────
say "SLAM 시작"
start slam ros2 launch ax_localization mapping.launch.py robot_id:="$ROBOT" scan:="$SCAN"

if [ "$SCAN" = merged ]; then
  wait_topic "/ax/$ROBOT/sensors/rear_lidar/scan_filtered" "B1 뒤 scan_filtered" 120
  wait_topic "/ax/$ROBOT/sensors/merged/scan_filtered" "앞·뒤 병합 스캔" 120
fi
wait_topic "/ax/$ROBOT/map" "SLAM 지도" 180

# ── 5. 주행 → 비교 ───────────────────────────────────────────────────
# 주행 전 관문(선택): 화면·지도 준비를 사람이 확인한 뒤 Enter로 주행을 시작한다. q면 주행 없이 정리한다.
if [ "$WAIT_BEFORE_DRIVE" = 1 ]; then
  say "주행 전 대기: Gazebo·지도 준비를 확인한 뒤 Enter로 주행 시작, q 입력 시 주행 없이 종료"
  # EOF·입력 오류는 주행으로 넘어가지 않고 종료한다.
  if ! read -r ANSWER; then
    say "입력을 읽지 못함(EOF·입력 오류): 주행 없이 종료"
    exit 1
  fi

  if [ "$ANSWER" = q ]; then
    say "주행 없이 종료(사용자 선택)"
    exit 0
  fi
fi

say "지도 작성 주행 시작. 중단하려면 Ctrl+C (odom 기준 정지 확인 뒤 종료)"
DRIVE_ARGS=(--output "$RUN" --scan "$SCAN" --gz-world "$GZ_WORLD" --robot-id "$ROBOT")
[ -n "$ROUTE" ] && DRIVE_ARGS+=(--route "$ROUTE")
[ -n "$WORLD" ] && DRIVE_ARGS+=(--world "$WORLD")

[ -n "$GATE_DIR" ] && DRIVE_ARGS+=(--gate-dir "$GATE_DIR")

if [ -n "$WAYPOINTS" ]; then
  DRIVE_ARGS+=(--max-waypoints "$WAYPOINTS")
  [ "$SAVE_PARTIAL" = 1 ] || DRIVE_ARGS+=(--no-save)
fi
# 주행·기록 프로세스도 추적 가능한 프로세스 그룹으로 띄운다(정리 시 그룹·소유 표식으로 찾는다).
# Ctrl+C는 세션 셸이 받아 주행 그룹에 SIGINT를 전달하고, 주행 도구가 정지 확인을 마칠 때까지 기다린다.
start drive python3 "$WS/src/ax_localization/scripts/mapping_drive.py" "${DRIVE_ARGS[@]}"
DRIVE_PID="$SESSION_PID_drive"
tail -n +1 -F --pid="$DRIVE_PID" "$SESSION/drive.log" 2>/dev/null &
TAIL_PID=$!
wait "$DRIVE_PID"
DRIVE=$?
DRIVE_PID=

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
