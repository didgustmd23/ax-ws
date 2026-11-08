#!/usr/bin/env bash
# 조건별로 임시 사본 xacro를 바꾸고 격리 시뮬레이션에서 slip_eval을 실행한다.
S=<tmp>; R=<workspace>/log/a2_rotation_slip/20261106_224038; D=$S/verify; X=<tmp>/overlay_contact/share/ax_description/urdf/ros2_control.xacro
cd ~/ax_ws; source /opt/ros/jazzy/setup.bash; source install/setup.bash
run() {
  name=$1; slip=$2
  if [ -z "$slip" ]; then cp $X.contact_base $X; else
    python3 - "$X.contact_base" "$X" "$slip" <<'PY'
import sys
from pathlib import Path
s = Path(sys.argv[1]).read_text()
old = '            <fdir1>0 0 1</fdir1>\n'
assert s.count(old) == 1
s = s.replace(old, old + f'            <collision><surface><friction><ode><slip1>{sys.argv[3]}</slip1></ode></friction></surface></collision>\n')
Path(sys.argv[2]).write_text(s)
PY
  fi
  cp $X $R/$name.ros2_control.xacro
  OVERLAY=$S/overlay_contact GPU=1 WORLD=ax_test.world SENSORS=false ROBOTS=robot_2 bash $D/sim.sh start slip_$name > /dev/null || { echo "$name sim fail"; return; }
  P=$(cat $D/slip_$name.partition)
  xacro_check=$(gz model -m robot_2 2>/dev/null | head -1)
  GZ_PARTITION=$P ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST timeout 2400 python3 $S/rotation/slip_eval.py $name $R/$name.json > $R/$name.log 2>&1
  echo "$name exit $?"
  bash $D/sim.sh stop slip_$name > /dev/null
}
for c in "$@"; do run ${c%%=*} $(echo $c | grep -q = && echo ${c#*=}); done
cp $X.contact_base $X
