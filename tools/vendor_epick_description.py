#!/usr/bin/env python3
# 고정된 외부 commit의 description만 선별 도입한다. 라이선스·원본 해시를 보존하고 하드웨어 드라이버는 포함하지 않는다.
"""Vendor the design-approved description subset, never its hardware driver."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / 'log/vendor_epick'
COMMIT = 'd07391307d2d16cacf88daa168ce02839589397e'

# upstream HEAD가 고정 commit인지 먼저 확인하고 description·원본 증빙·라이선스·해시를 갱신한다.
def main():
    if subprocess.check_output(['git', '-C', str(UPSTREAM), 'rev-parse', 'HEAD'], text=True).strip() != COMMIT:
        raise SystemExit('Unexpected upstream commit')
    package = ROOT / 'src/epick_description'
    package.mkdir(parents=True, exist_ok=True)
    files = ['LICENSE', 'epick_description/meshes/visual/epick_body.stl',
             'epick_description/urdf/epick_body.xacro', 'epick_description/urdf/helper_macros.xacro',
             'epick_description/urdf/epick_single_suction_cup.xacro',
             'epick_description/CMakeLists.txt', 'epick_description/package.xml']
    manifest = {'repository': 'https://github.com/PickNikRobotics/ros2_epick_gripper',
                'commit': COMMIT, 'license': 'BSD-3-Clause', 'files': []}
    for relative in files:
        source = UPSTREAM / relative
        if relative == 'LICENSE':
            destination = package / 'LICENSE'
        elif relative.endswith('.stl') or relative.endswith('/helper_macros.xacro'):
            destination = package / relative.removeprefix('epick_description/')
        else:
            destination = package / 'upstream' / (Path(relative).name + '.original')
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        manifest['files'].append({'upstream_path': relative, 'vendored_path': destination.relative_to(package).as_posix(),
                                  'sha256': hashlib.sha256(source.read_bytes()).hexdigest()})
    (package / 'urdf').mkdir(exist_ok=True)
    body = (UPSTREAM / 'epick_description/urdf/epick_body.xacro').read_text()
    body = body.replace("prefix:=^|''", "prefix:=^|'' virtual_mass:=0.71")
    body = body.replace('<link name="${body_link}">', '''<link name="${body_link}">
      <!-- AX simulation mass assumption; upstream provides no product inertia. -->
      <inertial><mass value="${virtual_mass}"/>
        <inertia ixx="${virtual_mass*(3*body_collision_radius**2+body_length**2)/12}"
                 iyy="${virtual_mass*(3*body_collision_radius**2+body_length**2)/12}"
                 izz="${virtual_mass*body_collision_radius**2/2}" ixy="0" ixz="0" iyz="0"/>
      </inertial>''')
    (package / 'urdf/epick_body.xacro').write_text(body)
    manifest['adaptations'] = ['epick_body.xacro: add configurable virtual cylinder mass/inertia only',
        'Hardware driver and MoveIt virtual vacuum joint are excluded from active macros',
        'AX cup, ToF mount and TCP are defined in ax_description/urdf/epick_adapter.xacro']
    manifest['adapted_sha256'] = {name: hashlib.sha256((package/name).read_bytes()).hexdigest()
                                 for name in ('urdf/epick_body.xacro', 'urdf/helper_macros.xacro')}
    (package / 'UPSTREAM_FILES.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Vendored EPick description at', COMMIT)

if __name__ == '__main__':
    main()
