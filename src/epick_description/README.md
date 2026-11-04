# EPick description subset

Source: PickNikRobotics/ros2_epick_gripper, commit `d07391307d2d16cacf88daa168ce02839589397e`.
BSD-3-Clause LICENSE and original source hashes are preserved in `UPSTREAM_FILES.json`.

The original body mesh and description helpers are retained, with upstream originals under `upstream/`. AX currently uses its own photo-inspired parcel suction head in `ax_description/meshes/tools/` under the existing `epick` selection; this is not the original EPick product geometry.
The body macro adds a configurable, explicitly virtual mass/inertia budget; upstream has no inertial specification.
AX can override the body's virtual centre of mass and inertia for tool comparison. The active AX configuration matches the Robotiq 2F-85 default finger pose, including cup and ToF mass; it is not a measured EPick specification.
AX defines the 12-cup head, ToF mount, and geometric TCP in `epick_adapter.xacro`.
The hardware driver, dummy pressure state and MoveIt vacuum revolute joint are not loaded.
Cup dimensions and TCP stroke compensation are development settings, not final measured product calibration.
