"""B2 비교용 선택 모드: 실행 중인 시뮬레이션(또는 기록 재생)에 RTAB-Map LiDAR·RGB-D 융합 지도 작성만 연결한다.

  ros2 launch ax_localization mapping_rtabmap.launch.py [robot_id:=robot_2] [scan:=front|merged]
      [config:=<rtabmap 설정 yaml>] [database:=<db 경로>] [preflight:=true|false] [start_merge:=true|false]

- 운영 기준 지도 작성은 mapping.launch.py(slam_toolbox)다. 이 launch는 채택 전 비교 시험용이며 기본 동작을 바꾸지 않는다.
- 입력: B1 처리본 front_rgbd(processed/image·depth_image·camera_info) → rgbd_sync → rtabmap.
  scan:=front는 sensors/front_lidar/scan_filtered,
  scan:=merged는 scan_merge.py의 sensors/merged/scan_filtered(앞·뒤 병합)다. start_merge:=true면 이 launch가 병합 노드도 띄운다.
- 프레임: <robot>/base_link, <robot>/odom(TF), map. RTAB-Map이 map→<robot>/odom을 발행한다.
- preflight:=true(기본)는 mapping.launch.py와 같은 검사로 같은 로봇의 slam_toolbox·amcl·map→odom 발행이 있으면 시작하지 않는다.
  기록 재생처럼 robot_state_publisher가 없는 환경에서만 false로 둔다.
- database를 비우면 실행마다 새 임시 폴더에 db를 만들고 그 db에만 시작 시 삭제(-d)를 건다.
  database:=<경로>를 주면 -d를 넘기지 않으며, 그 경로에 파일이 이미 있으면 지우거나 이어 쓰지 않도록 시작하지 않는다.
"""

import importlib.util
from pathlib import Path
import tempfile  # noqa: I100

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_IDS = ("robot_1", "robot_2", "robot_3", "robot_4")
SCANS = {"front": "sensors/front_lidar/scan_filtered", "merged": "sensors/merged/scan_filtered"}
BOOLEANS = {"true": True, "false": False}


# launch 인자를 검사해 (robot_id, scan 토픽, preflight, start_merge)로 돌려준다. 잘못된 값은 ValueError.
def validate(robot_id, scan, preflight, start_merge):
    if robot_id not in ROBOT_IDS:
        raise ValueError(f"robot_id는 {', '.join(ROBOT_IDS)} 중 하나")

    if scan not in SCANS:
        raise ValueError("scan은 front 또는 merged")

    if preflight not in BOOLEANS or start_merge not in BOOLEANS:
        raise ValueError("preflight·start_merge는 true 또는 false")

    if start_merge == "true" and scan != "merged":
        raise ValueError("start_merge:=true는 scan:=merged와 함께만 쓴다")

    return robot_id, SCANS[scan], BOOLEANS[preflight], BOOLEANS[start_merge]


# db 경로와 rtabmap 실행 인자를 정한다. 비우면 새 임시 폴더 db + -d, 지정하면 -d 없이 쓰되 기존 파일이면 ValueError.
def database_plan(database, make_temp=lambda: tempfile.mkdtemp(prefix="ax_rtabmap_")):
    if not database:
        return str(Path(make_temp()) / "rtabmap.db"), ["-d"]

    if Path(database).exists():
        raise ValueError(f"database 경로에 파일이 이미 있음: {database}. 기존 db를 지우거나 이어 쓰지 않도록 새 경로를 준다")

    return database, []


# 같은 폴더의 mapping.launch.py 모듈(map→odom 발행 후보 검사·병합 노드 생성·병합 중복 검사를 한 곳에서 공유).
def mapping_launch():
    spec = importlib.util.spec_from_file_location("ax_mapping_launch", Path(__file__).with_name("mapping.launch.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# map→odom 발행 후보 검사(mapping.launch.py와 같은 규칙)
def running_localization(namespace):
    return mapping_launch().running_localization(namespace)


# 인자를 검사하고 rgbd_sync·rtabmap(필요하면 scan_merge) 노드를 만든다.
def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot_id, scan_topic, preflight, start_merge = validate(value("robot_id"), value("scan"), value("preflight"), value("start_merge"))
    namespace = f"/ax/{robot_id}"

    if preflight:
        running = running_localization(namespace)
        if running:
            raise RuntimeError(f"이미 map→odom 발행 후보가 실행 중: {', '.join(running)}. 하나만 실행한다")

        # 병합 노드를 띄울 때는 mapping.launch.py와 같은 규칙으로 같은 namespace의 기존 scan_merge를 거절한다.
        if start_merge and mapping_launch().running_scan_merge(namespace):
            raise RuntimeError(f"{namespace}/scan_merge가 이미 실행 중. 병합 토픽 발행자는 하나여야 하므로 시작하지 않는다")

    config = value("config") or get_package_share_directory("ax_localization") + "/config/rtabmap_mapping.yaml"
    database, rtabmap_arguments = database_plan(value("database"))
    processed = "sensors/front_rgbd/processed"

    sync = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync", namespace=namespace, output="screen",
        parameters=[config, {"use_sim_time": True}],
        remappings=[("rgb/image", f"{processed}/image"), ("depth/image", f"{processed}/depth_image"),
                    ("rgb/camera_info", f"{processed}/camera_info"), ("rgbd_image", "rtabmap/rgbd_image")],
    )

    rtabmap = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace=namespace, output="screen",
        parameters=[config, {
            "use_sim_time": True,
            "frame_id": f"{robot_id}/base_link",
            "odom_frame_id": f"{robot_id}/odom",
            "map_frame_id": "map",
            "database_path": database,
        }],
        remappings=[("rgbd_image", "rtabmap/rgbd_image"), ("scan", scan_topic)],
        arguments=rtabmap_arguments,
    )

    actions = [LogInfo(msg=f"[ax_localization] RTAB-Map 비교 모드 {namespace}, scan {scan_topic}, db {database}"), sync, rtabmap]

    if start_merge:
        actions.append(mapping_launch().scan_merge_node(namespace, robot_id))
    return actions


# launch 인자 선언
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="robot_2"),
        DeclareLaunchArgument("scan", default_value="front", description="front(앞 LiDAR) 또는 merged(앞·뒤 병합)"),
        DeclareLaunchArgument("config", default_value="", description="비우면 설치된 config/rtabmap_mapping.yaml"),
        DeclareLaunchArgument("database", default_value="", description="비우면 실행마다 새 임시 폴더(-d), 지정 시 새 파일 경로만"),
        DeclareLaunchArgument("preflight", default_value="true", description="map→odom 발행자 중복 검사"),
        DeclareLaunchArgument("start_merge", default_value="false", description="true면 scan_merge.py도 띄움(scan:=merged 필요)"),
        OpaqueFunction(function=setup),
    ])
