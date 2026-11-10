#!/usr/bin/env python3
"""D1 isolated configuration test with labelled MOCK map/TF/scans/odom.

Planning actions only; no driving goals or controller commands. Run navigation.launch.py in the same
isolated domain first. Mock inputs must never be mixed with real operation.
"""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data as SENSOR
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('log/w2/navigation_mock.json'))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('w2_mock_navigation_inputs')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    map_pub = node.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = .05
    grid.info.width = grid.info.height = 400
    grid.info.origin.position.x = grid.info.origin.position.y = -10.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*(400*400)
    map_pub.publish(grid)
    dynamic, static = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    scans, odoms, fixed = {}, {}, []
    for index, robot in enumerate(('robot_1','robot_2')):
        root = '/ax/'+robot
        odoms[robot] = node.create_publisher(Odometry, root+'/diff_drive_controller/odom', 10)
        for name, x, yaw in [('front',.77 if robot=='robot_1' else .61,0.),
                              ('rear',-.73 if robot=='robot_1' else -.57,math.pi)]:
            scans[robot,name] = node.create_publisher(LaserScan, root+'/sensors/'+name+'_lidar/scan_filtered', SENSOR)
            transform = TransformStamped()
            transform.header.frame_id = robot+'/base_link'
            transform.child_frame_id = robot+'/'+name+'_lidar_link'
            transform.transform.translation.x = x
            transform.transform.translation.z = .2
            transform.transform.rotation.z = math.sin(yaw/2)
            transform.transform.rotation.w = math.cos(yaw/2)
            fixed.append(transform)
    static.sendTransform(fixed)
    started = time.monotonic()
    def tick():
        stamp = Duration(seconds=time.monotonic()-started+1.).to_msg()
        clock = Clock(); clock.clock.sec=stamp.sec;clock.clock.nanosec=stamp.nanosec
        clock_pub.publish(clock)
        transforms = []
        for index, robot in enumerate(('robot_1','robot_2')):
            for parent,child,y in [('map',robot+'/odom',float(index*3)),(robot+'/odom',robot+'/base_link',0.)]:
                t = TransformStamped();t.header.stamp=clock.clock;t.header.frame_id=parent;t.child_frame_id=child
                t.transform.translation.y=y;t.transform.rotation.w=1.;transforms.append(t)
            odom = Odometry();odom.header.stamp=clock.clock;odom.header.frame_id=robot+'/odom'
            odom.child_frame_id=robot+'/base_link';odom.pose.pose.orientation.w=1.;odoms[robot].publish(odom)
            for name in ('front','rear'):
                scan=LaserScan();scan.header.stamp=clock.clock;scan.header.frame_id=robot+'/'+name+'_lidar_link'
                scan.angle_min=-math.pi/2;scan.angle_max=math.pi/2;scan.angle_increment=math.pi/359
                scan.range_min=.12;scan.range_max=10.;scan.ranges=[math.inf]*360
                scans[robot,name].publish(scan)
        dynamic.sendTransform(transforms)
    timer=node.create_timer(.05,tick)
    result={'passed':False,'scope':'MOCK inputs; planning only, no driving goals, physical motion, B2/A4 integration or G1 judgment',
            'robots':{},'checks':{}}
    def call(client,request,timeout=40):
        deadline=time.monotonic()+timeout
        while not client.service_is_ready() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        if not client.service_is_ready():raise RuntimeError('Service unavailable: '+client.srv_name)
        future=client.call_async(request)
        while not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        if not future.done():raise RuntimeError('Service timeout: '+client.srv_name)
        return future.result()
    try:
        for robot in ('robot_1','robot_2'):
            root='/ax/'+robot
            manager=node.create_client(ManageLifecycleNodes,root+'/lifecycle_manager_navigation/manage_nodes')
            response=call(manager,ManageLifecycleNodes.Request(command=0),timeout=90)
            if not response.success:raise RuntimeError(robot+' lifecycle startup failed')
            states={}
            for name in ('controller_server','planner_server','bt_navigator'):
                client=node.create_client(GetState,root+'/'+name+'/get_state')
                states[name]=call(client,GetState.Request()).current_state.label
            get=node.create_client(GetParameters,root+'/global_costmap/global_costmap/get_parameters')
            values=call(get,GetParameters.Request(names=['robot_base_frame','footprint','static_layer.map_topic']))
            result['robots'][robot]={'states':states,'global_base_frame':values.values[0].string_value,
                'footprint':values.values[1].string_value,'map_topic':values.values[2].string_value}
            # Planning only: no NavigateToPose or FollowPath goal is submitted.
            planner=ActionClient(node,ComputePathToPose,root+'/compute_path_to_pose')
            deadline=time.monotonic()+20
            while not planner.server_is_ready() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
            if not planner.server_is_ready():raise RuntimeError(robot+' planner unavailable')
            goal=ComputePathToPose.Goal();goal.goal.header.frame_id='map'
            goal.goal.pose.position.x=2.;goal.goal.pose.position.y=0. if robot=='robot_1' else 3.
            goal.goal.pose.orientation.w=1.;goal.planner_id='GridBased'
            future=planner.send_goal_async(goal)
            while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            if not future.done() or not future.result().accepted:raise RuntimeError('Planning goal rejected')
            done=future.result().get_result_async()
            while not done.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            if not done.done():raise RuntimeError('Planning timed out')
            response=done.result();path=response.result.path
            result['robots'][robot]['plan']={'status':response.status,'pose_count':len(path.poses),
                'frame':path.header.frame_id,'start_y':path.poses[0].pose.position.y if path.poses else None,
                'end_x':path.poses[-1].pose.position.x if path.poses else None}
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        topics=dict(node.get_topic_names_and_types())
        for robot in ('robot_1','robot_2'):
            root='/ax/'+robot
            entries=node.get_publishers_info_by_topic(root+'/nav/cmd_vel')
            direct=node.get_publishers_info_by_topic(root+'/diff_drive_controller/cmd_vel')
            result['checks'][robot]={'command_types':topics.get(root+'/nav/cmd_vel',[]),
                'nav_publishers':[e.node_name for e in entries],
                'direct_controller_publishers':[e.node_name for e in direct],
                'navigate_to_pose_services_present':any('/navigate_to_pose/_action/' in name and root in name
                    for name,_ in node.get_service_names_and_types())}
        result['passed']=all(all(v=='active' for v in row['states'].values())
            and row['global_base_frame']==robot+'/base_link' and row['map_topic']=='/map'
            and row['plan']['status']==4 and row['plan']['pose_count']>=2
            and row['plan']['frame']=='map'
            and abs(row['plan']['start_y']-(0. if robot=='robot_1' else 3.))<.1
            and abs(row['plan']['end_x']-2.)<.1
            for robot,row in result['robots'].items()) and all(
                row['command_types']==['geometry_msgs/msg/TwistStamped'] and not row['direct_controller_publishers']
                and row['navigate_to_pose_services_present'] for row in result['checks'].values())
    except Exception as exc:result['error']=str(exc)
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
