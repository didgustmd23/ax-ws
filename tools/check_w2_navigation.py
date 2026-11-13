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
    parser.add_argument('--scenario', choices=['baseline', 'obstacle', 'blocked', 'missing_map',
                                               'missing_tf', 'missing_scan', 'stale_tf', 'stale_scan'],
                        default='baseline')
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
    if args.scenario in ('obstacle', 'blocked'):
        # A wall across the map blocks all routes; a short wall must be detoured.
        for row in range(grid.info.height):
            y = -10. + (row + .5) * .05
            if args.scenario == 'blocked' or abs(y) < .4 or abs(y - 3.) < .4:
                for column in range(218, 222):
                    grid.data[row * grid.info.width + column] = 100
    if args.scenario != 'missing_map':
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
    scan_enabled, tf_enabled = [args.scenario != 'missing_scan'], [args.scenario != 'missing_tf']
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
                if scan_enabled[0]:
                    scans[robot,name].publish(scan)
        if tf_enabled[0]:
            dynamic.sendTransform(transforms)
    timer=node.create_timer(.05,tick)
    result={'passed':False,'scope':'MOCK inputs; planning only, no driving goals, physical motion, B2/A4 integration or G1 judgment',
            'scenario':args.scenario, 'robots':{},'checks':{}}
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
        if args.scenario == 'missing_tf':
            manager = node.create_client(ManageLifecycleNodes,
                '/ax/robot_1/lifecycle_manager_navigation/manage_nodes')
            end = time.monotonic() + 15
            while not manager.service_is_ready() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            if not manager.service_is_ready():
                raise RuntimeError('Lifecycle manager unavailable, not an input-fault result')
            future = manager.call_async(ManageLifecycleNodes.Request(command=0))
            end = time.monotonic() + 8
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            names = [name for name, ns in node.get_node_names_and_namespaces() if ns == '/ax/robot_1']
            result['checks']['startup_with_missing_input'] = {
                'observed_wall_s': 8, 'startup_completed': future.done(),
                'startup_success': future.result().success if future.done() else None,
                'visible_nodes': sorted(names),
                'scope': 'Startup must not complete without required input; no motion goals submitted.'}
            result['passed'] = ('planner_server' in names and
                                (not future.done() or not future.result().success))
            return 0 if result['passed'] else 1
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
            if args.scenario in ('stale_scan', 'stale_tf'):
                initial = planner.send_goal_async(goal)
                end = time.monotonic() + 20
                while not initial.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.05)
                if not initial.done() or not initial.result().accepted:
                    raise RuntimeError('Pre-fault planning request not accepted')
                initial_result = initial.result().get_result_async()
                while not initial_result.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.05)
                if not initial_result.done() or initial_result.result().status != 4:
                    raise RuntimeError('Pre-fault path must succeed before injecting staleness')
                result['checks']['pre_fault_plan'] = {'status': 4,
                    'pose_count': len(initial_result.result().result.path.poses)}
                if args.scenario == 'stale_scan':
                    scan_enabled[0] = False
                else:
                    tf_enabled[0] = False
                end = time.monotonic() + 1.5
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.05)
                deadline = time.monotonic() + 20
            if args.scenario in ('missing_scan', 'stale_scan', 'stale_tf'):
                end = time.monotonic() + 1.5
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.05)
            future=planner.send_goal_async(goal)
            while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            if not future.done() or not future.result().accepted:raise RuntimeError('Planning goal rejected')
            done=future.result().get_result_async()
            if args.scenario in ('missing_scan', 'stale_scan', 'stale_tf'):
                end = time.monotonic() + 8
                while not done.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.05)
                response = done.result() if done.done() else None
                result['checks'][args.scenario] = {
                    'result_received': done.done(), 'status': response.status if response else None,
                    'pose_count': len(response.result.path.poses) if response else None,
                    'scope': 'No successful path with missing/stale input; no driving action.'}
                result['passed'] = response is None or (response.status != 4 and not response.result.path.poses)
                if response is None:
                    canceled = future.result().cancel_goal_async()
                    end = time.monotonic() + 2
                    while not canceled.done() and time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=.05)
                    result['checks'][args.scenario]['cancel_ack_received'] = canceled.done()
                return 0 if result['passed'] else 1
            while not done.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            if not done.done():raise RuntimeError('Planning timed out')
            response=done.result();path=response.result.path
            result['robots'][robot]['plan']={'status':response.status,'pose_count':len(path.poses),
                'frame':path.header.frame_id,'start_y':path.poses[0].pose.position.y if path.poses else None,
                'end_x':path.poses[-1].pose.position.x if path.poses else None,
                'max_lateral_deviation_m':max((abs(p.pose.position.y - (0. if robot=='robot_1' else 3.))
                                                for p in path.poses), default=0.)}
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
            and ((row['plan']['status']!=4 and row['plan']['pose_count']==0) if args.scenario in ('blocked', 'missing_map') else
                 (row['plan']['status']==4 and row['plan']['pose_count']>=2
                  and row['plan']['frame']=='map'
                  and abs(row['plan']['start_y']-(0. if robot=='robot_1' else 3.))<.1
                  and abs(row['plan']['end_x']-2.)<.1
                  and (args.scenario!='obstacle' or row['plan']['max_lateral_deviation_m']>.4)))
            for robot,row in result['robots'].items()) and all(
                row['command_types']==['geometry_msgs/msg/TwistStamped'] and not row['direct_controller_publishers']
                and row['navigate_to_pose_services_present'] for row in result['checks'].values())
    except Exception as exc:result['error']=str(exc)
    finally:
        # The missing-input branch returns early; still persist its result.
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
