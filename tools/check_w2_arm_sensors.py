#!/usr/bin/env python3
"""B1 arm RGB-D/ToF known-plane consistency, idle A2 simulation robot_1/epick.

Targets are derived from world pose + measured-time TF (not independent arm
extrinsic calibration). Sends no commands; creates/removes only its own boards.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time
import uuid
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rclpy.time import Time
from sensor_msgs.msg import Image, Range
from tf2_ros import Buffer, TransformListener
from gz.transport13 import Node as GazeboNode
from gz.msgs10.pose_v_pb2 import Pose_V
from scipy.spatial.transform import Rotation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('log/w2/arm_sensors.json'))
    args=parser.parse_args();rclpy.init()
    node=rclpy.create_node('w2_arm_sensor_check',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    transport=GazeboNode();poses={};latest={};cases=[]
    def poses_received(message):
        for p in message.pose:
            if p.name=='robot_1':poses['base']=p
    transport.subscribe(Pose_V,'/world/ax_test/dynamic_pose/info',poses_received)
    root='/ax/robot_1/sensors/'
    for key,cls,suffix in [('depth',Image,'arm_rgbd/processed/depth_image'),
                          ('raw',Range,'tool_tof/raw'),('tof',Range,'tool_tof')]:
        node.create_subscription(cls,root+suffix,lambda m,key=key:latest.update({key:m}),SENSOR)
    def wait(predicate,seconds=60):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.03)
            if predicate():return
        raise RuntimeError('Sensor/TF observations timed out')
    def transform(message):
        t=buffer.lookup_transform('robot_1/base_link',message.header.frame_id,
                                  Time.from_msg(message.header.stamp)).transform
        q=t.rotation;r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        p=poses['base'];q=p.orientation;world=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        origin=np.array([p.position.x,p.position.y,p.position.z])+world@np.array([t.translation.x,t.translation.y,t.translation.z])
        return origin,world@r
    def remove(name):
        subprocess.run(['gz','service','-s','/world/ax_test/remove','--reqtype','gz.msgs.Entity',
            '--reptype','gz.msgs.Boolean','--timeout','3000','--req',f'name: "{name}" type: MODEL'],
            capture_output=True,timeout=8,check=True)
    active=None;result={'passed':False,'scope':'ideal ray/depth and TF consistency; no hardware/independent arm calibration',
                         'tolerance_m':.02,'cases':cases}
    try:
        wait(lambda:'base' in poses and len(latest)==3)
        # At neutral, the centre ray sees foreground geometry around 0.198 m.
        # Test unoccluded near targets; the initial 0.25 m failure is retained in evidence.
        for key,distances in [('depth',(.10,.15)),('raw',(.08,.15))]:
            for distance in distances:
                message=latest[key]
                wait(lambda:buffer.can_transform('robot_1/base_link',latest[key].header.frame_id,Time.from_msg(latest[key].header.stamp)))
                origin,rotation=transform(latest[key])
                # Board local X is its normal; camera optical +Z / ToF +X is the ray.
                axis=rotation[:,2] if key=='depth' else rotation[:,0]
                board_rotation=rotation@Rotation.from_euler('y',-np.pi/2).as_matrix() if key=='depth' else rotation
                surface=origin+axis*distance;center=surface+axis*.005
                angles=Rotation.from_matrix(board_rotation).as_euler('xyz')
                active='w2_arm_board_'+uuid.uuid4().hex[:8]
                sdf=f'<sdf version="1.8"><model name="{active}"><static>true</static><link name="board"><collision name="collision"><geometry><box><size>.01 .12 .12</size></box></geometry></collision><visual name="visual"><geometry><box><size>.01 .12 .12</size></box></geometry></visual></link></model></sdf>'
                # camera board local X must point along +optical Z
                # sign of the normal is immaterial to plane geometry.
                subprocess.run(['ros2','run','ros_gz_sim','create','-world','ax_test','-name',active,'-string',sdf,
                    '-x',str(center[0]),'-y',str(center[1]),'-z',str(center[2]),
                    '-R',str(angles[0]),'-P',str(angles[1]),'-Y',str(angles[2])],capture_output=True,timeout=20,check=True)
                after=latest[key].header.stamp.sec+latest[key].header.stamp.nanosec*1e-9+.4
                samples=[];seen=set()
                def collect():
                    msg=latest[key];stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    if stamp<=after or stamp in seen:return False
                    if not buffer.can_transform('robot_1/base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp)):return False
                    current,r=transform(msg);ray=r[:,2] if key=='depth' else r[:,0]
                    expected=float(np.dot(surface-current,axis)/np.dot(ray,axis))
                    if key=='depth':
                        measured=float(np.ndarray((msg.height,msg.width),dtype='<f4',buffer=msg.data,
                            strides=(msg.step,4))[msg.height//2,msg.width//2])
                        processed=measured
                    else:
                        if latest['tof'].header.stamp!=msg.header.stamp:return False
                        measured=float(msg.range);processed=float(latest['tof'].range)
                    good=bool(np.isfinite(measured) and np.isfinite(processed) and abs(measured-expected)<=.02
                              and abs(processed-measured)<1e-5)
                    samples.append({'stamp_s':stamp,'expected_m':expected,'measured_m':measured if np.isfinite(measured) else None,
                                    'processed_m':processed if np.isfinite(processed) else None,'passed':good});seen.add(stamp)
                    return len(samples)>=3
                wait(collect)
                cases.append({'sensor':key,'nominal_distance_m':distance,'samples':samples,
                              'passed':all(s['passed'] for s in samples)})
                remove(active);active=None
        result['passed']=all(c['passed'] for c in cases)
    except Exception as exc:result['error']=str(exc)
    finally:
        if active:remove(active)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(result,ensure_ascii=False));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
