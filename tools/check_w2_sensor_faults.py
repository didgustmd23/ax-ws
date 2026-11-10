#!/usr/bin/env python3
"""B1 ROS transport test: synthetic inputs, isolated domain, no Gazebo or drive.

Start sensors.launch.py robot_ids:=robot_2 in the same isolated ROS domain.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('log/w2/sensor_faults.json'))
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('w2_synthetic_sensor_inputs')
    root='/ax/robot_2/sensors/'
    clock=node.create_publisher(Clock,'/clock',10)
    pubs={key:node.create_publisher(cls,root+suffix,SENSOR) for key,cls,suffix in [
        ('rgb',Image,'front_rgb/image_raw'),('image',Image,'front_rgbd/image'),
        ('depth_image',Image,'front_rgbd/depth_image'),('camera_info',CameraInfo,'front_rgbd/camera_info'),
        ('scan',LaserScan,'front_lidar/scan')]}
    outputs={'rgb':[],'cloud':[],'scan':[]};health={};cases=[]
    for key,cls,suffix in [('rgb',Image,'front_rgb/processed/image_raw'),
                          ('cloud',PointCloud2,'front_rgbd/processed/points'),
                          ('scan',LaserScan,'front_lidar/scan_filtered')]:
        node.create_subscription(cls,root+suffix,lambda m,key=key:outputs[key].append(m),SENSOR)
    node.create_subscription(DiagnosticArray,root+'health',
        lambda m:health.update({s.name:s for s in m.status}),10)
    current=10.
    def pump(seconds=.25):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            clock.publish(Clock(clock=Time(seconds=current).to_msg()))
            rclpy.spin_once(node,timeout_sec=.02)
    def rgb(stamp,frame='robot_2/front_rgb_optical_frame'):
        m=Image(width=2,height=1,step=6,encoding='rgb8',data=bytes([255,0,0,0,255,0]))
        m.header.stamp=Time(seconds=stamp).to_msg();m.header.frame_id=frame
        return m
    def check(name,condition,detail=None):cases.append({'name':name,'passed':bool(condition),'detail':detail})
    try:
        pump(2.)
        pubs['rgb'].publish(rgb(10.));pump()
        check('normal_rgb_preserves_stamp',len(outputs['rgb'])==1 and outputs['rgb'][-1].header.stamp.sec==10)
        for name,stamp,frame in [('duplicate',10.,'robot_2/front_rgb_optical_frame'),
            ('reversed',9.99,'robot_2/front_rgb_optical_frame'),('stale',9.,'robot_2/front_rgb_optical_frame'),
            ('future',10.1,'robot_2/front_rgb_optical_frame'),('wrong_robot_frame',10.01,'robot_1/front_rgb_optical_frame')]:
            before=len(outputs['rgb']);pubs['rgb'].publish(rgb(stamp,frame));pump(.6)
            row=health.get('robot_2/front_rgb/image_raw')
            check(name,len(outputs['rgb'])==before and row is not None and row.level==DiagnosticStatus.ERROR,
                row.message if row else 'missing diagnostic')
        def triple(stamp,skew=0.,invalid=False):
            image=rgb(stamp,'robot_2/front_rgbd_optical_frame')
            depth=Image(header=deepcopy(image.header),width=2,height=1,step=8,encoding='32FC1',
                data=np.asarray([np.nan,np.inf] if invalid else [1.,2.],dtype='<f4').tobytes())
            info=CameraInfo(header=deepcopy(image.header),width=2,height=1,
                k=[2.,0.,1.,0.,2.,0.,0.,0.,1.],p=[2.,0.,1.,0.,0.,2.,0.,0.,0.,0.,1.,0.])
            info.header.stamp=Time(seconds=stamp+skew).to_msg()
            for key,message in [('image',image),('depth_image',depth),('camera_info',info)]:pubs[key].publish(message)
        current=11.;pump();triple(11.,.04);pump(.6)
        check('rgbd_40ms_skew_no_cloud',not outputs['cloud'])
        current=12.;pump();triple(12.,.02);pump(.6)
        check('rgbd_20ms_skew_recovers',len(outputs['cloud'])==1)
        current=13.;pump();triple(13.,invalid=True);pump(.6)
        row=health.get('robot_2/front_rgbd/depth_valid')
        check('invalid_depth_published_but_diagnosed',len(outputs['cloud'])==2 and not outputs['cloud'][-1].is_dense
            and row is not None and row.level==DiagnosticStatus.ERROR,row.message if row else None)
        scan=LaserScan(angle_min=0.,angle_max=.3,angle_increment=.1,range_min=.12,range_max=10.,
            ranges=[.01,1.,11.,float('nan')]);scan.header.stamp=Time(seconds=13.).to_msg()
        scan.header.frame_id='robot_2/front_lidar_link';pubs['scan'].publish(scan);pump()
        values=list(outputs['scan'][-1].ranges) if outputs['scan'] else []
        check('scan_near_far_nan_preserved',len(values)==4 and np.isneginf(values[0]) and values[1]==1.
              and np.isposinf(values[2]) and np.isnan(values[3]))
        current=1.;pump(.6);pubs['rgb'].publish(rgb(1.));pump(.6)
        check('clock_reset_accepts_new_session_stamp',len(outputs['rgb'])==2)
        pump(2.6);row=health.get('robot_2/front_rgb/image_raw')
        check('missing_input_stale_with_frozen_sim_clock',row is not None and row.level==DiagnosticStatus.STALE)
        current=1.01;pump();pubs['rgb'].publish(rgb(1.01));pump(.6)
        row=health.get('robot_2/front_rgb/image_raw')
        check('fresh_input_recovers',len(outputs['rgb'])==3 and row is not None and row.level==DiagnosticStatus.OK)
        result={'passed':all(c['passed'] for c in cases),'scope':'synthetic ROS transport, not physical calibration', 'cases':cases}
    except Exception as exc:result={'passed':False,'error':str(exc),'cases':cases}
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(result,ensure_ascii=False));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
