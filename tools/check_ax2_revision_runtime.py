#!/usr/bin/env python3
"""Smoke check: cargo camera/guard removed; drive and front sensors retained."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]

def main():
    os.environ.update(ROS_DOMAIN_ID='101',GZ_PARTITION='ax-camera-removal-'+uuid.uuid4().hex,
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',LIBGL_ALWAYS_SOFTWARE='1')
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    from controller_manager_msgs.srv import ListControllers
    rclpy.init();node=rclpy.create_node('ax2_revision_probe');data={}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,'/ax/robot_2/robot_description',lambda m:data.update(description=m.data),qos)
    node.create_subscription(JointState,'/ax/robot_2/joint_states',lambda m:data.update(joints=list(m.name)),qos_profile_sensor_data)
    client=node.create_client(ListControllers,'/ax/robot_2/controller_manager/list_controllers')
    out=ROOT/'log/ax_revision_runtime';out.mkdir(exist_ok=True)
    log=(out/'camera_removed.log').open('w')
    proc=subprocess.Popen(['ros2','launch','ax_control','simulation.launch.py','robot_ids:=robot_2','gui:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    result=dict(passed=False)
    def wait(predicate,timeout=100):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if proc.poll() is not None:raise RuntimeError('Launch exited')
            if predicate():return
        raise TimeoutError(str(data.keys()))
    try:
        wait(lambda:'description' in data and len(data.get('joints',[]))>=4 and client.service_is_ready())
        deadline=time.monotonic()+40
        while True:
            future=client.call_async(ListControllers.Request());wait(future.done,15)
            states={c.name:c.state for c in future.result().controller}
            if all(states.get(k)=='active' for k in ('joint_state_broadcaster','diff_drive_controller')):break
            if time.monotonic()>deadline:raise TimeoutError(str(states))
        assert 'cargo_rgbd' not in data['description']
        assert 'front_rgbd' in data['description'] and 'cargo_contact' in data['description']
        topics=[t for t,_ in node.get_topic_names_and_types()]
        services=[s for s,_ in node.get_service_names_and_types()]
        assert not any('/cargo_rgbd/' in t or '/loading/' in t for t in topics)
        assert '/ax/robot_2/loading/check' not in services
        result=dict(passed=True,controllers=states,cargo_camera_absent=True,height_guard_absent=True,front_camera_and_contact_retained=True)
    except Exception as error:result['error']=repr(error)
    finally:
        node.destroy_node();rclpy.try_shutdown()
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close()
    (out/'camera_removed.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
