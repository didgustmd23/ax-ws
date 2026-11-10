#!/usr/bin/env python3
"""현재 창고의 AX_2 생성·주행 제어기·깊이 높이 제한을 격리된 Gazebo에서 검사한다."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def run_world(world, index):
    os.environ.update(ROS_DOMAIN_ID=str(101+index), GZ_PARTITION='ax-revision-'+uuid.uuid4().hex,
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', LIBGL_ALWAYS_SOFTWARE='1')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String, Float32
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from controller_manager_msgs.srv import ListControllers
    rclpy.init()
    node = rclpy.create_node('ax2_revision_probe')
    readings = {}
    node.create_subscription(String,'/ax/robot_2/loading/status',lambda m: readings.update(status=json.loads(m.data)),10)
    node.create_subscription(Float32,'/ax/robot_2/loading/height_m',lambda m: readings.update(height=m.data),10)
    node.create_subscription(JointState,'/ax/robot_2/joint_states',lambda m: readings.update(joints=list(m.name)),qos_profile_sensor_data)
    check=node.create_client(Trigger,'/ax/robot_2/loading/check')
    controllers=node.create_client(ListControllers,'/ax/robot_2/controller_manager/list_controllers')
    out=ROOT/'log/ax_revision_runtime';out.mkdir(exist_ok=True)
    log=(out/f'world_{index}.log').open('w')
    launch=subprocess.Popen(['ros2','launch','ax_control','simulation.launch.py',
        'robot_ids:=robot_2','gui:=false','sensors:=true',f'world:={world}'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    result=dict(world=world,cases=[])
    def wait(predicate,seconds=100):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if launch.poll() is not None:
                raise RuntimeError('Launch exited unexpectedly')
            if predicate():return
        raise TimeoutError(str(readings))
    def call(client,request):
        wait(client.service_is_ready)
        future=client.call_async(request);wait(future.done,20)
        response=future.result()
        if response is None:raise RuntimeError('Service failed')
        return response
    def fixture(height):
        spec=json.loads((ROOT/'src/ax_description/config/ax2_spec.json').read_text())
        deck=spec['cargo']['deck_origin_in_base_m']
        # Ground height from the current wheel geometry; no artificial robot fixed joint.
        base_z=.149900663548-(.21296115-.33208145)
        x,y=(-6.8,0.)
        # 광학 시험 전용 외형이다. 고정 시험 물체가 차체를 밀지 않도록 충돌체는 두지 않는다.
        sdf='<sdf version="1.8"><model name="ax_revision_height_probe"><static>true</static><link name="box"><visual name="box"><geometry><box><size>.2 .2 .2</size></box></geometry></visual></link></model></sdf>'
        subprocess.run(['ros2','run','ros_gz_sim','create','-world','ax_test',
            '-name','ax_revision_height_probe','-string',sdf,'-x',str(x+deck[0]),'-y',str(y+deck[1]),
            '-z',str(base_z+deck[2]+height-.1)],check=True,stdout=log,stderr=log,timeout=25)
    def remove():
        response=subprocess.run(['gz','service','-s','/world/ax_test/remove',
            '--reqtype','gz.msgs.Entity','--reptype','gz.msgs.Boolean','--timeout','5000',
            '--req','name: "ax_revision_height_probe" type: MODEL'],capture_output=True,text=True,timeout=10)
        if 'data: true' not in response.stdout:raise RuntimeError('Fixture removal failed')
    try:
        wait(lambda: readings.get('status',{}).get('reason')=='READY' and len(readings.get('joints',[]))>=4)
        # 첫 응답은 시작 중 상태일 수 있어 두 제어기가 active가 될 때까지 기다린다.
        deadline=time.monotonic()+40
        while True:
            states={c.name:c.state for c in call(controllers,ListControllers.Request()).controller}
            if all(states.get(c)=='active' for c in ('joint_state_broadcaster','diff_drive_controller')):break
            if time.monotonic()>deadline:raise TimeoutError(str(states))
            for _ in range(5):rclpy.spin_once(node,timeout_sec=.1)
        assert call(check,Trigger.Request()).success
        baseline=readings.get('height',0.)
        assert 0<=baseline<.03,('Unexpected empty-deck height',baseline)
        result['cases'].append(dict(case='empty_ready_and_controllers',height_m=baseline,passed=True))
        for height,allowed in ((.3,True),(.6,False)):
            fixture(height)
            wait(lambda: math.isfinite(readings.get('height',float('nan'))) and abs(readings['height']-height)<.025,45)
            response=call(check,Trigger.Request())
            assert response.success==allowed,(height,readings,response)
            result['cases'].append(dict(case=f'height_{height}',height_m=readings['height'],allowed=response.success,passed=True))
            remove()
            wait(lambda: readings.get('status',{}).get('reason')=='READY' and abs(readings.get('height',1)-baseline)<.025,30)
        result['passed']=True
    except Exception as error:
        result.update(passed=False,error=repr(error))
    finally:
        node.destroy_node();rclpy.shutdown()
        if launch.poll() is None:
            os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGKILL);launch.wait()
        log.close()
    print(json.dumps(result),flush=True)
    return result


if __name__=='__main__':
    results=[run_world(world,i) for i,world in enumerate(('planned/ax_warehouse.world',))]
    (ROOT/'log/ax_revision_runtime/result.json').write_text(json.dumps(results,indent=2)+'\n')
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)
