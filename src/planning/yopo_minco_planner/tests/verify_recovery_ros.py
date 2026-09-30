#!/usr/bin/env python3
"""Isolated synthetic ROS test. Never sets execute=true or uses flight master.
Broad trajectory limits apply ONLY to this test, not config/real.yaml.
"""
import argparse
import os
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('--node',required=True)
p.add_argument('--package',required=True,type=Path)
p.add_argument('--engine',required=True)
p.add_argument('--extrinsic',required=True)
p.add_argument('--port',type=int,default=11320)
p.add_argument('--auto',action='store_true')
p.add_argument('--fault',choices=['localization','imu','depth'],default='localization')
a=p.parse_args()
if not 1024<=a.port<=65535 or a.port==11311:raise SystemExit('Use an isolated unprivileged port, not 11311')
with socket.socket() as sock:
    if sock.connect_ex(('127.0.0.1',a.port))==0:raise SystemExit('Test port already in use; refusing existing master')
os.environ['ROS_MASTER_URI']=f'http://127.0.0.1:{a.port}'
os.environ['ROS_IP']='127.0.0.1';os.environ.pop('ROS_HOSTNAME',None)
os.environ['ROS_LOG_DIR']=str(a.package/'test_results'/'ros_logs')
Path(os.environ['ROS_LOG_DIR']).mkdir(parents=True,exist_ok=True)
processes=[];stop_event=threading.Event()
def wait_for(condition,timeout=15):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if condition():return
        time.sleep(.05)
    raise AssertionError('Timed out waiting for test condition')
try:
    log=open(a.package/'test_results'/(('recovery_auto_' if a.auto else 'recovery_manual_')+a.fault+'.log'),'w')
    master=subprocess.Popen(['roscore','-p',str(a.port)],stdout=log,stderr=log,start_new_session=True);processes.append(master)
    def ready():
        try:return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0]==1
        except Exception:return False
    wait_for(ready)
    import rospy
    import yaml
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image,Imu
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Empty,String,Bool
    from std_srvs.srv import Trigger
    from yopo_minco_planner.srv import StartGoal
    from quadrotor_msgs.msg import PositionCommand
    rospy.init_node('minco_isolated_test',anonymous=False,disable_signals=True)
    config=yaml.safe_load((a.package/'config/real.yaml').read_text())
    config.update(execute=False,engine_file=a.engine,camera_extrinsic_config=a.extrinsic,
                  min_height=.1,max_height=100.,height_band=100.,test_radius=100.,max_speed=100.,
                  max_acceleration=100.,max_jerk=1000.,max_tracking_error=100.,
                  safe_radius=.20,plan_timeout=.5,sensor_timeout=.5,max_sensor_skew=.2,
                  recovery_auto_resume=a.auto)
    rospy.set_param('/yopo_minco_planner',config)
    statuses=[];commands=[]
    rospy.Subscriber('/yopo_minco/status',String,lambda msg:statuses.append(msg.data))
    rospy.Subscriber('/yopo_minco/debug/position_cmd',PositionCommand,lambda msg:commands.append(msg))
    odom_pub=rospy.Publisher('/kf_fusion/kf_imu_odom',Odometry,queue_size=1)
    depth_pub=rospy.Publisher('/camera/depth/image_rect_raw',Image,queue_size=1)
    imu_pub=rospy.Publisher("/imu_filter/data",Imu,queue_size=1)
    goal_pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
    trigger_pub=rospy.Publisher('/traj_start_trigger',PoseStamped,queue_size=1)
    stop_pub=rospy.Publisher('/yopo_minco/stop',Empty,queue_size=1)
    fault_pub=rospy.Publisher("/kf_fusion/kf_fail",Bool,queue_size=1)
    import numpy as np
    image_bytes=np.full((480,640),20.,dtype='<f4').tobytes()
    clear_bytes=image_bytes
    blocked_bytes=np.full((480,640),.05,dtype="<f4").tobytes()
    feed_enabled=threading.Event();feed_enabled.set()
    rewind=threading.Event()
    def feed():
        while not stop_event.is_set():
            if feed_enabled.is_set():
                stamp=rospy.Time.now()
                odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='world'
                odom.pose.pose.position.z=1.;odom.pose.pose.orientation.w=1.
                image=Image();image.header.stamp=stamp;image.header.frame_id='camera_depth_optical_frame'
                image.width=640;image.height=480;image.step=640*4;image.encoding='32FC1';image.data=image_bytes
                imu=Imu();imu.header.stamp=stamp
                if rewind.is_set():
                    if a.fault=="imu":imu.header.stamp=stamp-rospy.Duration(.15)
                    if a.fault=="depth":image.header.stamp=stamp-rospy.Duration(.15)
                    rewind.clear()
                imu_pub.publish(imu)
                odom_pub.publish(odom);depth_pub.publish(image)
            time.sleep(1/30)
    thread=threading.Thread(target=feed,daemon=True);thread.start()
    node=subprocess.Popen([a.node],stdout=log,stderr=log,start_new_session=True);processes.append(node)
    wait_for(lambda:any(s.startswith('IDLE') for s in statuses),30)
    wait_for(lambda:goal_pub.get_num_connections()>0 and trigger_pub.get_num_connections()>0)
    time.sleep(.5)
    assert len(commands)==0,'Commands published before goal/trigger'
    start=rospy.ServiceProxy('/yopo_minco/start_goal',StartGoal)
    resume=rospy.ServiceProxy('/yopo_minco/resume',Trigger)
    rospy.wait_for_service('/yopo_minco/start_goal',timeout=3)
    def goal(x=1.,frame='world',age=0.):
        g=PoseStamped();g.header.frame_id=frame;g.header.stamp=rospy.Time.now()-rospy.Duration(age)
        g.pose.position.x=x;g.pose.position.z=1.;g.pose.orientation.w=1.
        return g
    for g in (goal(.01),goal(3.),goal(float('nan')),goal(frame='map'),goal(age=2)):
        r=start(g);assert not r.success,r.message
    assert not resume().success,'Resume accepted outside READY'
    assert not commands,'Rejected goals emitted commands'
    r=start(goal());assert r.success,r.message
    wait_for(lambda:len(commands)>3,10)
    assert not start(goal()).success,'Busy task replaced'
    image_bytes=blocked_bytes
    wait_for(lambda:any(s.startswith('WAIT_HOVER') for s in statuses),10)
    time.sleep(.2);count=len(commands)
    wait_for(lambda:any(s.startswith('HOVER_REPLAN') for s in statuses),10)
    time.sleep(.8)
    assert len(commands)==count,'Commands during rejected recovery preview'
    assert not resume().success,'Blocked corridor accepted resume'
    image_bytes=clear_bytes
    wait_for(lambda:any(s.startswith('READY') for s in statuses),10)
    if not a.auto:
        time.sleep(.5);assert len(commands)==count,'Default mode resumed without confirmation'
        # Any fresh failed plan revokes READY and its eligibility for confirmation.
        image_bytes=blocked_bytes
        wait_for(lambda:any('confirmation revoked' in s for s in statuses),5)
        assert not resume().success,'Invalidated READY accepted resume'
        image_bytes=clear_bytes
        ready_count=sum(s.startswith('READY') for s in statuses)
        wait_for(lambda:sum(s.startswith('READY') for s in statuses)>ready_count,5)
        r=resume();assert r.success,r.message
    wait_for(lambda:any(s.startswith('RESUME') for s in statuses),5)
    wait_for(lambda:len(commands)>count+3,5)
    # STOP is terminal and neither legacy trigger nor resume can resurrect it.
    stop_pub.publish(Empty());wait_for(lambda:any('user request' in s for s in statuses))
    time.sleep(.2);count=len(commands);trigger_pub.publish(PoseStamped());time.sleep(.5)
    assert len(commands)==count,'Old goal resurrected after STOP'
    assert not resume().success
    r=start(goal());assert r.success,r.message
    wait_for(lambda:len(commands)>count+3,5)
    expected={"localization":"localization failure latched", "imu":"IMU timestamp moved backwards",
              "depth":"depth timestamp moved backwards"}[a.fault]
    if a.fault=="localization":fault_pub.publish(Bool(data=True))
    else:rewind.set()
    wait_for(lambda:any(expected in s for s in statuses),5)
    time.sleep(.2);count=len(commands);fault_pub.publish(Bool(data=False));time.sleep(.5)
    r=start(goal());assert not r.success and expected in r.message,r.message
    assert len(commands)==count,'Latched failure resumed'
    publishers=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/test')[2][0]
    assert not any(topic=='/position_cmd' for topic,nodes in publishers),'Flight command publisher created'
    assert node.poll() is None,'Node crashed'
    print('PASS: atomic goal guards, corridor failure, stable hover, consecutive plans, '+
          ('automatic' if a.auto else 'manual')+' recovery, STOP latch, '+a.fault+' fault latch, debug-only')

finally:
    stop_event.set()
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
