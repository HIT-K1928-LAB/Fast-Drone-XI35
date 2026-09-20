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
p.add_argument('--port',type=int,default=11319)
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
    log=open(a.package/'test_results'/'ros_test.log','w')
    master=subprocess.Popen(['roscore','-p',str(a.port)],stdout=log,stderr=log,start_new_session=True);processes.append(master)
    def ready():
        try:return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0]==1
        except Exception:return False
    wait_for(ready)
    import rospy
    import yaml
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Empty,String
    from quadrotor_msgs.msg import PositionCommand
    rospy.init_node('minco_isolated_test',anonymous=False,disable_signals=True)
    config=yaml.safe_load((a.package/'config/real.yaml').read_text())
    config.update(execute=False,engine_file=a.engine,camera_extrinsic_config=a.extrinsic,
                  min_height=.1,max_height=100.,height_band=100.,test_radius=100.,max_speed=100.,
                  max_acceleration=100.,max_jerk=1000.,max_tracking_error=100.,
                  safe_radius=.0001,plan_timeout=2.,sensor_timeout=.5,max_sensor_skew=.2)
    rospy.set_param('/yopo_minco_planner',config)
    statuses=[];commands=[]
    rospy.Subscriber('/yopo_minco/status',String,lambda msg:statuses.append(msg.data))
    rospy.Subscriber('/yopo_minco/debug/position_cmd',PositionCommand,lambda msg:commands.append(msg))
    odom_pub=rospy.Publisher('/kf_fusion/kf_imu_odom',Odometry,queue_size=1)
    depth_pub=rospy.Publisher('/camera/depth/image_rect_raw',Image,queue_size=1)
    goal_pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
    trigger_pub=rospy.Publisher('/traj_start_trigger',PoseStamped,queue_size=1)
    stop_pub=rospy.Publisher('/yopo_minco/stop',Empty,queue_size=1)
    import numpy as np
    image_bytes=np.full((480,640),20.,dtype='<f4').tobytes()
    feed_enabled=threading.Event();feed_enabled.set()
    def feed():
        while not stop_event.is_set():
            if feed_enabled.is_set():
                stamp=rospy.Time.now()
                odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='world'
                odom.pose.pose.position.z=1.;odom.pose.pose.orientation.w=1.
                image=Image();image.header.stamp=stamp;image.header.frame_id='camera_depth_optical_frame'
                image.width=640;image.height=480;image.step=640*4;image.encoding='32FC1';image.data=image_bytes
                odom_pub.publish(odom);depth_pub.publish(image)
            time.sleep(1/30)
    thread=threading.Thread(target=feed,daemon=True);thread.start()
    node=subprocess.Popen([a.node],stdout=log,stderr=log,start_new_session=True);processes.append(node)
    wait_for(lambda:any(s.startswith('IDLE') for s in statuses),30)
    wait_for(lambda:goal_pub.get_num_connections()>0 and trigger_pub.get_num_connections()>0)
    time.sleep(.5)
    assert len(commands)==0,'Commands published before goal/trigger'
    goal=PoseStamped();goal.header.frame_id='world';goal.pose.position.x=1.;goal.pose.position.z=1.;goal.pose.orientation.w=1.
    goal_pub.publish(goal);wait_for(lambda:any(s.startswith('GOAL_SET') for s in statuses))
    time.sleep(.4);assert len(commands)==0,'Goal alone started commands'
    trigger_pub.publish(PoseStamped())
    wait_for(lambda:len(commands)>3,10)
    assert all(msg.header.frame_id=='world' and msg.trajectory_flag==msg.TRAJECTORY_STATUS_READY for msg in commands)
    publishers=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/test')[2][0]
    assert not any(topic=='/position_cmd' for topic,nodes in publishers),'Flight command publisher created'
    stop_pub.publish(Empty());wait_for(lambda:any('user request' in s for s in statuses))
    time.sleep(.2);count=len(commands);time.sleep(.5);assert len(commands)==count,'Commands continued after stop'
    trigger_pub.publish(PoseStamped());wait_for(lambda:len(commands)>count+3)
    feed_enabled.clear();wait_for(lambda:any('missing/stale' in s for s in statuses),5)
    time.sleep(.2);count=len(commands);time.sleep(.5);assert len(commands)==count,'Stale input did not stop commands'
    feed_enabled.set();time.sleep(.6);assert len(commands)==count,'Unexpected automatic resume'
    assert node.poll() is None,'Node crashed'
    print('PASS: default idle, explicit trigger, debug-only topic, valid messages, user stop, stale input stop, no automatic resume')
finally:
    stop_event.set()
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
