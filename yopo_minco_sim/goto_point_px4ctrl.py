"""Direct point-to-point Gazebo test. No obstacle avoidance or automatic arming."""
import argparse
import math
import time
import numpy as np
import rospy
import rosgraph
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from quadrotor_msgs.msg import PositionCommand


def sample(start, goal, t, duration):
    u = np.clip(t / duration, 0., 1.)
    delta = goal - start
    s = 10*u**3 - 15*u**4 + 6*u**5
    ds = (30*u**2 - 60*u**3 + 30*u**4) / duration
    dds = (60*u - 180*u**2 + 120*u**3) / duration**2
    ddds = (60 - 360*u + 360*u**2) / duration**3 if 0 <= t < duration else 0.
    return start + delta*s, delta*ds, delta*dds, delta*ddds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    args = parser.parse_args(rospy.myargv()[1:])
    goal = np.array([args.x, args.y, args.z])
    if not np.isfinite(goal).all() or goal[2] < 0.6:
        parser.error('Use a finite airborne target with world z >= 0.6 m; use land.sh for landing.')
    rospy.init_node('goto_point_px4ctrl', anonymous=True)
    data = {}
    def receive(key):
        def callback(msg):
            data[key] = (msg, time.monotonic())
        return callback
    handles = [rospy.Subscriber('/gazebo/iris_0/odometry', Odometry, receive('odom'), queue_size=1),
               rospy.Subscriber('/iris_0/mavros/state', State, receive('state'), queue_size=1)]
    deadline = time.monotonic() + 10
    while len(data) < 2 and not rospy.is_shutdown():
        if time.monotonic() > deadline:
            raise RuntimeError('No odometry or MAVROS state')
        time.sleep(.05)

    def current():
        odom, ot = data['odom']; state, st = data['state']
        age = (rospy.Time.now() - odom.header.stamp).to_sec()
        if time.monotonic()-ot > .5 or time.monotonic()-st > 3 or not -.05 <= age <= .5:
            raise RuntimeError('Stale odometry/state; stopping commands')
        if not state.connected or not state.armed or state.mode != 'OFFBOARD':
            raise RuntimeError('First take off and establish OFFBOARD hover with PX4Ctrl')
        if odom.header.frame_id != 'world':
            raise RuntimeError('Expected world-frame odometry')
        p, v = odom.pose.pose.position, odom.twist.twist.linear
        pos, vel = np.array([p.x,p.y,p.z]), np.array([v.x,v.y,v.z])
        if not np.isfinite(pos).all() or not np.isfinite(vel).all():
            raise RuntimeError('Nonfinite odometry')
        return odom, pos, vel

    odom, start, vel = current()
    if np.linalg.norm(vel) > .15:
        raise RuntimeError('Wait for stable hover, speed <= 0.15 m/s')
    distance = float(np.linalg.norm(goal-start))
    if distance > 10:
        raise RuntimeError('This initial direct-flight test is limited to 10 m')
    pubs = rosgraph.Master(rospy.get_name()).getSystemState()[0]
    if any(topic == '/position_cmd' and nodes for topic, nodes in pubs):
        raise RuntimeError('Stop the YOPO command node and any other /position_cmd publisher first')
    # Quintic trajectory: peak speed <= 0.5 m/s, acceleration <= 0.4 m/s^2.
    duration = max(3., 1.875*distance/.5, math.sqrt(5.774*distance/.4))
    q = odom.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=1)
    deadline = time.monotonic()+5
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if time.monotonic() > deadline:
            raise RuntimeError('No PX4Ctrl subscriber')
        time.sleep(.05)
    current()
    t0 = rospy.Time.now()
    rospy.logwarn('DIRECT FLIGHT, NO AVOIDANCE: %s -> %s; duration %.1f s', start, goal, duration)
    rate = rospy.Rate(50)
    settled = None
    while not rospy.is_shutdown():
        odom, actual, velocity = current()
        t = (rospy.Time.now()-t0).to_sec()
        if t < 0:
            raise RuntimeError('Simulation time reset')
        p,v,a,j = sample(start, goal, t, duration)
        if np.linalg.norm(actual-p) > 1:
            raise RuntimeError('Tracking error exceeds 1 m; stopping commands')
        error = float(np.linalg.norm(actual-goal))
        rospy.loginfo_throttle(1., 'Actual (%.2f, %.2f, %.2f); target error %.3f m', *actual, error)
        if t >= duration and error < .15 and np.linalg.norm(velocity) < .15:
            if settled is None:
                settled = t
            if t-settled >= 1:
                rospy.logwarn('ARRIVED: error %.3f m. Publishing stopped; wait for PX4Ctrl AUTO_HOVER.', error)
                return
        else:
            settled = None
        if t > duration+20:
            raise RuntimeError('Target settling timeout; stopping commands')
        msg = PositionCommand();msg.header.stamp=rospy.Time.now();msg.header.frame_id='world'
        msg.trajectory_flag=msg.TRAJECTORY_STATUS_READY
        for field,vec in [(msg.position,p),(msg.velocity,v),(msg.acceleration,a),(msg.jerk,j)]:
            field.x,field.y,field.z = map(float,vec)
        msg.yaw=yaw;msg.yaw_dot=0.
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr('%s', exc)
        raise SystemExit(1)
