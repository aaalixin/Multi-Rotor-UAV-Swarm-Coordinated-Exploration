#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

est = {}       # drone_id -> Point (target_world)
drone_pos = {} # drone_id -> (x,y,z) local
drone_yaw = {} # drone_id -> body yaw (from mavros pose)
cam_yaw = {}   # drone_id -> camera yaw (from imu_gazebo)
states = None

def cb_t(msg, i):
    est[i] = msg

def cb_p(msg, i):
    drone_pos[i] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

def cb_s(msg):
    global states
    states = msg

def cb_pose_yaw(msg, i):
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    _, _, y = euler_from_quaternion(q)
    drone_yaw[i] = y

def cb_imu_yaw(msg, i):
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    _, _, y = euler_from_quaternion(q)
    cam_yaw[i] = y

rospy.init_node("check_target")
for i in range(6):
    rospy.Subscriber(f"/drone_{i}/target_world", Point, cb_t, i)
    rospy.Subscriber(f"/typhoon_h480_{i}/mavros/local_position/pose", PoseStamped, cb_p, i)
    rospy.Subscriber(f"/typhoon_h480_{i}/mavros/local_position/pose", PoseStamped, cb_pose_yaw, i)
    rospy.Subscriber(f"/typhoon_h480_{i}/imu_gazebo", Imu, cb_imu_yaw, i)
rospy.Subscriber("/gazebo/model_states", ModelStates, cb_s)

r = rospy.Rate(2)
while not rospy.is_shutdown():
    if states is None or not est:
        r.sleep(); continue
    actors = {n: p.position for n, p in zip(states.name, states.pose) if 'actor' in n}
    for i in list(est):
        e = est[i]
        if not actors:
            continue
        best = min(((n, math.hypot(p.x-e.x, p.y-e.y)) for n, p in actors.items()), key=lambda t: t[1])
        n = best[0]
        p = actors[n]
        lp = drone_pos.get(i)
        dy = cam_yaw.get(i, 0) - drone_yaw.get(i, 0)
        offs = {0:(-17,-3),1:(-14,-3),2:(-17,0),3:(-14,0),4:(-17,3),5:(-14,3)}
        ox, oy = offs.get(i, (0,0))
        td = math.hypot(lp[0]+ox - p.x, lp[1]+oy - p.y) if lp else -1
        ed = math.hypot(lp[0]+ox - e.x, lp[1]+oy - e.y) if lp else -1
        print(f"drone{i}: est=({e.x:6.1f},{e.y:6.1f}) {n}=({p.x:6.1f},{p.y:6.1f}) err=({e.x-p.x:+5.1f},{e.y-p.y:+5.1f}) |err|={best[1]:.1f}m true_dist={td:4.1f} est_dist={ed:4.1f} dyaw={math.degrees(dy):+5.1f}" if lp else f"drone{i}: err={best[1]:.1f}m", flush=True)
    r.sleep()
