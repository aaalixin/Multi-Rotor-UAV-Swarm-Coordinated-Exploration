#!/usr/bin/env python3
import rospy
import math
from gazebo_msgs.srv import ApplyJointEffort
import sys

if __name__ == "__main__":
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node('gimbal_control'+'_'+vehicle_type+'_'+vehicle_id)
    rate = rospy.Rate(50)

    pitch_error_deg = -40.0
    pitch_error_rad = math.radians(pitch_error_deg)

    apply_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    pitch_joint = f"{vehicle_type}_{vehicle_id}::cgo3_camera_joint"

    kp = 0.95
    ki = 0.1245
    cmd_max = 5.0
    cmd_min = -5.0

    pitch_integral = 0.0
    last_time = rospy.Time.now()

    print(f"{vehicle_type}_{vehicle_id}: pitch_err={pitch_error_deg} deg")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        if dt <= 0 or dt > 0.1:
            dt = 0.02
        last_time = now

        pitch_integral += pitch_error_rad * dt
        pitch_effort = kp * pitch_error_rad + ki * pitch_integral
        pitch_effort = max(min(pitch_effort, cmd_max), cmd_min)

        apply_effort(
            joint_name=pitch_joint,
            effort=pitch_effort,
            start_time=rospy.Time(0, 0),
            duration=rospy.Duration(0.02)
        )

        rate.sleep()
