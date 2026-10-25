#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

def cb(msg):
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    print(f"roll: {math.degrees(roll):6.2f}°  pitch: {math.degrees(pitch):6.2f}°  yaw: {math.degrees(yaw):6.2f}°")

if __name__ == "__main__":
    rospy.init_node("check_gimbal_pitch")
    rospy.Subscriber("/typhoon_h480_0/imu_gazebo", Imu, cb)
    print("Listening /typhoon_h480_0/imu_gazebo ...")
    rospy.spin()
