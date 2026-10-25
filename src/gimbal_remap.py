#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

class GimbalRemap:
    def __init__(self):
        self.drone_id = rospy.get_param("~drone_id", 0)
        self.pub = rospy.Publisher(f"/typhoon_h480_{self.drone_id}/gimbal_imu", Imu, queue_size=10)
        self.idx = 0
        rospy.Subscriber("/gimbal_imu", Imu, self.cb)
        rospy.loginfo(f"GimbalRemap drone{self.drone_id} running")

    def cb(self, msg):
        # Round-robin: each drone takes every Nth message
        self.idx += 1
        if self.idx % 6 == (self.drone_id + 1) % 6:  # crude round-robin
            msg.header.frame_id = f"typhoon_h480_{self.drone_id}/gimbal_imu"
            self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("gimbal_remap")
    GimbalRemap()
    rospy.spin()
