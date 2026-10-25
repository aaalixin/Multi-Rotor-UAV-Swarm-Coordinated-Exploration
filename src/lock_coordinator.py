#!/usr/bin/env python3
import rospy
from ros_msg.msg import LockReport, LockMap

class LockCoordinator:
    def __init__(self):
        self.locked_by = [-1] * 6  # class → uav_id
        self.pub = rospy.Publisher("/lock_map", LockMap, queue_size=1)
        for i in range(6):
            rospy.Subscriber(f"/typhoon_h480_{i}/lock_report", LockReport, self.cb, i)
        rospy.loginfo("LockCoordinator running")

    def cb(self, msg, drone_id):
        cls = msg.cls
        if 0 <= cls < 6:
            if msg.lock:
                self.locked_by[cls] = drone_id
            elif self.locked_by[cls] == drone_id:
                self.locked_by[cls] = -1
        self.publish()

    def publish(self):
        m = LockMap()
        m.locked_by = list(self.locked_by)
        self.pub.publish(m)

if __name__ == "__main__":
    rospy.init_node("lock_coordinator")
    LockCoordinator()
    rospy.spin()
