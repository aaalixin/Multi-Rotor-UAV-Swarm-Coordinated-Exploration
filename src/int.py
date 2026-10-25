#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from ros_msg.msg import LockReport, LockMap
import threading

class LockManager:
    def __init__(self):
        rospy.init_node("lock_manager", anonymous=False)  # 固定名字: 重复启动直接报错, 保证只有一个中央

        self.lock_table = {i: None for i in range(6)}
        self.mutex = threading.Lock()

        # 只收锁表、广播锁表
        for uav_id in range(6):
            rospy.Subscriber("/typhoon_h480_{}/lock_report".format(uav_id),
                             LockReport, self.lock_cb, queue_size=10)

        self.map_pub = rospy.Publisher("/lock_map", LockMap, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.publish_map)  # 50 Hz

    def lock_cb(self, msg):
        with self.mutex:
            cls, uav_id, lock = msg.cls, msg.uav_id, msg.lock

            if lock:                       # 想锁定
                # 若已被其他 UAV 占用，直接拒绝
                if self.lock_table[cls] is not None and self.lock_table[cls] != uav_id:
                    return
                self.lock_table[cls] = uav_id
            else:                          # 主动释放
                if self.lock_table[cls] == uav_id:
                    self.lock_table[cls] = None

    def publish_map(self, event):
        with self.mutex:
            msg = LockMap()
            msg.locked_by = [np.int8(self.lock_table[i] if self.lock_table[i] is not None else -1)
                             for i in range(6)]
            self.map_pub.publish(msg)


if __name__ == "__main__":
    try:
        LockManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass