#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.drone_id = rospy.get_param("~drone_id", 0)
        self.save_dir = rospy.get_param("~save_dir", "/home/lx/robocup_gazebo/images")
        os.makedirs(self.save_dir, exist_ok=True)
        self.count = 40
        self.frame_counter = 0  # ✅ 新增：帧计数器
        rospy.Subscriber(f"/typhoon_h480_{self.drone_id}/realsense/depth_camera/color/image_raw",
                         Image, self.cb, queue_size=1)
        print(f"Saving to {self.save_dir}, drone {self.drone_id}")

    def cb(self, msg):
        self.frame_counter += 1  # ✅ 每帧+1
        
        # ✅ 只有帧计数器能被5整除时才保存
        if self.frame_counter % 5 == 0:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            path = os.path.join(self.save_dir, f"drone{self.drone_id}_{self.count:06d}.jpg")
            cv2.imwrite(path, img)
            self.count += 1
            print(f"Saved: {path}")  # 可选：打印保存信息

if __name__ == "__main__":
    rospy.init_node("save_images")
    ImageSaver()
    rospy.spin()
