#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

model = YOLO("/home/lx/下载/best_11s_new.pt", verbose=False)
bridge = CvBridge()
latest = None

def cb(msg):
    global latest
    latest = bridge.imgmsg_to_cv2(msg, "bgr8")

if __name__ == "__main__":
    rospy.init_node("yolo_view")
    drone_id = rospy.get_param("~drone_id", 0)
    rospy.Subscriber(f"/typhoon_h480_{drone_id}/realsense/depth_camera/color/image_raw",
                     Image, cb, queue_size=1)
    print(f"Subscribed /typhoon_h480_{drone_id}/.../color/image_raw, waiting...")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest is not None:
            results = model(latest, conf=0.5, verbose=False)
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls)
                    conf = float(box.conf)
                    cv2.rectangle(latest, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(latest, f"{cls} {conf:.2f}",
                                (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 1)
            cv2.imshow("YOLO", latest)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        rate.sleep()

    cv2.destroyAllWindows()
