#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
import threading
import sys
from std_msgs.msg import Bool, String, Float64
from sensor_msgs.msg import Image, Imu, LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point, TwistStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
from tf.transformations import euler_from_quaternion
from ros_msg.msg import LockReport, LockMap
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

model = YOLO("/home/lx/robocup_gazebo/src/best_11s_new.pt", verbose=False)

# 相机内参 (640x480)
CX, CY = 320.5, 240.5
FX, FY = 554.254691191187, 554.254691191187
IMG_W, IMG_H = 640, 480

# 高度配置
TARGET_HEIGHT_TRACK = 2.3   # 有深度时追踪高度
TARGET_HEIGHT_SEARCH = 2.3  # 无深度时搜索高度
TARGET_HEIGHT_RETURN = 2.2  # 返回时固定高度

TRACK_DIST = 5.0  # 期望追踪距离

# 红色双锁槽 (两个红都是YOLO cls=4): 4/5两个槽, 先发布坐标者先占, 优先4
RED_SLOTS = (4, 5)
RED_DUP_DIST = 2.4       # 实际坐标判定"已被追踪"的距离阈值 (m), 两红可能靠近故给宽
RED_POS_FRESH = 2.5      # 对方已认领红坐标的有效期 (s), 发布间隔0.5s
RED_SLOT_TIMEOUT = 0.5   # 抢槽被拒后等待判定时长 (s)
RED_DUP_HOLD = 1.0       # 自愈: 与另一槽坐标重合持续多久后让位 (s)
RED_YAW_DUP_DEG = 4.0    # 无深度兜底: 目标框像素yaw与已发布红位置方位yaw夹角≤4° => 同一个红,已被追踪,不追

# 相机安装位置 (SDF gimbal链求和: mount 0,0,0 + 竖臂 -0.026,0,-0.10 + 横臂 -0.07,0,-0.162 + 相机关节 -0.01,0.03,-0.162)
CAM_OFFSET = np.array([-0.106, 0.03, -0.424])

# 云台俯仰角（世界系下固定，相机下俯角度）
GIMBAL_PITCH = 15.0 * math.pi / 180.0

# 6机偏移 (spawn: x=-17/-14)
OFFSETS = {0: (-17, -3), 1: (-14, -3), 2: (-17, 0),
           3: (-14, 0), 4: (-17, 3), 5: (-14, 3)}

# ==================== 路线定义 (与C++保持一致) ====================
def densify(wps, step=3.0):
    """路径点密化"""
    out = []
    for i in range(len(wps)):
        a = wps[i]
        b = wps[(i + 1) % len(wps)]
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(d / step)))
        for k in range(n):
            t = k / n
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out

def ang_diff(a, b):
    """两角最小夹角 (rad)"""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

# 路线配置 [route_id][drone_id] = (waypoints_raw, offset_x, offset_y, speed)
ROUTES = {
    0: {  # route 0 (contX.py)
        0: (densify([(-14,-3),(-14,-22),(-14,-40),(2,-40),(21,-40),(42.5,-40),(42.5,-22),(42.5,-3),(60,-3),(75,-3),(90,-3),(105,-3),(105,-22),(105,-40),(90,-40),(75,-40),(60,-40),(42.5,-40),(42.5,-22),(42.5,-3),(21,-3),(2,-3)]), -17, -3, 15),
        1: (densify([(-11,-4.5),(0,-4.5),(15,-4.5),(30,-4.5),(40,-4.5),(40,-21.5),(40,-38),(55,-38),(70,-38),(85,-38),(100,-38),(114.5,-38),(114.5,-21.5),(114.5,-4.5),(114.5,15),(114.5,30),(114.5,45),(100,45),(85,45),(70,45),(55,45),(40,45),(25,45),(5,45),(-11,45),(-11,30),(-11,15)]), -14, -3, 15),
        2: (densify([(-16,-1.5),(-30,-1.5),(-45,-1.5),(-45,-21.5),(-45,-40),(-25,-40),(-10.5,-40),(-10.5,-20),(-10.5,-1.5),(5,-1.5),(20,-1.5),(35,-1.5),(48,-1.5),(48,-20),(48,-42.5),(35,-42.5),(20,-42.5),(5,-42.5),(-10.5,-42.5),(-10.5,-20),(-10.5,-5)]), -17, 0, 15),
        3: (densify([(-11,4.5),(0,4.5),(15,4.5),(30,4.5),(40,4.5),(40,21.5),(40,38),(55,38),(70,38),(85,38),(100,38),(112.5,38),(112.5,21.5),(112.5,4.5),(112.5,-15),(112.5,-30),(112.5,-45),(100,-45),(85,-45),(70,-45),(55,-45),(40,-45),(25,-45),(5,-45),(-11,-45),(-11,-30),(-11,-15)]), -14, 0, 15),
        4: (densify([(-16,1.5),(-30,1.5),(-45,1.5),(-45,21.5),(-45,40),(-25,40),(-10.5,40),(-10.5,15),(-10.5,1.5),(5,1.5),(20,1.5),(35,1.5),(48,1.5),(48,15),(48,42.5),(35,42.5),(20,42.5),(5,42.5),(-10.5,42.5),(-10.5,15),(-10.5,5)]), -17, 3, 15),
        5: (densify([(-14,3),(-14,22),(-14,40),(2,40),(21,40),(42.5,40),(42.5,22),(42.5,3),(60,3),(75,3),(90,3),(105,3),(105,22),(105,40),(90,40),(75,40),(60,40),(42.5,40),(42.5,22),(42.5,3),(21,3),(2,3)]), -14, 3, 15),
    },
    1: {  # route 1 (contX_1.py)
        0: (densify([(-14,-12.5),(-14,-24.5),(-14,-40),(2,-40),(21,-40),(42.5,-40),(42.5,-24.5),(42.5,-12.5),(60,-12.5),(75,-12.5),(90,-12.5),(105,-12.5),(105,-22),(105,-40),(90,-40),(75,-40),(60,-40),(42.5,-40),(42.5,-22),(42.5,-12.5),(21,-12.5),(2,-12.5)]), -17, -3, 15),
        1: (densify([(-11,0),(-11,-14),(-14,-14),(0,-14),(15,-14),(30,-14),(40,-14),(40,-21.5),(40,-38),(55,-38),(70,-38),(85,-38),(100,-38),(114.5,-38),(114.5,-21.5),(114.5,-14),(114.5,0),(114.5,15),(114.5,30),(114.5,45),(100,45),(85,45),(70,45),(55,45),(40,45),(25,45),(5,45),(-11,45),(-11,30),(-11,15)]), -14, -3, 15),
        2: (densify([(-17,-11),(-30,-11),(-45,-11),(-45,-21.5),(-45,-40),(-25,-40),(-10.5,-40),(-10.5,-20),(-10.5,-11),(5,-11),(20,-11),(35,-11),(48,-11),(48,-20),(48,-42.5),(35,-42.5),(20,-42.5),(5,-42.5),(-10.5,-42.5),(-10.5,-20),(-10.5,-14.5),(-16,-11)]), -17, 0, 15),
        3: (densify([(-14,-5),(0,-5),(15,-5),(30,-5),(40,-5),(40,21.5),(40,38),(55,38),(70,38),(85,38),(100,38),(112.5,38),(112.5,21.5),(112.5,4.5),(112.5,-15),(112.5,-30),(112.5,-45),(100,-45),(85,-45),(70,-45),(55,-45),(40,-45),(25,-45),(5,-45),(-11,-45),(-11,-30),(-11,-15),(-11,-5)]), -14, 0, 15),
        4: (densify([(-10.5,-4.5),(-16,-8),(-17,-8),(-30,-8),(-45,-8),(-45,10),(-45,25),(-45,40),(-25,40),(-10.5,40),(-10.5,15),(-10.5,-8),(5,-8),(20,-8),(35,-8),(48,-8),(48,15),(48,42.5),(35,42.5),(20,42.5),(5,42.5),(-10.5,42.5),(-10.5,15)]), -17, 3, 15),
        5: (densify([(-14,10),(-14,26),(-14,40),(2,40),(21,40),(42.5,40),(42.5,26),(42.5,10),(42.5,-6.5),(60,-6.5),(75,-6.5),(90,-6.5),(105,-6.5),(105,10),(105,26),(105,40),(90,40),(75,40),(60,40),(42.5,40),(42.5,26),(42.5,10),(42.5,-6.5),(21,-6.5),(2,-6.5),(-14,-6.5)]), -14, 3, 15),
    },
    2: {  # route 2 (contX_2.py)
        0: (densify([(-14,-10),(-14,-25),(-14,-40),(2,-40),(21,-40),(42.5,-40),(42.5,-25),(42.5,-10),(42.5,7.8),(60,7.8),(75,7.8),(90,7.8),(105,7.8),(105,-10),(105,-25),(105,-40),(90,-40),(75,-40),(60,-40),(42.5,-40),(42.5,-25),(42.5,-10),(42.5,7.8),(21,7.8),(2,7.8),(-14,7.8)]), -17, -3, 15),
        1: (densify([(-14,6.3),(0,6.3),(15,6.3),(30,6.3),(40,6.3),(40,-11.7),(40,-21.5),(40,-38),(55,-38),(70,-38),(85,-38),(100,-38),(114.5,-38),(114.5,-21.5),(114.5,-11.7),(114.5,6.3),(114.5,15),(114.5,30),(114.5,45),(100,45),(85,45),(70,45),(55,45),(40,45),(25,45),(5,45),(-11,45),(-11,30),(-11,15),(-11,6.3)]), -14, -3, 15),
        2: (densify([(-10.5,-5),(-10.5,9.3),(-16,9.3),(-17,9.3),(-30,9.3),(-45,9.3),(-45,-10),(-45,-21.5),(-45,-40),(-25,-40),(-10.5,-40),(-10.5,-20),(-10.5,-5),(-10.5,9.3),(5,9.3),(20,9.3),(35,9.3),(48,9.3),(48,-5),(48,-20),(48,-42.5),(35,-42.5),(20,-42.5),(5,-42.5),(-10.5,-42.5),(-10.5,-20)]), -17, 0, 15),
        3: (densify([(-11,0),(-11,15.3),(-14,15.3),(0,15.3),(15,15.3),(30,15.3),(40,15.3),(40,25),(40,38),(55,38),(70,38),(85,38),(100,38),(112.5,38),(112.5,25),(112.5,15.3),(112.5,0),(112.5,-15),(112.5,-30),(112.5,-45),(100,-45),(85,-45),(70,-45),(55,-45),(40,-45),(25,-45),(5,-45),(-11,-45),(-11,-30),(-11,-15)]), -14, 0, 15),
        4: (densify([(-17,12.3),(-30,12.3),(-45,12.3),(-45,25),(-45,40),(-25,40),(-10.5,40),(-10.5,25),(-10.5,12.3),(5,12.3),(20,12.3),(35,12.3),(48,12.3),(48,25),(48,42.5),(35,42.5),(20,42.5),(5,42.5),(-10.5,42.5),(-10.5,25),(-10.5,15.8),(-16,12.3)]), -17, 3, 15),
        5: (densify([(-14,13.8),(-14,26),(-14,40),(2,40),(21,40),(42.5,40),(42.5,26),(42.5,13.8),(60,13.8),(75,13.8),(90,13.8),(105,13.8),(105,26),(105,40),(90,40),(75,40),(60,40),(42.5,40),(42.5,26),(42.5,13.8),(21,13.8),(2,13.8)]), -14, 3, 15),
    },
    3: {  # route 3 (contX_3.py)
        0: (densify([(-14,-12.5),(-14,-24.5),(-14,-40),(2,-40),(21,-40),(42.5,-40),(60,-40),(60,-24.5),(60,-12.5),(75,-12.5),(90,-12.5),(105,-12.5),(105,-22),(105,-40),(90,-40),(75,-40),(60,-40),(60,-22),(60,-12.5),(40,-12.5),(21,-12.5),(2,-12.5)]), -17, -3, 15),
        1: (densify([(-11,0),(-11,-14),(-14,-14),(0,-14),(15,-14),(30,-14),(57,-14),(57,-21.5),(57,-38),(70,-38),(85,-38),(100,-38),(114.5,-38),(114.5,-21.5),(114.5,-14),(114.5,0),(114.5,15),(114.5,30),(114.5,45),(100,45),(85,45),(70,45),(55,45),(40,45),(25,45),(5,45),(-11,45),(-11,30),(-11,15)]), -14, -3, 15),
        2: (densify([(-17,-11),(-30,-11),(-40,-11),(-40,-21.5),(-45,-21.5),(-45,-40),(-25,-40),(-10.5,-40),(-10.5,-20),(-10.5,-11),(5,-11),(20,-11),(35,-11),(48,-11),(62,-11),(62,-20),(62,-42.5),(48,-42.5),(35,-42.5),(20,-42.5),(5,-42.5),(-10.5,-42.5),(-10.5,-20),(-10.5,-14.5),(-16,-11)]), -17, 0, 15),
        3: (densify([(-14,-5),(0,-5),(15,-5),(32,-5),(57,-5),(57,21.5),(57,38),(70,38),(85,38),(100,38),(112.5,38),(112.5,21.5),(112.5,4.5),(112.5,-15),(112.5,-30),(112.5,-45),(100,-45),(85,-45),(70,-45),(55,-45),(40,-45),(25,-45),(5,-45),(-11,-45),(-11,-30),(-11,-15),(-11,-5)]), -14, 0, 15),
        4: (densify([(-10.5,-4.5),(-16,-8),(-17,-8),(-30,-8),(-45,-8),(-45,10),(-45,25),(-45,40),(-25,40),(-10.5,40),(-10.5,15),(-10.5,-8),(5,-8),(20,-8),(35,-8),(48,-8),(62,-8),(62,15),(62,42.5),(40,42.5),(35,42.5),(20,42.5),(5,42.5),(-10.5,42.5),(-10.5,15)]), -17, 3, 15),
        5: (densify([(-14,10),(-14,26),(-14,40),(2,40),(21,40),(42.5,40),(60,40),(60,26),(60,10),(60,-6.5),(75,-6.5),(90,-6.5),(105,-6.5),(105,10),(105,26),(105,40),(90,40),(75,40),(60,40),(60,26),(60,10),(60,-6.5),(42,-6.5),(21,-6.5),(2,-6.5),(-14,-6.5)]), -14, 3, 15),
    },
    4: {  # route 4 (contX_4.py)
        0: (densify([(-14,-10),(-14,-25),(-14,-40),(2,-40),(21,-40),(42.5,-40),(60,-40),(60,-25),(60,-10),(60,7.8),(75,7.8),(90,7.8),(105,7.8),(105,-10),(105,-25),(105,-40),(90,-40),(75,-40),(60,-40),(60,-25),(60,-10),(60,7.8),(21,7.8),(2,7.8),(-14,7.8)]), -17, -3, 15),
        1: (densify([(-14,6.3),(0,6.3),(15,6.3),(30,6.3),(57,6.3),(57,-11.7),(57,-21.5),(57,-38),(70,-38),(85,-38),(100,-38),(114.5,-38),(114.5,-21.5),(114.5,-11.7),(114.5,6.3),(114.5,15),(114.5,30),(114.5,45),(100,45),(85,45),(70,45),(55,45),(40,45),(25,45),(5,45),(-11,45),(-11,30),(-11,15),(-11,6.3)]), -14, -3, 15),
        2: (densify([(-10.5,-5),(-10.5,9.3),(-16,9.3),(-17,9.3),(-30,9.3),(-45,9.3),(-45,-10),(-45,-21.5),(-45,-40),(-25,-40),(-10.5,-40),(-10.5,-20),(-10.5,-5),(-10.5,9.3),(5,9.3),(20,9.3),(35,9.3),(48,9.3),(62,9.3),(62,-5),(62,-20),(62,-42.5),(48,-42.5),(35,-42.5),(20,-42.5),(5,-42.5),(-10.5,-42.5),(-10.5,-20)]), -17, 0, 15),
        3: (densify([(-11,0),(-11,15.3),(-14,15.3),(0,15.3),(15,15.3),(30,15.3),(57,15.3),(57,25),(57,38),(70,38),(85,38),(100,38),(112.5,38),(112.5,25),(112.5,15.3),(112.5,0),(112.5,-15),(112.5,-30),(112.5,-45),(100,-45),(85,-45),(70,-45),(55,-45),(40,-45),(25,-45),(5,-45),(-11,-45),(-11,-30),(-11,-15)]), -14, 0, 15),
        4: (densify([(-17,12.3),(-30,12.3),(-45,12.3),(-45,25),(-45,40),(-25,40),(-10.5,40),(-10.5,25),(-10.5,12.3),(5,12.3),(20,12.3),(35,12.3),(48,12.3),(62,12.3),(62,25),(62,42.5),(48,42.5),(35,42.5),(20,42.5),(5,42.5),(-10.5,42.5),(-10.5,25),(-10.5,15.8),(-16,12.3)]), -17, 3, 15),
        5: (densify([(-14,13.8),(-14,26),(-14,40),(2,40),(21,40),(42.5,40),(60,40),(60,26),(60,13.8),(75,13.8),(90,13.8),(105,13.8),(105,26),(105,40),(90,40),(75,40),(60,40),(60,26),(60,13.8),(40,13.8),(21,13.8),(2,13.8)]), -14, 3, 15),
    },
    5: {  # route 5 (contX_6.py)
        0: (densify([(-14,-3),(-14,-22),(-14,-40),(2,-40),(15,-40),(28,-40),(28,-22),(28,-3),(40,-3),(60,-3),(75,-3),(90,-3),(105,-3),(105,-22),(105,-40),(90,-40),(75,-40),(60,-40),(42.5,-40),(28,-40),(28,-22),(28,-3),(2,-3)]), -17, -3, 15),
        1: (densify([(-11,-4.5),(0,-4.5),(10,-4.5),(25,-4.5),(25,-21.5),(25,-38),(40,-38),(55,-38),(70,-38),(85,-38),(100,-38),(114.5,-38),(114.5,-21.5),(114.5,-4.5),(114.5,15),(114.5,30),(114.5,45),(100,45),(85,45),(70,45),(55,45),(40,45),(25,45),(5,45),(-11,45),(-11,30),(-11,15)]), -14, -3, 15),
        2: (densify([(-16,-1.5),(-16,-20),(-10.5,-20),(-10.5,-5),(-30,-1.5),(-45,-1.5),(-45,-21.5),(-45,-40),(-25,-40),(-10.5,-40),(-10.5,-20),(-10.5,-1.5),(5,-1.5),(20,-1.5),(35,-1.5),(35,-20),(35,-42.5),(20,-42.5),(5,-42.5),(-10.5,-42.5),(-10.5,-1.5)]), -17, 0, 15),
        3: (densify([(-14,4.5),(0,4.5),(15,4.5),(30,4.5),(37,4.5),(37,21.5),(37,38),(55,38),(70,38),(85,38),(100,38),(112.5,38),(112.5,21.5),(112.5,4.5),(112.5,-15),(112.5,-30),(112.5,-45),(100,-45),(85,-45),(70,-45),(55,-45),(40,-45),(25,-45),(5,-45),(-11,-45),(-11,-30),(-11,-15),(-11,4.5)]), -14, 0, 15),
        4: (densify([(-16,1.5),(-16,15),(-10.5,15),(-10.5,5),(-30,1.5),(-45,1.5),(-45,21.5),(-45,40),(-25,40),(-10.5,40),(-10.5,15),(-10.5,1.5),(5,1.5),(20,1.5),(35,1.5),(35,15),(35,42.5),(20,42.5),(5,42.5),(-10.5,42.5),(-10.5,1.5)]), -17, 3, 15),
        5: (densify([(-14,3),(-14,22),(-14,40),(2,40),(27,40),(27,22),(27,0),(40,0),(60,3),(75,3),(90,3),(105,3),(105,22),(105,40),(90,40),(75,40),(60,40),(42.5,40),(27,40),(27,22),(27,3),(2,3)]), -14, 3, 15),
    },
    6: {  # route 6 (contX_7.py)
        0: (densify([(-14,-3),(-14,-22),(-14,-40),(2,-40),(21,-40),(42.5,-40),(60,-40),(60,-22),(60,-3),(75,-3),(90,-3),(105,-3),(105,-22),(105,-40),(90,-40),(75,-40),(60,-40),(60,-22),(60,-3),(42.5,-3),(21,-3),(2,-3)]), -17, -3, 15),
        1: (densify([(-11,-4.5),(0,-4.5),(15,-4.5),(30,-4.5),(40,-4.5),(57,-4.5),(57,-21.5),(57,-38),(70,-38),(85,-38),(100,-38),(114.5,-38),(114.5,-21.5),(114.5,-4.5),(114.5,15),(114.5,30),(114.5,45),(100,45),(85,45),(70,45),(57,45),(40,45),(25,45),(5,45),(-11,45),(-11,30),(-11,15)]), -14, -3, 15),
        2: (densify([(-16,-1.5),(-30,-1.5),(-45,-1.5),(-45,-21.5),(-45,-40),(-25,-40),(-10.5,-40),(-10.5,-20),(-10.5,-1.5),(5,-1.5),(20,-1.5),(35,-1.5),(48,-1.5),(62,-1.5),(62,-20),(62,-42.5),(42,-42.5),(35,-42.5),(20,-42.5),(5,-42.5),(-10.5,-42.5),(-10.5,-20),(-10.5,-5)]), -17, 0, 15),
        3: (densify([(-14,4.5),(0,4.5),(15,4.5),(30,4.5),(40,4.5),(57,4.5),(57,21.5),(57,38),(70,38),(85,38),(100,38),(112.5,38),(112.5,21.5),(112.5,4.5),(112.5,-15),(112.5,-30),(112.5,-45),(100,-45),(85,-45),(70,-45),(57,-45),(40,-45),(25,-45),(5,-45),(-11,-45),(-11,-30),(-11,-15),(-11,4.5)]), -14, 0, 15),
        4: (densify([(-16,1.5),(-30,1.5),(-45,1.5),(-45,21.5),(-45,40),(-25,40),(-10.5,40),(-10.5,15),(-10.5,1.5),(5,1.5),(20,1.5),(35,1.5),(48,1.5),(62,1.5),(62,15),(62,42.5),(42,42.5),(35,42.5),(20,42.5),(5,42.5),(-10.5,42.5),(-10.5,15),(-10.5,5)]), -17, 3, 15),
        5: (densify([(-14,3),(-14,22),(-14,40),(2,40),(21,40),(42.5,40),(60,40),(60,22),(60,3),(75,3),(90,3),(105,3),(105,22),(105,40),(90,40),(75,40),(60,40),(60,22),(60,3),(42.5,3),(21,3),(2,3)]), -14, 3, 15),
    },
}


class VisionTrack:
    def __init__(self):
        # 从 namespace 解析: /drone_X/... → X
        ns = rospy.get_namespace()
        parts = [p for p in ns.strip('/').split('/') if p]
        self.drone_id = 0
        for p in parts:
            if p.startswith('drone_'):
                try:
                    self.drone_id = int(p.split('_')[1])
                except ValueError:
                    pass
                break

        # ===== 读取路线参数 =====
        self.route_id = rospy.get_param("~route_id", 0)
        if self.route_id < 0 or self.route_id > 6:
            self.route_id = 0

        # 获取当前无人机的路线
        route_data = ROUTES.get(self.route_id, {}).get(self.drone_id)
        if route_data is None:
            rospy.logwarn(f"[d{self.drone_id}] route {self.route_id} not found for drone, using default")
            route_data = ROUTES[0].get(self.drone_id)

        self.waypoints, self.off_x, self.off_y, self.route_speed = route_data
        self.wp_idx = 0
        self.target_wp = self.waypoints[0] if self.waypoints else (0, 0)

        rospy.loginfo(f"[d{self.drone_id}] route={self.route_id}, waypoints={len(self.waypoints)}, offset({self.off_x},{self.off_y})")

        self.bridge = CvBridge()

        # 状态
        self.current_pos = Point()
        self.origin_samples = []   # 起飞前前10帧位置, 平均作为局部系原点
        self.origin = None
        self.cam_q = None          # 相机姿态 (来自mavros/local_position/pose)
        self.body_q = None         # mavros 机体姿态四元数
        self.latest_img = None
        self.latest_depth = None
        self.locked_target = None
        self.last_lock_assert = rospy.Time.now()  # 定期重申锁, 防中央重启丢锁
        self.red_lock_box = None  # 锁定红色实例的框 (两个红都是cls=4, 用IoU面积重叠锁定防来回跳)
        self.lock_slot = None      # 上报给协调器的锁槽: 0-3=单实例类, 4/5=红槽
        self.red_topic_pos = {4: None, 5: None}  # 各红槽 topic 最新 (x, y, stamp)
        self.last_pub_red = None       # 本机最近发布的红坐标 (x, y, stamp)
        self.red_dup_since = None      # 自愈: 与另一槽坐标重合起始时间
        self.red_mismatch_since = None  # 抢槽被拒监控起始时间
        self.is_tracking = False
        self.track_enable = False  # 巡逻节点: 起飞≥2m后发true, 否则false
        self.target_lost_time = None
        self.far_depth_time = None  # 深度≤8m的计时起点 (15s后丢目标判定目标可能被消除)
        self.near_depth_time = None  # 第一次进入 6m 内的时间戳，None 表示还没进入过
        self.track_path = []       # 追踪路线记录 (x, y) 二元组
        self.returning = False     # 按记录路线原路返回中
        self.return_start_time = None
        self.lost_pos = None       # 丢失位置 (返回前4s机头朝向)
        self.current_target_height = TARGET_HEIGHT_SEARCH  # 当前目标高度

        # lock map
        self.lock_map = [-1] * 6
        self.lock_map_ok = False

        # 调试/控制状态
        self.dbg_state = "INIT"
        self.last_lin_x = 0.0
        self.last_ang_z = 0.0
        self.last_depth = -1.0
        self.last_cx = 0
        self.last_cy = 0
        self.last_tx = 0.0
        self.last_ty = 0.0

        # 速度（用于避障方向判断）
        self.odom_vx = 0.0
        self.odom_vy = 0.0

        # YOLO 检测线程: 控制循环永不阻塞, OFFBOARD指令流不中断
        self.dets_lock = threading.Lock()
        self.latest_dets = []
        self.img_seq = -1

        # 激光雷达避障（带0.25秒保持）
        self.lidar_avoid_vel = None
        self.lidar_avoid_time = rospy.Time.now()
        self.avoid_vel_hold_time = 0.15

        # 目标发布节流
        self.last_target_pub_time = rospy.Time.now()
        self.target_pub_interval = 0.5  # 2Hz

        # 订阅
        ns = f"/typhoon_h480_{self.drone_id}"
        rospy.Subscriber(ns + "/realsense/depth_camera/color/image_raw", Image, self.cb_img, queue_size=1)
        rospy.Subscriber(ns + "/realsense/depth_camera/depth/image_raw", Image, self.cb_depth, queue_size=1)
        # 使用 mavros/local_position/pose 获取位置和姿态（相机固定在机体上）
        rospy.Subscriber(ns + "/mavros/local_position/pose", PoseStamped, self.cb_pose)
        rospy.Subscriber(ns + "/mavros/local_position/velocity_local", TwistStamped, self.cb_vel)
        rospy.Subscriber("/lock_map", LockMap, self.cb_lockmap)
        # ===== 激光雷达订阅 =====
        rospy.Subscriber(ns + "/scan", LaserScan, self.cb_scan)
        # 巡逻节点决定的追踪许可 (起飞≥2m 后为 true)
        rospy.Subscriber(f"/drone_{self.drone_id}/track_enable", Bool, self.cb_track_enable)
        # 红槽坐标互见: 用于抢槽判定与"追重让位"自愈
        rospy.Subscriber("/actor_red1_info", ActorInfo, self.cb_red1)
        rospy.Subscriber("/actor_red2_info", ActorInfo, self.cb_red2)

        # 发布
        self.tracking_pub = rospy.Publisher(f"/drone_{self.drone_id}/tracking", Bool, queue_size=1)
        self.target_pub = rospy.Publisher(f"/drone_{self.drone_id}/target_world", Point, queue_size=1)
        # 速度指令走XTDrone防断流通信节点
        self.vel_pub = rospy.Publisher(f"/xtdrone/typhoon_h480_{self.drone_id}/cmd_vel_flu", Twist, queue_size=10)
        self.vel_enu_pub = rospy.Publisher(f"/xtdrone/typhoon_h480_{self.drone_id}/cmd_vel_enu", Twist, queue_size=10)
        self.lock_pub = rospy.Publisher(f"/typhoon_h480_{self.drone_id}/lock_report", LockReport, queue_size=1)
        self.actor_pubs = {
            0: rospy.Publisher("/actor_green_info", ActorInfo, queue_size=1),
            1: rospy.Publisher("/actor_blue_info", ActorInfo, queue_size=1),
            2: rospy.Publisher("/actor_brown_info", ActorInfo, queue_size=1),
            3: rospy.Publisher("/actor_white_info", ActorInfo, queue_size=1),
            4: rospy.Publisher("/actor_red1_info", ActorInfo, queue_size=1),
            5: rospy.Publisher("/actor_red2_info", ActorInfo, queue_size=1),
        }

        rospy.loginfo(f"VisionTrack drone{self.drone_id} ready, route={self.route_id}, offset({self.off_x},{self.off_y})")

    # ---- callbacks ----
    def cb_img(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.img_seq = msg.header.seq
        except Exception:
            pass

    def cb_depth(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception:
            pass

    def cb_pose(self, msg):
        self.current_pos = msg.pose.position
        self.body_q = msg.pose.orientation
        # 相机固定在机体上，姿态与机体相同
        self.cam_q = msg.pose.orientation

        if self.origin is None:
            self.origin_samples.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
            if len(self.origin_samples) >= 10:
                n = len(self.origin_samples)
                self.origin = (sum(s[0] for s in self.origin_samples)/n,
                               sum(s[1] for s in self.origin_samples)/n,
                               sum(s[2] for s in self.origin_samples)/n)
                rospy.loginfo(f"[d{self.drone_id}] origin=({self.origin[0]:.2f},{self.origin[1]:.2f},{self.origin[2]:.2f})")

    def cb_lockmap(self, msg):
        self.lock_map = list(msg.locked_by)
        self.lock_map_ok = True

    def cb_vel(self, msg):
        self.odom_vx = msg.twist.linear.x
        self.odom_vy = msg.twist.linear.y

    def cb_track_enable(self, msg):
        self.track_enable = msg.data

    def cb_red1(self, msg):
        self.red_topic_pos[4] = (msg.x, msg.y, rospy.Time.now())

    def cb_red2(self, msg):
        self.red_topic_pos[5] = (msg.x, msg.y, rospy.Time.now())

    # ---- laser scan callback ----
    def cb_scan(self, msg):
        if self.world_z() < 1.0:
            self.lidar_avoid_vel = None
            return

        # ===== 根据状态决定避障距离和速度 =====
        if self.returning:
            obstacle_dist = 2.0
            avoid_speed = 4.5
        elif self.last_depth >= 0:
            obstacle_dist = 2.0
            avoid_speed = 4.5
        else:
            obstacle_dist = 2.5
            avoid_speed = 4.5

        # ===== 获取机体系下的速度方向 =====
        vx_world = self.odom_vx
        vy_world = self.odom_vy

        yaw = self.get_body_yaw()
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_body = cos_yaw * vx_world + sin_yaw * vy_world
        vy_body = -sin_yaw * vx_world + cos_yaw * vy_world

        vel_mag = math.hypot(vx_body, vy_body)
        if vel_mag < 0.3:
            self.lidar_avoid_vel = None
            return

        move_angle = math.atan2(vy_body, vx_body)
        FOV_HALF = 20.0 * math.pi / 180.0

        min_dist = 100.0
        min_angle = 0.0
        found = False

        # ===== 统计正前方左右40度内的障碍物数量（仅在最近障碍物在正前方时使用） =====
        left_count = 0
        right_count = 0

        for i, d in enumerate(msg.ranges):
            if d < obstacle_dist and d > 0.1:
                angle = msg.angle_min + i * msg.angle_increment
                diff = angle - move_angle
                while diff > math.pi:
                    diff -= 2 * math.pi
                while diff < -math.pi:
                    diff += 2 * math.pi

                if abs(diff) <= FOV_HALF:
                    if d < min_dist:
                        min_dist = d
                        min_angle = angle
                        found = True

                    # 统计正前方 ±35° 内的左右障碍物数量
                    if diff > 0:
                        right_count += 1
                    elif diff < 0:
                        left_count += 1

        if found and min_dist < obstacle_dist:
            # ===== 避障速度：只给x,y，不给偏航 =====
            vx = math.cos(min_angle) * avoid_speed
            vy = math.sin(min_angle) * avoid_speed

            vy_extra = 0.0

            # ===== 判断最近障碍物是否在正前方（±20度以内） =====
            if abs(min_angle) < 20.0 * math.pi / 180.0:
                # ===== 正前方：根据左右障碍物分布决定方向 =====
                total = left_count + right_count
                if total > 0:
                    if left_count > right_count:
                        vy_extra = avoid_speed * 4.5      # 左边多 → 向右
                    elif right_count > left_count:
                        vy_extra = -avoid_speed * 4.5     # 右边多 → 向左
                    else:
                        vy_extra = 4.5
                        if np.random.random() > 0.5:
                            vy_extra = -vy_extra
                else:
                    vy_extra = 4.5
                    if np.random.random() > 0.5:
                        vy_extra = -vy_extra
            else:
                # ===== 非正前方：使用原来的逻辑 =====
                if abs(min_angle) < 0.25:
                    vy_extra = 4.5
                    if np.random.random() > 0.5:
                        vy_extra = -vy_extra
                elif min_angle > 0:
                    vy_extra = -avoid_speed * 4.5
                else:
                    vy_extra = avoid_speed * 4.5

            # ===== 更新避障速度和触发时间 =====
            self.lidar_avoid_vel = (-vx, -vy + vy_extra)
            self.lidar_avoid_time = rospy.Time.now()
        # 不在这里清空 lidar_avoid_vel，让 vision_control 根据时间决定

    # ---- lock ----
    def get_allowed_classes(self):
        if not self.lock_map_ok:
            return []
        allowed = []
        for cls in range(4):  # 0-3 单实例类: 空闲或本机持有才检测 (cls 5 是今年不存在的目标)
            if self.lock_map[cls] == -1 or self.lock_map[cls] == self.drone_id:
                allowed.append(cls)
        # 红: 4/5 两个槽, 只要还有一槽可用(空闲或本机)就允许检测
        if (self.lock_map[4] in (-1, self.drone_id) or
                self.lock_map[5] in (-1, self.drone_id)):
            allowed.append(4)
        return allowed

    def publish_lock(self, slot, lock):
        msg = LockReport()
        msg.uav_id = self.drone_id
        msg.cls = slot
        msg.lock = lock
        self.lock_pub.publish(msg)

    def release_lock_now(self):
        if self.lock_slot is not None:
            self.publish_lock(self.lock_slot, False)
            self.lock_slot = None
        self.last_pub_red = None
        self.red_dup_since = None
        self.red_mismatch_since = None
        if self.locked_target is not None:
            self.locked_target = None
            self.red_lock_box = None
            rospy.loginfo("[d%d] lock released" % self.drone_id)

    def other_claimed_reds(self):
        """其它无人机正在发布坐标的红位置 (来自本机未持有的那个红槽 topic)"""
        out = []
        now = rospy.Time.now()
        for s in RED_SLOTS:
            if self.lock_slot == s:
                continue
            if self.lock_map_ok and self.lock_map[s] in (-1, self.drone_id):
                continue
            info = self.red_topic_pos.get(s)
            if info is not None and (now - info[2]).to_sec() <= RED_POS_FRESH:
                out.append((info[0], info[1]))
        return out

    def depth_invalid_at(self, cx, cy):
        """目标框中心深度无效 (无深度图/越界/非有限/非正)"""
        d_img = self.latest_depth
        if d_img is None:
            return True
        h, w = d_img.shape
        if not (0 <= cx < w and 0 <= cy < h):
            return True
        d = d_img[cy, cx]
        return not (np.isfinite(d) and d > 0)

    def red_yaw_matches_other(self, cx, cy):
        """无深度兜底: 像素yaw(相对机体, atan2(-dx,FX) 内参) + 机体姿态yaw -> 世界系,
        与已发布红位置的世界系方位yaw 相差≤RED_YAW_DUP_DEG -> 同一红已被追踪, 不追;
        与自机发布位置一致则保留 (本机在追的红, 深度暂无效)"""
        if self.body_q is None:
            return False
        yaw_pix = self.get_body_yaw() + math.atan2(-(cx - CX), FX)
        wx, wy = self.world_xy()
        tol = math.radians(RED_YAW_DUP_DEG)
        now = rospy.Time.now()
        mine = self.last_pub_red
        if mine is not None and (now - mine[2]).to_sec() <= RED_POS_FRESH:
            if ang_diff(yaw_pix, math.atan2(mine[1] - wy, mine[0] - wx)) <= tol:
                return False
        for (ox, oy) in self.other_claimed_reds():
            if ang_diff(yaw_pix, math.atan2(oy - wy, ox - wx)) <= tol:
                return True
        return False

    def try_claim_red(self, cx, cy, depth_m):
        """首次拿到红的世界坐标: 未被认领则抢空槽(优先4), 已被认领/无槽则放弃返回"""
        pos = self.compute_target_world(cx, cy, depth_m)
        if pos is None:
            return True  # 姿态未就绪, 下一周期再试
        for (ox, oy) in self.other_claimed_reds():
            if math.hypot(pos[0] - ox, pos[1] - oy) < RED_DUP_DIST:
                rospy.logwarn("[d%d] red at (%.1f,%.1f) already claimed, abort" %
                              (self.drone_id, pos[0], pos[1]))
                self.abandon_red_target()
                return False
        slot = None
        if self.lock_map_ok:
            for s in RED_SLOTS:  # 优先4
                if self.lock_map[s] == -1:
                    slot = s
                    break
        if slot is None:
            rospy.logwarn("[d%d] no free red slot, abort red target" % self.drone_id)
            self.abandon_red_target()
            return False
        self.lock_slot = slot
        self.publish_lock(slot, True)
        rospy.loginfo("[d%d] claimed red slot %d" % (self.drone_id, slot))
        return True

    def abandon_red_target(self):
        """放弃红目标 (已被别人认领/抢槽失败/追重)"""
        self.start_return()

    def supervise_red_lock(self):
        """红槽竞态自愈: (a)抢槽被拒 (b)槽5与槽4追了同一个红"""
        if self.lock_slot not in RED_SLOTS:
            self.red_mismatch_since = None
            self.red_dup_since = None
            return
        if not self.lock_map_ok:
            return
        now = rospy.Time.now()

        # (a) 协调器里该槽已被别人持有 -> 抢占被拒
        if self.lock_map[self.lock_slot] not in (-1, self.drone_id):
            if self.red_mismatch_since is None:
                self.red_mismatch_since = now
            elif (now - self.red_mismatch_since).to_sec() > RED_SLOT_TIMEOUT:
                self.red_mismatch_since = None
                self.resolve_lost_slot()
            return
        self.red_mismatch_since = None

        # (b) 槽5发现与槽4发布的红坐标重合 -> 让位
        if self.lock_slot != 5 or self.lock_map[4] in (-1, self.drone_id):
            self.red_dup_since = None
            return
        other = self.red_topic_pos.get(4)
        mine = self.last_pub_red
        if other is None or mine is None:
            self.red_dup_since = None
            return
        if ((now - other[2]).to_sec() > RED_POS_FRESH or
                (now - mine[2]).to_sec() > RED_POS_FRESH or
                math.hypot(other[0] - mine[0], other[1] - mine[1]) >= RED_DUP_DIST):
            self.red_dup_since = None
            return
        if self.red_dup_since is None:
            self.red_dup_since = now
        elif (now - self.red_dup_since).to_sec() > RED_DUP_HOLD:
            rospy.logwarn("[d%d] same red as slot4 drone, yielding slot5" % self.drone_id)
            self.abandon_red_target()

    def resolve_lost_slot(self):
        """抢槽被拒: 改占另一个空槽; 若追的是同一个红, supervise 的重合检查会随后让位"""
        for s in RED_SLOTS:
            if s != self.lock_slot and self.lock_map[s] == -1:
                rospy.logwarn("[d%d] lost slot %d, switching to slot %d" %
                              (self.drone_id, self.lock_slot, s))
                self.lock_slot = s
                self.publish_lock(s, True)
                return
        rospy.logwarn("[d%d] red slots busy, abort red target" % self.drone_id)
        self.abandon_red_target()

    # ---- yolo (只在线程中调用) ----
    def detect(self):
        if self.latest_img is None:
            return []
        allowed = self.get_allowed_classes()
        if not allowed:
            return []
        results = model(self.latest_img, conf=0.4, verbose=False, classes=allowed)
        dets = []
        for r in results:
            for box in r.boxes:
                cls = int(box.cls)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf)
                dets.append((cls, conf, (x1+x2)//2, (y1+y2)//2, x1, y1, x2, y2))
        if self.locked_target is not None:
            dets = [d for d in dets if d[0] == self.locked_target]
        # 红兜底去重: 深度无效的红框, 若像素yaw与已发布(他机)红位置方位yaw一致(≤4°) -> 已被追踪, 丢弃
        kept = []
        for d in dets:
            if (d[0] == 4 and self.depth_invalid_at(d[2], d[3])
                    and self.red_yaw_matches_other(d[2], d[3])):
                rospy.loginfo_throttle(2.0, "[d%d] red box yaw-matched claimed red, skip" % self.drone_id)
                continue
            kept.append(d)
        dets = kept
        if self.locked_target == 4 and self.red_lock_box is not None and len(dets) >= 2:
            best = max(dets, key=lambda d: self.iou(self.red_lock_box, d[4:8]))
            if self.iou(self.red_lock_box, best[4:8]) < 0.2:
                return []
            return [best]
        dets.sort(key=lambda x: x[1], reverse=True)
        return dets

    def iou(self, a, b):
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        inter = max(0, ix2-ix1) * max(0, iy2-iy1)
        if inter == 0:
            return 0.0
        union = (ax2-ax1)*(ay2-ay1) + (bx2-bx1)*(by2-by1) - inter
        return inter / union

    def detect_loop(self):
        rate = rospy.Rate(20)
        last_seq = -1
        while not rospy.is_shutdown():
            seq = self.img_seq
            if seq != last_seq:
                last_seq = seq
                try:
                    dets = self.detect()
                except Exception as e:
                    dets = []
                    rospy.logwarn(f"[d{self.drone_id}] detect error: {e}")
                with self.dets_lock:
                    self.latest_dets = dets
            rate.sleep()

    # ---- vision control ----
    def vision_control(self, det):
        cls, conf, cx, cy = det[0], det[1], det[2], det[3]
        x1, y1, x2, y2 = det[4], det[5], det[6], det[7]

        # ===== 返回状态强制使用返回高度 =====
        if self.returning:
            self.current_target_height = TARGET_HEIGHT_RETURN

        # ===== 激光雷达避障（带0.25秒保持） =====
        if self.lidar_avoid_vel is not None:
            elapsed = (rospy.Time.now() - self.lidar_avoid_time).to_sec()
            if elapsed < self.avoid_vel_hold_time:
                tw = Twist()
                tw.linear.x = self.lidar_avoid_vel[0]
                tw.linear.y = self.lidar_avoid_vel[1]
                tw.linear.z = (self.current_target_height - self.world_z()) * 1.5
                tw.angular.z = 0.0  # 避障时yaw速度为0，保持当前角度
                self.vel_pub.publish(tw)
                return
            else:
                self.lidar_avoid_vel = None

        if self.locked_target != cls:
            if self.locked_target is not None:
                self.release_lock_now()
            self.locked_target = cls
            self.red_lock_box = None
            if cls != 4:
                # 单实例类: 开始追踪即锁; 红: 等首次发布坐标时抢红槽
                self.lock_slot = cls
                self.publish_lock(cls, True)

        if cls == 4:
            box = det[4:8]
            if self.red_lock_box is None:
                self.red_lock_box = tuple(box)
            else:
                self.red_lock_box = tuple(0.7*self.red_lock_box[i] + 0.3*box[i] for i in range(4))

        # ===== 只取目标中心的深度 =====
        depth_m = None
        if self.latest_depth is not None:
            h, w = self.latest_depth.shape
            if 0 <= cx < w and 0 <= cy < h:
                d = self.latest_depth[cy, cx]
                if np.isfinite(d) and d > 0:
                    depth_m = float(d)
            # 中心点无效/越界：depth_m 保持 None，走无深度模式

        # ===== 第一次进入 6m 深度后，才开始发布目标世界坐标 =====
        if depth_m is not None and depth_m <= 10.0 and self.near_depth_time is None:
            self.near_depth_time = rospy.Time.now()   # 第一次进入 6m 内，打时间戳

        if (self.near_depth_time is not None and
                depth_m is not None and self.cam_q is not None):
            now = rospy.Time.now()
            if (now - self.last_target_pub_time).to_sec() >= self.target_pub_interval:
                # 红: 首次拿到坐标时先做认领判定, 通过才抢槽发布
                if cls == 4 and self.lock_slot is None:
                    if not self.try_claim_red(cx, cy, depth_m):
                        return
                self.publish_target_world(cx, cy, depth_m, cls)
                self.last_target_pub_time = now

        if depth_m is not None:
            self.current_target_height = TARGET_HEIGHT_TRACK
        else:
            self.current_target_height = TARGET_HEIGHT_SEARCH

        tw = Twist()

        # ===== 深度有效范围 0.2m ~ 9.8m =====
        if depth_m is not None and 0.2 < depth_m < 10.0:
            tw.linear.x = (depth_m - TRACK_DIST) * 1.5
        else:
            tw.linear.x = 6.0

        dx = cx - CX
        tw.angular.z = math.atan2(-dx, CX) * 1.5
        tw.linear.z = (self.current_target_height - self.world_z()) * 1.5
        #tw.linear.y = math.atan2(-dx, CX) *0.8
        self.vel_pub.publish(tw)

        if depth_m is not None and depth_m <= 8.0 and self.far_depth_time is None:
            self.far_depth_time = rospy.Time.now()

        self.last_lin_x = tw.linear.x
        self.last_ang_z = tw.angular.z
        self.last_depth = depth_m if depth_m is not None else -1.0
        self.last_cx = cx
        self.last_cy = cy

    # ---- world position ----
    def world_z(self):
        if self.origin is not None:
            return self.current_pos.z - self.origin[2]
        return self.current_pos.z

    def world_xy(self):
        if self.origin is not None:
            return (self.current_pos.x - self.origin[0] + self.off_x,
                    self.current_pos.y - self.origin[1] + self.off_y)
        return (self.current_pos.x + self.off_x, self.current_pos.y + self.off_y)

    def record_track(self):
        """记录路径点 (x, y) 二元组"""
        p = self.world_xy()
        if not self.track_path:
            self.track_path.append((p[0], p[1]))
        else:
            lx_, ly_ = self.track_path[-1]
            if math.hypot(p[0]-lx_, p[1]-ly_) >= 1.0:
                self.track_path.append((p[0], p[1]))

    # ===== 检查是否接近路线航点 =====
    def check_near_waypoint(self):
        """检查当前是否接近路线上的某个航点，如果是则切换回巡逻模式"""
        if not self.waypoints:
            return False

        wx, wy = self.world_xy()
        for i, (wpx, wpy) in enumerate(self.waypoints):
            dist = math.hypot(wx - wpx, wy - wpy)
            if dist < 4.5:
                self.wp_idx = i
                self.target_wp = self.waypoints[i]
                rospy.loginfo(f"[d{self.drone_id}] 接近航点 {i} ({wpx:.1f},{wpy:.1f}), 距离={dist:.2f}m, 切换到巡逻模式")
                return True
        return False

    # ===== 开始返回 =====
    def start_return(self):
        self.release_lock_now()
        self.returning = True
        self.is_tracking = True
        self.return_start_time = rospy.Time.now()
        self.lost_pos = self.world_xy()
        self.dbg_state = "RETURN"
        rospy.loginfo("[d%d] 开始返回，保持 tracking=True 防止 C++ 接管", self.drone_id)

    def return_control(self):
        """按记录路线返回，固定高度2.2m，只给x,y,z速度，不给偏航"""
        now = rospy.Time.now()
        if (now - self.return_start_time).to_sec() > 2.0:
            self.release_lock_now()

        if not self.track_path:
            self.returning = False
            self.is_tracking = False
            self.release_lock_now()
            self.track_path = []
            self.dbg_state = "PATROL"
            rospy.loginfo("[d%d] 返回结束（无路径），tracking=False，C++ 接管", self.drone_id)
            return

        # ===== 检查是否接近路线航点 =====
        if self.check_near_waypoint():
            self.returning = False
            self.is_tracking = False
            self.release_lock_now()
            self.track_path = []
            self.dbg_state = "PATROL"
            rospy.loginfo("[d%d] 返回结束（接近航点），tracking=False，C++ 接管", self.drone_id)
            return

        tx, ty = self.track_path[-1]
        wx_, wy_ = self.world_xy()
        dx, dy = tx - wx_, ty - wy_
        d = math.hypot(dx, dy)
        if d < 0.5:
            self.track_path.pop()
            return

        tw = Twist()
        current_z = self.world_z()

        # ===== 高度控制 =====
        target_h = TARGET_HEIGHT_RETURN
        if current_z > target_h + 0.3:
            tw.linear.z = (target_h - current_z) * 0.5
            if tw.linear.z > 0.8: tw.linear.z = 0.8
            if tw.linear.z < -0.8: tw.linear.z = -0.8
        elif current_z < target_h - 0.3:
            tw.linear.z = (target_h - current_z) * 0.5
            if tw.linear.z > 0.8: tw.linear.z = 0.8
            if tw.linear.z < -0.8: tw.linear.z = -0.8
        else:
            tw.linear.z = 0.0

        # ===== 位置返回：只给x,y速度，不给偏航 =====
        sp = 3.5
        if d > 0.3:
            tw.linear.x = dx / d * sp
            tw.linear.y = dy / d * sp
        else:
            tw.linear.x = 0.0
            tw.linear.y = 0.0

        # ===== 返回时不给偏航速度 =====
        tw.angular.z = 0.0

        self.vel_enu_pub.publish(tw)

    def compute_target_world(self, cx, cy, depth_m):
        """计算目标世界坐标 (云台偏航跟随无人机，横滚=0，俯仰固定); 姿态未就绪返回 None"""
        x_cam = (cx - CX) * depth_m / FX
        y_cam = (cy - CY) * depth_m / FY
        z_cam = depth_m
        p_cam = np.array([x_cam, y_cam, z_cam])

        if self.body_q is None:
            return None

        # ===== 从四元数提取偏航角 =====
        q = self.body_q
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # ===== 云台姿态：偏航跟随无人机，横滚=0，俯仰固定 =====
        PITCH_FIXED = GIMBAL_PITCH
        ROLL_FIXED = 0.0

        # 构建云台旋转矩阵：R = R_yaw * R_pitch * R_roll
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(PITCH_FIXED), math.sin(PITCH_FIXED)
        cr, sr = math.cos(ROLL_FIXED), math.sin(ROLL_FIXED)  # cr=1, sr=0

        R_yaw = np.array([[cy_, -sy_, 0], [sy_, cy_, 0], [0, 0, 1]])
        R_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_roll = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

        R_gimbal = R_yaw @ R_pitch @ R_roll

        # 相机光学系→IMU系
        R_opt2imu = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])

        # 目标在相机系下转到世界系
        p_world_cam = R_gimbal @ R_opt2imu @ p_cam

        # 相机安装位置（机体坐标系下）
        R_body = self.quat_to_mat(q)
        cam_pos_body = R_body @ CAM_OFFSET

        # 无人机世界位置
        if self.origin is not None:
            drone_world = np.array([self.current_pos.x - self.origin[0] + self.off_x,
                                    self.current_pos.y - self.origin[1] + self.off_y,
                                    self.current_pos.z - self.origin[2]])
        else:
            drone_world = np.array([self.current_pos.x + self.off_x,
                                    self.current_pos.y + self.off_y,
                                    self.current_pos.z])

        target = drone_world + cam_pos_body + p_world_cam

        return (target[0], target[1], target[2])

    def publish_target_world(self, cx, cy, depth_m, cls):
        """发布目标世界坐标 (红只发到本机红槽对应的 red topic, 两红同时追互不串)"""
        target = self.compute_target_world(cx, cy, depth_m)
        if target is None:
            return

        pt = Point()
        pt.x, pt.y, pt.z = target[0], target[1], target[2]
        self.target_pub.publish(pt)
        self.last_tx = pt.x
        self.last_ty = pt.y

        actor = ActorInfo()
        actor.x = pt.x
        actor.y = pt.y
        if cls == 4:
            if self.lock_slot in RED_SLOTS:
                actor.cls = "red"
                self.actor_pubs[self.lock_slot].publish(actor)
                self.last_pub_red = (pt.x, pt.y, rospy.Time.now())
        else:
            actor.cls = ["green", "blue", "brown", "white"][cls]
            self.actor_pubs[cls].publish(actor)

    def quat_to_mat(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])

    def get_body_yaw(self):
        if self.body_q is None:
            return 0.0
        q = [self.body_q.x, self.body_q.y, self.body_q.z, self.body_q.w]
        _, _, yaw = euler_from_quaternion(q)
        return yaw

    def publish_hold(self):
        tw = Twist()
        tw.linear.x = self.last_lin_x
        tw.linear.y = 0.0
        tw.linear.z = (self.current_target_height - self.world_z()) * 1.5
        self.vel_pub.publish(tw)
        self.last_ang_z = 0.0

    # ---- main loop ----
    def run(self):
        rospy.sleep(3.0)
        t = threading.Thread(target=self.detect_loop)
        t.daemon = True
        t.start()

        rate = rospy.Rate(60)
        dbg_n = 0
        while not rospy.is_shutdown():
            if not self.track_enable:
                # 巡逻节点未允许追踪 (起飞未过2m): 不进入追踪
                self.dbg_state = "WAIT_TAKEOFF"
                if self.track_path:
                    self.returning = True
                    self.is_tracking = True
                    self.return_control()
                    self.tracking_pub.publish(Bool(True))
                else:
                    self.is_tracking = False
                    self.tracking_pub.publish(Bool(False))
                rate.sleep()
                continue

            with self.dets_lock:
                dets = list(self.latest_dets)

            if self.returning:
                if dets:
                    self.returning = False
                    self.dbg_state = "TRACK"
                    self.vision_control(dets[0])
                    self.target_lost_time = None
                    self.tracking_pub.publish(Bool(True))
                else:
                    self.return_control()
                    self.tracking_pub.publish(Bool(True))
                rate.sleep()
                continue

            if dets:
                if not self.is_tracking:
                    self.is_tracking = True
                    self.track_path = []
                    self.far_depth_time = None
                    self.near_depth_time = None      # 新一次追踪，重置
                self.dbg_state = "TRACK"
                self.record_track()
                self.vision_control(dets[0])
                self.target_lost_time = None
            elif self.is_tracking:
                if self.target_lost_time is None:
                    self.target_lost_time = rospy.Time.now()
                lost_t = (rospy.Time.now() - self.target_lost_time).to_sec()
                depth_valid = self.last_depth >= 0

                if depth_valid:
                    if lost_t > 0.20:
                        self.dbg_state = "SEARCH"
                        self.start_return()
                    else:
                        self.dbg_state = "LOST_HOLD"
                        self.publish_hold()
                else:
                    if lost_t < 0.2:
                        self.dbg_state = "LOST_HOLD"
                        self.publish_hold()
                    else:
                        self.dbg_state = "SEARCH"
                        self.start_return()
            else:
                self.dbg_state = "PATROL"

            self.supervise_red_lock()

            if self.lock_slot is not None:
                self.publish_lock(self.lock_slot, True)

            if self.returning:
                self.tracking_pub.publish(Bool(True))
            else:
                self.tracking_pub.publish(Bool(self.is_tracking))

            dbg_n += 1
            if dbg_n % 40 == 0:
                rospy.loginfo("[d%d] %s x=%.1f y=%.1f z=%.2f depth=%.1f lin.x=%.2f lock=%s path_len=%d" % (
                    self.drone_id, self.dbg_state,
                    self.current_pos.x, self.current_pos.y, self.current_pos.z,
                    self.last_depth, self.last_lin_x,
                    self.locked_target, len(self.track_path)))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("vision_track")
    try:
        VisionTrack().run()
    except rospy.exceptions.ROSInterruptException:
        pass
