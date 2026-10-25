# RoboCup 6机算法环境 — Ubuntu 20.04 + ROS1 Noetic (与开发机一致)
# 官方机器负责 Gazebo 世界/PX4 SITL/mavros/裁判, 本镜像只跑算法:
#   simple_explore x6 (C++) + vision_track.py x6 + int.py
#   + XTDrone 通信节点 (multirotor_communication.py, 手动启动)
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1

# apt 换中科大镜像 (与开发机一致), 再装算法依赖
RUN sed -i -e 's|http://archive.ubuntu.com/ubuntu|https://mirrors.ustc.edu.cn/ubuntu|g' \
           -e 's|http://security.ubuntu.com/ubuntu|https://mirrors.ustc.edu.cn/ubuntu|g' /etc/apt/sources.list; \
    for f in /etc/apt/sources.list.d/*.list; do \
        [ -f "$f" ] && sed -i 's|http://packages.ros.org/ros/ubuntu|https://mirrors.ustc.edu.cn/ros/ubuntu|g' "$f"; \
    done; \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake \
      python3-pip python3-catkin-tools \
      ros-noetic-cv-bridge \
      ros-noetic-mavros-msgs \
      ros-noetic-gazebo-msgs \
      python3-tf \
      ros-noetic-pcl-ros ros-noetic-pcl-conversions \
      libpcl-dev libeigen3-dev libgazebo11-dev \
    && rm -rf /var/lib/apt/lists/*

# Python 依赖 (版本与开发机一致): torch 2.4.1+cu121 / ultralytics 8.4.24 / numpy 1.24.4
RUN python3 -m pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple \
      'pip==24.0' 'setuptools<76' wheel \
 && python3 -m pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple \
      numpy==1.24.4 \
      torch==2.4.1 torchvision==0.19.1 \
      ultralytics==8.4.24

# opencv 版本对齐开发机 (ultralytics 默认会拉最新 5.0.0)
RUN python3 -m pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple \
      opencv-python==4.13.0.92

# XTDrone 通信节点依赖 (multirotor_communication.py 用 pyquaternion)
RUN python3 -m pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple pyquaternion

# ---- 算法代码: 容器内路径与开发机完全一致, 脚本/模型路径无需改动 ----
# /home/lx/robocup_gazebo/src : vision_track.py, int.py, drone_ctrl, *.pt 等
# /home/lx/catkin_ws/src      : 自编译消息包 (ros_msg, ros_actor_cmd_pose_plugin_msgs)
COPY robocup_gazebo/src /home/lx/robocup_gazebo/src
COPY catkin_ws/src/ros_msg /home/lx/catkin_ws/src/ros_msg
COPY catkin_ws/src/gazebo_ros_actor_cmd_plugin_msgs /home/lx/catkin_ws/src/gazebo_ros_actor_cmd_plugin_msgs
RUN ln -s /home/lx/robocup_gazebo/src/drone_ctrl /home/lx/catkin_ws/src/drone_ctrl

# ---- 编译 catkin 工作空间 (drone_ctrl + 两个消息包) ----
RUN bash -c 'source /opt/ros/noetic/setup.bash && cd /home/lx/catkin_ws && catkin init && catkin build' \
 && cp -n /home/lx/robocup_gazebo/src/drone_ctrl/scripts/*.sh /home/lx/catkin_ws/devel/lib/drone_ctrl/ \
 && test -x /home/lx/catkin_ws/devel/lib/drone_ctrl/simple_explore \
 && ls -la /home/lx/catkin_ws/devel/lib/drone_ctrl/

# ---- XTDrone 通信节点: 中继 cmd_vel_flu/cmd_vel_enu (Twist) -> mavros setpoint_raw ----
# 手动启动: cd /home/lx/XTDrone/communication && ./multi_vehicle_communication.sh
COPY XTDrone/communication/multirotor_communication.py XTDrone/communication/multi_vehicle_communication.sh /home/lx/XTDrone/communication/
RUN chmod +x /home/lx/XTDrone/communication/multi_vehicle_communication.sh

# ---- 入口: 自动 source ROS + 工作空间, 交互 shell 也生效 ----
COPY entrypoint.sh /entrypoint.sh
COPY start_all.sh /home/lx/robocup_gazebo/src/start_all.sh
RUN chmod +x /entrypoint.sh /home/lx/robocup_gazebo/src/start_all.sh \
 && ln -sf /home/lx/robocup_gazebo/src/start_all.sh /usr/local/bin/start_all.sh \
 && echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
 && echo 'source /home/lx/catkin_ws/devel/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
