# Multi-Rotor UAV Swarm Coordinated Exploration

**基于 ROS / PX4 SITL / Gazebo 的多机协同仿真项目**

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)](https://wiki.ros.org/noetic)
[![PX4](https://img.shields.io/badge/PX4-SITL-3D7EBB)](https://px4.io/)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-F58113)](https://gazebosim.org/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-00A3FF)](https://github.com/ultralytics/ultralytics)
[![Python](https://img.shields.io/badge/Python-3.8+-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> **English abstract** — A ROS 1 / PX4 SITL / Gazebo simulation project in which six quadrotors
> cooperatively patrol an area, detect ground targets with YOLOv8, and track them under a
> centralized lock protocol that guarantees each target is tracked by at most one UAV.

## 项目简介

本项目是 6 架 `typhoon_h480` 多旋翼无人机在 Gazebo / PX4 SITL 仿真中的**集群协同巡逻与目标追踪**方案：
无人机编队起飞后按全局路线分区巡逻，机载视觉实时检测地面目标
（绿 / 蓝 / 棕 / 白 / 红），发现目标后由视觉节点接管追踪，追踪结束后自动返回原巡逻路线。

系统由四部分组成：

| 模块 | 语言 | 职责 |
|---|---|---|
| `simple_explore` ×6 | C++ | OFFBOARD 切换、解锁、起飞、多航点巡逻、激光雷达避障 |
| `vision_track.py` ×6 | Python | YOLOv8 检测、深度/像素定位、追踪控制、丢目标返回 |
| `int.py` | Python | 中央锁协调器，保证同一目标最多被一架无人机追踪 |
| XTDrone 通信中继 ×6 | Python | 把速度指令以 30 Hz 常流转换为 mavros `setpoint_raw/local` |

> 本仓库只包含**机载算法与控制**。Gazebo 世界、PX4 SITL 六机、mavros 与 actor 插件等
> 仿真资产需另行准备，仓库中不包含。

## 主要特性

- **6 机并行协同**：7 条全局巡逻路线 × 6 架无人机，路线、起飞点坐标化配置
- **全自动起飞巡逻**：OFFBOARD → ARM → 起飞至 3.5 m → 航点巡航，激光雷达前方避障
- **YOLOv8 实时检测**：自训练模型，检测绿 / 蓝 / 棕 / 白 / 红五类地面目标
- **集中式目标锁**：`LockMap` / `LockReport` 协议，单实例目标只由一架无人机追踪，
  避免多机重复追踪同一目标
- **红色双目标双锁槽**：两个红人均为同一检测类，通过槽位 4/5 分配给两架不同无人机**同时追踪**
- **双重去重兜底**：世界坐标距离（2.4 m）+ 无深度时的像素方位角（≤4°）判定，防止漏追 / 重追
- **追踪与巡逻解耦**：巡逻节点通过 `track_enable` 授予追踪许可，起飞高度达标后才允许追踪
- **掉线自愈**：视觉节点心跳丢失 3 s 后，C++ 节点自动恢复巡逻
- **Docker 一键部署**：CUDA + PyTorch + Ultralytics + 编译产物打包为镜像，路径与开发机一致

## 效果演示

[![六机协同巡逻与目标追踪演示](docs/robocup_gazebo_poster.jpg)](docs/robocup_gazebo_demo.mp4)

▶ **[观看完整演示视频（约 83 秒，MP4）](docs/robocup_gazebo_demo.mp4)**

演示画面：上方为 6 路机载相机视角，右侧为 Gazebo 场景视角，中下方为仿真计分面板
（实时得分与剩余目标），四周为各节点运行日志。视频完整展示了 6 机编队起飞、分区巡逻、
YOLO 发现目标、锁协调分配与持续追踪直至得分的全过程。

> 提示：GitHub 网页端如无法直接播放 MP4，可点击链接下载观看；在 GitHub README 编辑器中
> 直接拖入视频文件可生成内嵌播放器。

## 系统架构

```
                    ┌───────────────────────────────────────────┐
                    │        Gazebo 11 + PX4 SITL + mavros      │
                    │        typhoon_h480_0 ... typhoon_h480_5  │
                    └──────▲──────────────────────────▲─────────┘
        传感器 / 位姿       │                          │  setpoint_raw/local
                           │                          │
        ┌──────────────────┴──────────┐   ┌───────────┴─────────────────┐
        │  vision_track.py ×6         │   │  simple_explore (C++) ×6    │
        │  YOLOv8 检测 / 追踪 / 返回    │◄──┤  起飞 / 巡逻 / 避障           │
        │                             │   │  track_enable / tracking     │
        └──────────┬──────────────────┘   └─────────────────────────────┘
                   │ Twist (cmd_vel_flu / cmd_vel_enu)
        ┌──────────▼──────────────────┐
        │  XTDrone 通信中继 ×6         │   ← 需手动启动
        │  30 Hz 常流 + 悬停防抖        │
        └──────────┬──────────────────┘
                   │ PositionTarget
                   ▼  (PX4 / mavros)

        int.py  ◄── /typhoon_h480_i/lock_report ── vision_track ×6
           └──────► /lock_map (50 Hz)
```

## 仓库结构

```
robocup_gazebo/
├── src/
│   ├── vision_track.py                 # 视觉检测 + 追踪 + 返回（核心 Python 节点）
│   ├── int.py                          # 中央锁协调器
│   ├── best_11s_new.pt                 # 当前使用的 YOLOv8 权重
│   ├── best.pt / yolo_26s.pt           # 其他训练版本（可选）
│   ├── check_target.py                 # 目标定位调试
│   ├── yolo_view.py                    # YOLO 实时可视化
│   ├── save_images.py                  # 数据集采集
│   ├── gimbal_*.py                     # 云台控制 / 调试
│   ├── typhoon_h480_depth.sdf          # 仿真模型（深度相机 + 云台）
│   ├── gazebo_gimbal_controller_plugin.cpp
│   ├── drone_ctrl/                     # catkin 包：C++ 控制 + launch + 启动脚本
│   │   ├── src/simple_explore.cpp      # 巡逻 / 起飞 / 避障主逻辑
│   │   ├── launch/{simple,vision,drones,full}.launch
│   │   └── scripts/*.sh                # roslaunch 包装脚本
│   └── drone_view/                     # 可视化辅助包（可选）
├── catkin_ws/src/
│   ├── ros_msg/                        # LockMap / LockReport / TargetRaw 消息
│   └── gazebo_ros_actor_cmd_plugin_msgs/
├── XTDrone/communication/              # XTDrone 通信中继（Twist → setpoint）
├── tools/check_routes.py               # 校验 C++ 与 Python 路线是否一致
├── docker/                             # Dockerfile + 构建 / 启动脚本
└── README.md
```

## 环境依赖

- Ubuntu 20.04 + ROS 1 Noetic
- PX4 SITL（6 架 `typhoon_h480`）、Gazebo 11、mavros
- [XTDrone](https://github.com/robin-shaun/XTDrone) 仿真环境（提供机型、控制与通信脚本）
- C++：`roscpp`、`mavros_msgs`、`sensor_msgs`、`std_msgs`、`nav_msgs`、`pcl_ros`、`pcl_conversions`、Gazebo 11 开发库
- Python 3.8+：`torch 2.4.1`、`ultralytics 8.4.24`、`numpy 1.24.4`、`opencv-python 4.13`、
  `pyquaternion`、`cv_bridge`、`tf`

## 快速开始

> 脚本和模型路径默认按 `/home/lx/robocup_gazebo` 布置（Docker 镜像内路径固定）。
> 克隆到其他位置时，需要相应修改 `src/drone_ctrl/scripts/*.sh`、模型路径与 Dockerfile。

### 1. 编译工作空间

```bash
cd ~/catkin_ws
catkin init

# 消息包与 drone_ctrl 加入工作空间
ln -sfn /home/lx/robocup_gazebo/catkin_ws/src/ros_msg src/ros_msg
ln -sfn /home/lx/robocup_gazebo/catkin_ws/src/gazebo_ros_actor_cmd_plugin_msgs src/gazebo_ros_actor_cmd_plugin_msgs
ln -sfn /home/lx/robocup_gazebo/src/drone_ctrl src/drone_ctrl

catkin build
source devel/setup.bash
```

### 2. 准备仿真环境

启动 Gazebo 世界、6 架 `typhoon_h480` 的 PX4 SITL 与 mavros，确认：

```bash
rostopic echo -n1 /typhoon_h480_0/mavros/state    # connected: True
rostopic list | grep realsense                    # 相机话题存在
```

### 3. 启动 XTDrone 通信中继（必须先启动）

```bash
cd /home/lx/XTDrone/communication
./multi_vehicle_communication.sh        # 拉起 6 路中继（typhoon_h480_num=6）
```

### 4. 启动算法栈

```bash
# 方式一：一键脚本（容器内为 start_all.sh）
bash docker/start_all.sh 0

# 方式二：手动分步
roslaunch drone_ctrl simple.launch route_id:=0     # int.py + 6 路视觉
sleep 6
roslaunch drone_ctrl vision.launch route_id:=0     # 6 路 C++ 控制（负责起飞）
```

`route_id` 取 0~6，对应 7 套巡逻路线。无人机起飞后自动进入巡逻，检测到目标即转入追踪。

## Docker 部署

部署机推荐使用 Docker，镜像内含全部依赖与编译产物，路径与开发机保持一致：

```bash
# 开发机：构建 / 导出
bash docker/build.sh
docker save tdfly001:latest | gzip > tdfly001.tar.gz

# 部署机：导入 / 运行
docker load < tdfly001.tar.gz
docker run -it --rm --gpus all --network host --name TDfly001 tdfly001:latest

# 容器内
cd /home/lx/XTDrone/communication && ./multi_vehicle_communication.sh   # 终端 1
start_all.sh 0                                                         # 终端 2
```

镜像构建、网络模式、GPU、常见问题等详见 [docker/README.md](docker/README.md)。

## 配置说明

### 路线

- 巡逻路线定义在两处，**必须完全一致**：
  - C++：`src/drone_ctrl/src/simple_explore.cpp` 的 `cfgs[7][6]`
  - Python：`src/vision_track.py` 的 `ROUTES`
- 每条记录为 `(原始航点, 起飞点 x, 起飞点 y, 巡航速度)`，运行时按 3 m 步长密化并首尾闭环。
- 修改后运行校验：

  ```bash
  python3 tools/check_routes.py      # 输出“两文件完全一致”即通过
  ```

### 起飞点 / 编队

| 无人机 | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| spawn (x, y) | (-17, -3) | (-14, -3) | (-17, 0) | (-14, 0) | (-17, 3) | (-13.5, 3) |

需同步修改：`drones.launch` 的 `off_x / off_y`、`vision_track.py` 的 `OFFSETS`、
以及 C++ `cfgs` 中每条路线的 `sox / soy`。

### 相机与模型

| 参数 | 值 |
|---|---|
| 分辨率 | 640 × 480 |
| 内参 `CX, CY` | 320.5, 240.5 |
| 内参 `FX, FY` | 554.254691191187 |
| 云台俯仰 | 15°（下俯） |
| 检测模型 | `src/best_11s_new.pt` |

### 关键参数

| 参数 | 位置 | 默认值 |
|---|---|---|
| 起飞高度 `TAKEOFF_H` | `simple_explore.cpp` | 3.5 m |
| 巡航速度 `SPEEDS` | `simple_explore.cpp` 每条路线 | 15 |
| 避障距离 `obstacle_dist` | `simple_explore.cpp` | 4.5 m |
| 追踪距离 `TRACK_DIST` | `vision_track.py` | 5.0 m |
| 追踪 / 搜索 / 返回高度 | `vision_track.py` | 2.3 / 2.3 / 2.2 m |
| 返回速度 | `vision_track.py` | 3.5 |
| 红目标坐标去重阈值 `RED_DUP_DIST` | `vision_track.py` | 2.4 m |
| 无深度方位去重阈值 `RED_YAW_DUP_DEG` | `vision_track.py` | 4° |

### 主要话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/drone_<i>/tracking` | `std_msgs/Bool` | 视觉 → 巡逻：追踪状态 |
| `/drone_<i>/track_enable` | `std_msgs/Bool` | 巡逻 → 视觉：起飞 ≥2 m 后许可追踪（之后保持 true） |
| `/drone_<i>/target_world` | `geometry_msgs/Point` | 当前目标世界坐标 |
| `/xtdrone/typhoon_h480_<i>/cmd_vel_flu` | `geometry_msgs/Twist` | 机体系速度（追踪 / 巡逻） |
| `/xtdrone/typhoon_h480_<i>/cmd_vel_enu` | `geometry_msgs/Twist` | 世界系速度（返回巡逻路线） |
| `/lock_map` | `ros_msg/LockMap` | 中央锁表，50 Hz 广播 |
| `/typhoon_h480_<i>/lock_report` | `ros_msg/LockReport` | 各机锁申请 / 释放 |
| `/actor_{green,blue,brown,white}_info` | `ros_actor_cmd_pose_plugin_msgs/ActorInfo` | 目标世界坐标共享 |
| `/actor_red1_info`, `/actor_red2_info` | `ActorInfo` | 两个红色目标（双锁槽） |
| `/typhoon_h480_<i>/scan` | `sensor_msgs/LaserScan` | 激光避障 |
| `/typhoon_h480_<i>/realsense/depth_camera/*` | `sensor_msgs/Image` | 彩色 / 深度图像 |

## 核心机制

### 职责划分与状态机

- C++ `simple_explore` 负责飞行安全相关流程：OFFBOARD、解锁、起飞、巡逻、避障。
- 当视觉节点发布 `/drone_<i>/tracking = true` 时，C++ 停止发送速度指令，由
  `vision_track.py` 接管控制；追踪结束（`false`）后 C++ 从最近航点恢复巡逻。
- C++ 若 3 s 收不到视觉心跳，会强制退出追踪并恢复巡逻。
- `track_enable` 由 C++ 在起飞高度 ≥2 m 后置 true 并保持，视觉节点只有收到许可后才进入追踪，
  避免地面阶段误检导致撞地。

### 目标锁协调

`int.py` 维护一张 6 槽锁表（0~3 为单实例类，4/5 为红色双槽），各机通过 `LockReport`
申请 / 释放，`LockMap` 以 50 Hz 广播。视觉节点只检测本机持有的类别，从源头避免重复追踪。

### 红色双目标

两个红人在检测模型中同为 `cls=4`，因此使用槽位 4 和 5 让两架无人机分别追踪：

- 首次获得世界坐标时抢占空闲槽（优先 4）；
- 每个红只发布到本机槽位对应的 `actor_red1_info` / `actor_red2_info`；
- 坐标距离 < 2.4 m 视为同一目标，放弃抢锁；
- 深度无效时，用“机头 yaw + 像素方位角”换算世界方位，与已发布位置夹角 ≤4° 视为同一目标。

### 通信中继与悬停防抖

XTDrone 中继以 30 Hz 常流向 PX4 发送 `setpoint_raw/local`。平移速度连续 0.5 s 小于 0.02 m/s
时切换到**位置锁定悬停**；悬停期间 `yaw_rate` 继续转发，保证追踪时云台 / 机头可继续对准目标。

## 调试工具

| 脚本 | 用途 |
|---|---|
| `src/check_target.py` | 打印目标估计坐标与无人机位姿 |
| `src/yolo_view.py` | 实时显示 YOLO 检测结果 |
| `src/save_images.py` | 按无人机保存图像，用于数据集采集 |
| `src/check_gimbal_pitch.py` | 云台姿态角检查 |
| `tools/check_routes.py` | C++ / Python 路线一致性校验 |

## 已知限制

- 依赖 Gazebo 世界、PX4 SITL 六机、mavros 与 actor 插件，本仓库不包含这些仿真资产。
- 脚本与模型路径按 `/home/lx/robocup_gazebo` 固定布局编写。
- 未包含第三方 `ego_planner` 等与本方案运行无关的包。
- 仅在仿真环境中验证，未经真机飞行验证。

## 致谢

- [XTDrone](https://github.com/robin-shaun/XTDrone) — 多机仿真与控制通信框架
- [PX4](https://px4.io/) / [Gazebo](https://gazebosim.org/) — 飞控与仿真
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) — 目标检测
- [ROS](https://www.ros.org/) — 机器人中间件

## License

本项目采用 [MIT License](LICENSE) 开源。
