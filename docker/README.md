# 六机协同算法 Docker 镜像

Ubuntu 20.04 + ROS1 Noetic，只包含算法程序，不含 Gazebo / PX4 等仿真环境。
运行时由宿主机启动 Gazebo 世界 + PX4 SITL + mavros，容器通过 ROS 网络直接对接。

镜像名 **tdfly001**，容器名建议 **TDfly001**（Docker 规定镜像名必须全小写，容器名可以大写）。

## 镜像内容

| 内容 | 说明 |
|---|---|
| `simple_explore` ×6 (C++) | 巡逻 / OFFBOARD切换 / 解锁起飞 / 激光避障，编在 `/home/lx/catkin_ws/devel` |
| `vision_track.py` ×6 | YOLO 视觉追踪，模型 `best_11s_new.pt` 已打进镜像 |
| `int.py` | 红/绿/蓝/棕/白 锁协调器 |
| XTDrone 通信节点 | `multirotor_communication.py` + `multi_vehicle_communication.sh`（`/home/lx/XTDrone/communication/`，手动启动） |
| 消息包 | `ros_msg`、`ros_actor_cmd_pose_plugin_msgs`（等 6 个类）源码编译 |
| Python 栈 | torch 2.4.1+cu121、ultralytics 8.4.24、numpy 1.24.4（与开发机逐版本一致） |

容器内路径与开发机**完全一致**（`/home/lx/robocup_gazebo/src`、`/home/lx/catkin_ws`），
脚本里的绝对路径、模型路径都不用改。

## 1. 开发机上构建

```bash
bash /home/lx/robocup_gazebo/docker/build.sh          # 构建 tdfly001:latest
docker images tdfly001
```

## 2. 导出 / 导入（离线部署时这样传）

```bash
# 开发机
docker save tdfly001:latest | gzip > tdfly001.tar.gz     # 约 7~8GB

# 部署机
docker load < tdfly001.tar.gz
```

## 3. 部署机运行

前提：装了 NVIDIA 驱动 + `nvidia-container-toolkit`（`docker info` 的 Runtimes 里能看到 nvidia）。

**方式一：每次全新容器（退出即删，环境最干净）**

```bash
docker run -it --rm \
  --gpus all \
  --network host \
  --name TDfly001 \
  tdfly001:latest
```

**方式二：固定容器，用 start/exec（贴合日常习惯）**

```bash
# 宿主机第一次（仅一次，注意不加 --rm）：
docker run -it --gpus all --network host --name TDfly001 tdfly001:latest
# exit 出来后容器停止但保留，之后每次：
docker start TDfly001
docker exec -it TDfly001 bash
```

两种别混用：方式二的容器如果当初用了 `--rm`，退出就被删了，没法 `docker start`。

| 参数 | 作用 |
|---|---|
| `--gpus all` | 宿主机的 GPU 对容器可见；没有 nvidia-container-toolkit 时可去掉，自动退回 CPU（会慢） |
| `--network host` | 与宿主机的 Gazebo/mavros 共用 ROS 网络（同一台机器时用这个） |
| `--name TDfly001` | 容器名（可大写），进去用 `docker exec -it TDfly001 bash` |
| 宿主机和容器不在同一台机器 | 用 `-e ROS_MASTER_URI=http://<宿主机IP>:11311 -e ROS_IP=<本机IP>` 替代 `--network host` |

## 4. 容器内启动

控制指令经 XTDrone 通信节点中继（Twist → mavros setpoint），**必须先手动启动**（不写进 start_all.sh）：

```bash
# 1) 终端1: XTDrone 通信中继 (6 机)
cd /home/lx/XTDrone/communication && ./multi_vehicle_communication.sh

# 2) 终端2: 算法全栈
start_all.sh 0        # 参数 = route_id (0~6)，默认 0
```

`start_all.sh` 等价于：先 `roslaunch drone_ctrl simple.launch route_id:=0`（int + 6路视觉），
6 秒后 `roslaunch drone_ctrl vision.launch route_id:=0`（6路 C++ 控制，负责起飞）。
Ctrl+C 停止全部。

手动分步：

```bash
roslaunch drone_ctrl simple.launch route_id:=0     # 终端1
roslaunch drone_ctrl vision.launch route_id:=0     # 终端2 (稍后)
```

日志在 `~/.ros/log`；调试版 `check_target.py`、`yolo_view.py` 也都在 `/home/lx/robocup_gazebo/src` 里。

## 5. 改了代码怎么办

- **只改 Python**（vision_track.py 等）：直接重新构建镜像即可（见第 1 节），或运行时把代码目录挂进去：
  ```bash
  docker run -it --rm --gpus all --network host --name TDfly001 \
    -v /home/lx/robocup_gazebo/src:/home/lx/robocup_gazebo/src \
    tdfly001:latest
  ```
- **改了 C++**（simple_explore.cpp）：挂载代码后容器内重新编：
  ```bash
  catkin build drone_ctrl        # 在 /home/lx/catkin_ws 下
  ```

## 6. 常见问题

| 现象 | 处理 |
|---|---|
| `[warn] 连不上 ROS master` | Gazebo 没起 or 少了 `--network host` |
| `cuda: False (CPU模式)` | 部署机缺 nvidia-container-toolkit，或 run 时忘了 `--gpus all` |
| 节点找不到 (`int.sh` 等) | 镜像构建时 `catkin build` 失败过，重新 `bash build.sh` 看构建日志 |
| 想看当前 ROS 是否通 | 容器内 `rostopic list`、`rostopic hz /typhoon_h480_0/mavros/state` |
