#!/bin/bash
# 打包脚本: 收集构建上下文并构建 tdfly001 镜像 (docker 镜像名强制小写)
# 用法: bash build.sh [tag]     (默认 tdfly001:latest)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
SRC=${SRC:-$REPO}
CTX=${CTX:-/tmp/robocup_algo_ctx}
TAG=${1:-tdfly001:latest}

echo "[build] 收集构建上下文 -> $CTX"
rm -rf "$CTX"
mkdir -p "$CTX/robocup_gazebo/src" "$CTX/catkin_ws/src"

# 算法代码 (排除不参与运行的 ego_planner / drone_view / 缓存)
for f in "$SRC"/src/*; do
    case "$(basename "$f")" in
        ego_planner|drone_view|__pycache__|.vscode) echo "[build] skip $(basename "$f")"; continue ;;
    esac
    cp -a "$f" "$CTX/robocup_gazebo/src/"
done

# 自编译消息包
cp -a "$SRC/catkin_ws/src/ros_msg" "$CTX/catkin_ws/src/"
cp -a "$SRC/catkin_ws/src/gazebo_ros_actor_cmd_plugin_msgs" "$CTX/catkin_ws/src/"

# XTDrone 通信节点 (手动启动, 不写进 start_all.sh)
mkdir -p "$CTX/XTDrone/communication"
cp "$SRC/XTDrone/communication/multirotor_communication.py" "$CTX/XTDrone/communication/"
cp "$SRC/XTDrone/communication/multi_vehicle_communication.sh" "$CTX/XTDrone/communication/"

# docker 构建文件
cp "$HERE/Dockerfile" "$HERE/entrypoint.sh" "$HERE/start_all.sh" "$CTX/"

echo "[build] docker build -t $TAG"
docker build -t "$TAG" "$CTX"

echo ""
echo "[build] 完成: $TAG"
docker images "$TAG" --format '  镜像: {{.Repository}}:{{.Tag}}  大小: {{.Size}}'
echo "[build] 导出到文件:  docker save $TAG | gzip > ${TAG%%:*}.tar.gz"
