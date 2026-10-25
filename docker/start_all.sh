#!/bin/bash
# 一键启动算法全栈 (锁协调 int.py + 6路视觉 vision_track.py + 6路C++ simple_explore)
# 用法: start_all.sh [route_id]    route_id 默认 0
# 前提: 官方机器的 Gazebo/PX4 SITL/mavros 已在运行, 容器用 --network host 启动
ROUTE=${1:-0}

echo "==================== robocup algo 启动 (route_id=$ROUTE) ===================="

python3 -c "import torch; print('[env] torch', torch.__version__, '| cuda:', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '(CPU模式, 检查 --gpus all)')" 2>/dev/null || echo "[env] torch 导入失败"

if ! timeout 5 rostopic list >/dev/null 2>&1; then
    echo "[warn] 连不上 ROS master —— 确认官方 Gazebo 已启动, 且容器加了 --network host"
fi

# 1) int.py (锁协调) + 6路 vision_track.py
roslaunch drone_ctrl simple.launch route_id:=$ROUTE &
P1=$!
sleep 6

# 2) 6路 simple_explore (C++ 巡逻/OFFBOARD/起飞), 最后启动
roslaunch drone_ctrl vision.launch route_id:=$ROUTE &
P2=$!

trap "echo '[start_all] 退出, 停止全部算法节点'; kill $P1 $P2 2>/dev/null" INT TERM
wait $P2
kill $P1 2>/dev/null
