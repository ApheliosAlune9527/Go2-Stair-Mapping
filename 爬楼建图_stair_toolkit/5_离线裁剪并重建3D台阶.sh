#!/usr/bin/env bash
set -euo pipefail

echo "=== [5] 离线裁剪上楼梯片段并重建 3D 台阶点云（无需接狗） ==="
set +u
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
set -u

exec python3 ~/Go2/process_stair_bag.py "$@"
