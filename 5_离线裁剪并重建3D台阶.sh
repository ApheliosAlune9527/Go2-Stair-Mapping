#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== [5] 离线裁剪上楼梯片段并重建 3D 台阶点云（无需接狗） ==="
set +u
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
set -u

exec python3 "$SCRIPT_DIR/process_stair_bag.py" "$@"
