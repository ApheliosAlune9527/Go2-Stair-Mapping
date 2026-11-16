#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ACTION="${1:-start}"
if [ "$ACTION" != "start" ] && [ "$ACTION" != "stop" ]; then
  echo "用法：$0 [start|stop]  （默认 start）"
  exit 1
fi

if ! ip -brief addr show enp4s0 | grep -q "192\.168\.123\."; then
  echo "❌ 有线网卡 enp4s0 未连接到 Go2，请先运行：$SCRIPT_DIR/1_连接Go2并检查状态.sh"
  exit 1
fi

set +u
source ~/unitree_ros2/setup.sh
set -u

echo "=== [3] 发送 USLAM 指令: mapping/$ACTION ==="
ros2 topic pub --once --keep-alive 1 /uslam/client_command std_msgs/msg/String "{data: 'mapping/$ACTION'}"

if [ "$ACTION" = "start" ]; then
  sleep 2
  for topic in /uslam/frontend/odom /uslam/frontend/cloud_world_ds; do
    pub_cnt=$(timeout 4 ros2 topic info --no-daemon "$topic" 2>/dev/null | awk '/Publisher count:/{print $3}' || echo "0")
    echo "  $topic -> Publisher count: ${pub_cnt:-0}"
  done
  echo "提示：若需结束 USLAM 建图，可再运行：$0 stop"
fi
