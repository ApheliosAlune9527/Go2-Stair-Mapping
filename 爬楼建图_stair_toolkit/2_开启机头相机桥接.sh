#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== [2] 启动 Go2 机头前视相机桥接节点 ==="
if ! ip -brief addr show enp4s0 | grep -q "192\.168\.123\."; then
  echo "❌ 有线网卡 enp4s0 未连接到 Go2，请先插好网线并运行：$SCRIPT_DIR/1_连接Go2并检查状态.sh"
  exit 1
fi

set +u
source ~/unitree_ros2/setup.sh
set -u

echo "保持此终端运行即可持续发布 /go2/front_camera/image_raw/compressed（录完包按 Ctrl+C 关闭）"
exec python3 ~/Go2/go2_front_cam_node.py
