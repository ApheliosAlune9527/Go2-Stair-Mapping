#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== [4] 开始录制 Go2 爬楼梯数据包 ==="
if ! ip -brief addr show enp4s0 | grep -q "192\.168\.123\."; then
  echo "❌ 有线网卡 enp4s0 未连接到 Go2，请先运行：$SCRIPT_DIR/1_连接Go2并检查状态.sh"
  exit 1
fi

set +u
source ~/unitree_ros2/setup.sh
set -u

cam_pub=$(timeout 4 ros2 topic info --no-daemon /go2/front_camera/image_raw/compressed 2>/dev/null | awk '/Publisher count:/{print $3}' || echo "0")
lidar_pub=$(timeout 4 ros2 topic info --no-daemon /utlidar/cloud_deskewed 2>/dev/null | awk '/Publisher count:/{print $3}' || echo "0")
uslam_pub=$(timeout 4 ros2 topic info --no-daemon /uslam/frontend/cloud_world_ds 2>/dev/null | awk '/Publisher count:/{print $3}' || echo "0")

echo "当前发布者状态："
echo "  /go2/front_camera/image_raw/compressed : ${cam_pub:-0}"
echo "  /utlidar/cloud_deskewed                : ${lidar_pub:-0}"
echo "  /uslam/frontend/cloud_world_ds         : ${uslam_pub:-0} (未开脚本3时可为0，不影响后续3D重建)"

if [ "${lidar_pub:-0}" -lt 1 ] 2>/dev/null; then
  echo "❌ 未检测到 /utlidar/cloud_deskewed 雷达数据，请确认 Go2 已完全启动后重试。"
  exit 1
fi

if [ "${cam_pub:-0}" -lt 1 ] 2>/dev/null; then
  echo "⚠️  警告：未检测到机头相机发布者！请先在另一个终端运行：$SCRIPT_DIR/2_开启机头相机桥接.sh"
  read -r -p "是否仍继续只录雷达和状态？(y/N): " ans
  if [[ ! "$ans" =~ ^[Yy]$ ]]; then
    exit 1
  fi
fi

mkdir -p ~/unitree_ros2/bags
OUT_DIR="$HOME/unitree_ros2/bags/stair_demo_$(date +%Y%m%d_%H%M%S)"
echo
echo "数据包将保存到：$OUT_DIR"
echo "准备就绪后即可遥控 Go2 爬楼梯；爬完后在本终端按 Ctrl+C 结束并保存！"
echo

set +e
ros2 bag record -o "$OUT_DIR" \
  /go2/front_camera/image_raw/compressed \
  /utlidar/cloud_deskewed \
  /utlidar/robot_odom \
  /utlidar/height_map_array \
  /uslam/frontend/odom \
  /uslam/frontend/cloud_world_ds \
  /uslam/cloud_map \
  /uslam/server_log \
  /sportmodestate \
  /lowstate

echo
echo "✅ 录包结束：$OUT_DIR"
