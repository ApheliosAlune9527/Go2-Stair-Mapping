#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== [1] 连接 Go2 有线网卡并检查状态 ==="
sudo nmcli connection up Unitree-Go2 ifname enp4s0
echo
echo "网卡状态："
ip -brief addr show enp4s0

if ! ip -brief addr show enp4s0 | grep -q "192\.168\.123\."; then
  echo "❌ enp4s0 未获取到 192.168.123.x IP，请检查网线是否插紧、Go2 是否已开机。"
  exit 1
fi

# 清掉可能残留的旧 ros2-daemon，避免接口切换后 CLI 卡住
pkill -f "ros2-daemon" 2>/dev/null || true

set +u
source ~/unitree_ros2/setup.sh
set -u

echo
echo "正在检查机器狗在线话题（约需 2~3 秒）..."
for topic in /utlidar/cloud_deskewed /utlidar/robot_odom /sportmodestate /api/videohub/response; do
  pub_cnt=$(timeout 4 ros2 topic info --no-daemon "$topic" 2>/dev/null | awk '/Publisher count:/{print $3}' || echo "0")
  pub_cnt="${pub_cnt:-0}"
  if [ "$pub_cnt" -ge 1 ] 2>/dev/null; then
    echo "  ✅ $topic (Publisher count: $pub_cnt)"
  else
    echo "  ⚠️  $topic (Publisher count: $pub_cnt) -- 若刚开机请等几秒重试"
  fi
done

echo
echo "操作提醒："
echo "  1. 下一步请新开终端运行：$SCRIPT_DIR/2_开启机头相机桥接.sh"
echo "  2. 遥控爬楼时尽量只推摇杆控制走动/转向，避免按实体按键触发 USLAM 急停。"
