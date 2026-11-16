#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-3d}"
if [ "$MODE" = "raw" ]; then
  CFG="$HOME/Go2/go2_slam_nav.rviz"
else
  CFG="$HOME/Go2/go2_stair_3d.rviz"
fi

echo "=== [6] 离线模式用 RTX 3050 GPU 启动 RViz2（无需插网线） ==="
echo "配置文件：$CFG"

set +u
source ~/unitree_ros2/setup.sh
set -u

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>'
exec env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia rviz2 -d "$CFG"
