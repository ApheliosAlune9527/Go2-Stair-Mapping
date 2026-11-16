#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== [7] 离线回放数据包（无需插网线，空格暂停/方向键调速） ==="

set +u
source ~/unitree_ros2/setup.sh
set -u

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces></General></Domain></CycloneDDS>'

if [ $# -ge 1 ]; then
  BAG_DIR="$1"
else
  SEARCH_DIRS=()
  [ -d "$SCRIPT_DIR/bags" ] && SEARCH_DIRS+=("$SCRIPT_DIR/bags")
  [ -d "$HOME/unitree_ros2/bags" ] && SEARCH_DIRS+=("$HOME/unitree_ros2/bags")
  BAGS=()
  if [ "${#SEARCH_DIRS[@]}" -gt 0 ]; then
    mapfile -t BAGS < <(
      find -L "${SEARCH_DIRS[@]}" -maxdepth 2 -name "metadata.yaml" -printf "%T@ %h\n" 2>/dev/null \
        | sort -rn \
        | cut -d' ' -f2- \
        | awk '!seen[$0]++'
    )
  fi
  if [ "${#BAGS[@]}" -eq 0 ]; then
    echo "❌ 在 $SCRIPT_DIR/bags 与 ~/unitree_ros2/bags 下均未找到任何数据包。"
    exit 1
  fi
  echo "可用数据包（按修改时间从新到旧）："
  for i in "${!BAGS[@]}"; do
    sz=$(du -sh "${BAGS[$i]}" | awk '{print $1}')
    echo "  [$((i+1))] ${BAGS[$i]} ($sz)"
  done
  read -r -p "请输入编号（直接回车播放最新 [1]）: " pick
  pick="${pick:-1}"
  if ! [[ "$pick" =~ ^[0-9]+$ ]] || [ "$pick" -lt 1 ] || [ "$pick" -gt "${#BAGS[@]}" ]; then
    echo "❌ 无效编号: $pick"
    exit 1
  fi
  BAG_DIR="${BAGS[$((pick-1))]}"
fi

echo "正在回放：$BAG_DIR"
exec ros2 bag play "$BAG_DIR" --rate 1.0
