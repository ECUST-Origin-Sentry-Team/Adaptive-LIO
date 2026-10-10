#!/usr/bin/env bash
# 回放 163 雷达及其 IMU，发布 /clock 供 RViz 使用。
set -eo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${1:-$PROJECT_DIR/rosbag2_2026_07_07-16_37_53}"
cd "$PROJECT_DIR"

if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$PROJECT_DIR/install/setup.bash" ]]; then
  echo "找不到 ROS Jazzy 或工作区 install/setup.bash，请先完成编译。" >&2
  exit 1
fi
if [[ ! -f "$BAG_DIR/metadata.yaml" ]]; then
  echo "找不到 bag：$BAG_DIR" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$PROJECT_DIR/install/setup.bash"

echo "以 1 倍速回放 163 点云和 IMU；空格暂停/继续，Ctrl+C 停止。"
exec ros2 bag play "$BAG_DIR" \
  --clock \
  --rate 1.0 \
  --topics /livox/lidar_192_168_1_163 /livox/imu_192_168_1_163
