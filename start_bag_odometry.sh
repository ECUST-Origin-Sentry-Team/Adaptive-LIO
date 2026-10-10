#!/usr/bin/env bash
# 启动 Adaptive-LIO 和 RViz；在另一个终端回放带 --clock 的 bag。
set -eo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$PROJECT_DIR/install/setup.bash" ]]; then
  echo "找不到 ROS Jazzy 或工作区 install/setup.bash，请先完成编译。" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$PROJECT_DIR/install/setup.bash"

lio_pid=""
rviz_pid=""
cleanup() {
  trap - EXIT INT TERM
  for pid in "$lio_pid" "$rviz_pid"; do
    if [[ -n "$pid" ]]; then
      kill -INT -- "-$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid ros2 launch adaptive_lio run.launch.py rviz:=false \
  adaptive_lio_cfg_dir:="$PROJECT_DIR/src/Adaptive-LIO/config/mapping_m.yaml" &
lio_pid=$!

setsid rviz2 -d "$PROJECT_DIR/bag_odometry.rviz" \
  --ros-args -p use_sim_time:=true &
rviz_pid=$!

echo "Adaptive-LIO 和 RViz 已启动，正在等待数据。在另一个终端执行："
echo "  $PROJECT_DIR/play_bag_163.sh"
echo "回放开始并完成 IMU 初始化后，日志会出现 init successful。"
echo "按 Ctrl+C 关闭本脚本启动的两个程序。"
wait -n "$lio_pid" "$rviz_pid"
