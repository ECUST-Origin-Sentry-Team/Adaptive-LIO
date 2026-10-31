#!/usr/bin/env bash
set -eo pipefail
# Run inside ros:jazzy-ros-base with /workspace mounted and dependencies installed.
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base /workspace/bench/log-deps build --base-paths /workspace/Adaptive-LIO/src/livox_ros_driver2 --build-base /workspace/build-deps --install-base /workspace/install-deps --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF
source /workspace/install-deps/setup.bash
for variant in baseline fixed; do
  if [[ "$variant" == baseline ]]; then source_root=/workspace/baseline; else source_root=/workspace/Adaptive-LIO; fi
  colcon --log-base /workspace/bench/log-$variant build --base-paths "$source_root/src/Adaptive-LIO" --build-base /workspace/build-$variant --install-base /workspace/install-$variant --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
 done
