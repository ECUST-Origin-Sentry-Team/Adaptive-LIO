#!/usr/bin/env bash
set -euo pipefail
# Inside ros:jazzy-ros-base (Ubuntu24.04). Mount combined CA bundle as /run/host-ca.pem.
apt-get -o Acquire::https::CaInfo=/run/host-ca.pem update
DEBIAN_FRONTEND=noninteractive apt-get -o Acquire::https::CaInfo=/run/host-ca.pem install -y --no-install-recommends \
 libpcl-dev libceres-dev libgoogle-glog-dev libgflags-dev libyaml-cpp-dev libopencv-dev \
 ros-jazzy-pcl-conversions ros-jazzy-tf2-geometry-msgs ros-jazzy-message-filters \
 ros-jazzy-ament-cmake-auto ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
 ros-jazzy-rosbag2-storage-default-plugins python3-scipy python3-numpy python3-yaml
