#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "错误：未找到 ROS 2 Humble。请先在 Ubuntu 22.04 上安装 ROS 2 Humble。" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  adb \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libsimde-dev \
  pkg-config \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-yaml \
  ros-humble-ament-cmake \
  ros-humble-ament-cmake-gtest \
  ros-humble-pinocchio \
  ros-humble-proxsuite \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-robot-state-publisher \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-generator-dds-idl \
  ros-humble-urdf

echo "真机遥操编译工具、ROS 2 接口依赖、Pinocchio、ProxSuite、CycloneDDS 和 ADB 已安装。"
echo "下一步：安装 XRoboToolkit PC Service，再运行 src/scripts/build_xrtele.sh。"
