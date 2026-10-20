#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "错误：未找到 /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [[ ! -f /opt/apps/roboticsservice/SDK/include/PXREARobotSDK.h ]] ||
   [[ ! -f /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so ]]; then
  echo "错误：未找到 XRoboToolkit C++ SDK。请先安装 XRoboToolkit PC Service。" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

cd "$ROOT_DIR"
colcon build \
  --base-paths "$ROOT_DIR/src" \
  --symlink-install \
  --packages-up-to qiling_kinematics_real \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

echo "编译完成。新终端中请 source：$ROOT_DIR/install/setup.bash"
