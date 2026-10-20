#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "错误：未找到 /opt/ros/humble/setup.bash" >&2
  exit 1
fi
SDK_ROOT="${PXREA_SDK_ROOT:-/opt/apps/roboticsservice/SDK}"
SDK_INCLUDE_DIR="${PXREA_SDK_INCLUDE_DIR:-${SDK_ROOT}/include}"
SDK_LIB_DIR="${PXREA_SDK_LIB_DIR:-}"

if [[ -z "$SDK_LIB_DIR" ]]; then
  case "$(uname -m)" in
    aarch64|arm64)
      candidates=("${SDK_ROOT}/linux_aarch64/64" "${SDK_ROOT}/arm64" "${SDK_ROOT}/aarch64" "${SDK_ROOT}/lib/arm64" "${SDK_ROOT}/lib/aarch64")
      ;;
    x86_64|amd64)
      candidates=("${SDK_ROOT}/x64" "${SDK_ROOT}/linux/64")
      ;;
    *)
      echo "错误：未知平台，请显式设置 PXREA_SDK_LIB_DIR。" >&2
      exit 1
      ;;
  esac
  for candidate in "${candidates[@]}"; do
    if [[ -f "${candidate}/libPXREARobotSDK.so" ]]; then
      SDK_LIB_DIR="$candidate"
      break
    fi
  done
fi

if [[ ! -f "${SDK_INCLUDE_DIR}/PXREARobotSDK.h" ]] ||
   [[ ! -f "${SDK_LIB_DIR}/libPXREARobotSDK.so" ]]; then
  echo "错误：未找到 XRoboToolkit C++ SDK。请检查安装结果，必要时设置 PXREA_SDK_ROOT、PXREA_SDK_INCLUDE_DIR 和 PXREA_SDK_LIB_DIR。" >&2
  exit 1
fi

echo "XRoboToolkit SDK：头文件=${SDK_INCLUDE_DIR}，库=${SDK_LIB_DIR}"

# colcon forwards --cmake-args to every package. Export SDK paths instead so
# only qiling_kinematics_real consumes them and interface packages stay quiet.
export PXREA_SDK_ROOT="$SDK_ROOT"
export PXREA_SDK_INCLUDE_DIR="$SDK_INCLUDE_DIR"
export PXREA_SDK_LIB_DIR="$SDK_LIB_DIR"

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
