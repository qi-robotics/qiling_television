#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DDS_CONFIG="${QILING_DDS_CONFIG:-${ROOT_DIR}/config/cyclonedds_nx.xml}"

if [[ ! -r "$ROOT_DIR/install/setup.bash" ]]; then
  echo "错误：工程尚未编译，请先运行 src/scripts/build_xrtele.sh" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

export ROS_DOMAIN_ID=21
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
  if [[ ! -r "$DDS_CONFIG" ]]; then
    echo "错误：无法读取 NX DDS 配置：$DDS_CONFIG；请从 config/cyclonedds_nx.xml.example 创建配置，或设置 QILING_DDS_CONFIG/CYCLONEDDS_URI。" >&2
    exit 1
  fi
  DDS_CONFIG="$(realpath "$DDS_CONFIG")"
  export CYCLONEDDS_URI="file://$DDS_CONFIG"
elif [[ "$CYCLONEDDS_URI" == file://* ]] &&
     [[ ! -r "${CYCLONEDDS_URI#file://}" ]]; then
  echo "错误：无法读取 CYCLONEDDS_URI 指定的文件：${CYCLONEDDS_URI#file://}" >&2
  exit 1
fi

if [[ "$CYCLONEDDS_URI" == file://* ]] &&
   grep -q 'REPLACE_WITH_NX_ETHERNET_INTERFACE' "${CYCLONEDDS_URI#file://}"; then
  echo "错误：DDS 配置中的 NX 网卡名尚未填写。" >&2
  exit 1
fi

echo "真机模式：ROS_DOMAIN_ID=$ROS_DOMAIN_ID，RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "DDS 配置：$CYCLONEDDS_URI"
echo "复用已自启的 qi_topic_converter 状态输出；遥操命令通过专用桥发往 /lowcmd_arms。"
echo "确认机器人 SDK 已启动、双臂周围无障碍物并准备好急停。"

state_wait_sec="${QILING_STATE_WAIT_SEC:-30}"
if ! timeout "$state_wait_sec" ros2 topic echo /human_lower_state --once >/dev/null 2>&1; then
  echo "错误：${state_wait_sec} 秒内未收到 /human_lower_state。请检查已自启的 SDK、qi_topic_converter 和 DDS 配置。" >&2
  exit 2
fi

echo "已收到真机状态，即将自动执行：当前姿态 → 过渡点 → home → 遥操。"
echo "按 Ctrl+C 停止本次遥操；已自启的 SDK 和 qi_topic_converter 由原服务管理。"
exec ros2 launch qiling_kinematics_real xr_teleop_real.launch.py
