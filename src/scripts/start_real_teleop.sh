#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DDS_CONFIG="/home/coral/qi_deploy_igh_release/config/dds/cyclonedds_enp8s0.xml"

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
export CYCLONEDDS_URI="file://$DDS_CONFIG"

if [[ ! -r "$DDS_CONFIG" ]]; then
  echo "错误：无法读取 DDS 配置：$DDS_CONFIG" >&2
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
