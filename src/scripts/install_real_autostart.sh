#!/usr/bin/env bash
set -Eeuo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
scripts_dir="$root_dir/src/scripts"
unit_dir=/etc/systemd/system

if ((EUID != 0)); then
  echo "请用 sudo 运行：sudo $scripts_dir/install_real_autostart.sh" >&2
  exit 1
fi
if [[ "$root_dir" != /home/user/liujun/qiling_television ]]; then
  echo "错误：unit 使用固定 NX 路径 /home/user/liujun/qiling_television，当前路径为 $root_dir" >&2
  exit 1
fi
for required in \
  "$root_dir/install/setup.bash" \
  "$root_dir/config/cyclonedds_nx.xml" \
  "$scripts_dir/wait_for_boot_60s.sh" \
  "$scripts_dir/run_xrobotoolkit_supervised.sh" \
  "$scripts_dir/start_real_teleop.sh" \
  /opt/ros/humble/setup.bash \
  /opt/apps/roboticsservice/runService.sh; do
  if [[ ! -r "$required" ]]; then
    echo "错误：自启所需文件不存在或不可读：$required" >&2
    exit 1
  fi
done
for executable in \
  "$scripts_dir/wait_for_boot_60s.sh" \
  "$scripts_dir/run_xrobotoolkit_supervised.sh" \
  "$scripts_dir/start_real_teleop.sh" \
  /opt/apps/roboticsservice/runService.sh; do
  if [[ ! -x "$executable" ]]; then
    echo "错误：自启所需文件不可执行：$executable" >&2
    exit 1
  fi
done
if grep -q 'REPLACE_WITH_NX_ETHERNET_INTERFACE' "$root_dir/config/cyclonedds_nx.xml"; then
  echo "错误：NX DDS 文件中的网卡名尚未填写。" >&2
  exit 1
fi

install -m 0644 "$scripts_dir/qiling-xrobotoolkit.service" "$unit_dir/qiling-xrobotoolkit.service"
install -m 0644 "$scripts_dir/qiling-real-teleop.service" "$unit_dir/qiling-real-teleop.service"
systemctl daemon-reload
systemctl enable qiling-xrobotoolkit.service qiling-real-teleop.service
echo "已安装并启用开机自启；本次未启动服务。下次开机满 60 秒后开始运行。"
