#!/usr/bin/env bash
set -Eeo pipefail

service_root="${XROBO_SERVICE_ROOT:-/opt/apps/roboticsservice}"
launcher="${service_root}/runService.sh"
if [[ ! -x "$launcher" ]]; then
  echo "错误：XRoboToolkit 启动入口不存在或不可执行：$launcher" >&2
  exit 1
fi
if [[ ! -x "${service_root}/RoboticsServiceProcess" ]]; then
  echo "错误：XRoboToolkit 主程序不存在或不可执行：${service_root}/RoboticsServiceProcess" >&2
  exit 1
fi
service_binary="$(realpath "${service_root}/RoboticsServiceProcess")"
for proc_exe in /proc/[0-9]*/exe; do
  running_binary="$(readlink -f "$proc_exe" 2>/dev/null || true)"
  if [[ "$running_binary" == "$service_binary" ]]; then
    existing_pid="${proc_exe#/proc/}"
    existing_pid="${existing_pid%/exe}"
    echo "错误：RoboticsServiceProcess 已运行，PID=$existing_pid；请勿启动第二份 PC Service。" >&2
    exit 1
  fi
done

# The vendor launcher backgrounds RoboticsServiceProcess. Source it so the
# spawned process stays our child; wait propagates its exit to systemd.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
source "$launcher"
service_pid=$!
if [[ -z "$service_pid" ]]; then
  echo "错误：XRoboToolkit 启动入口未创建后台服务进程。" >&2
  exit 1
fi
echo "XRoboToolkit PC Service 已启动，PID=$service_pid"
wait "$service_pid"
