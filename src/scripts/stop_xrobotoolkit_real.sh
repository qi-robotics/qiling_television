#!/usr/bin/env bash
set -Eeuo pipefail

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/qiling_xrobotoolkit"
SERVICE_PID_FILE="${RUNTIME_DIR}/robotics_service.pid"

if [[ ! -r "$SERVICE_PID_FILE" ]]; then
  echo "没有本脚本启动的 XRoboToolkit PC Service PID 文件，跳过。"
  exit 0
fi

read -r service_pid < "$SERVICE_PID_FILE"
if [[ ! "$service_pid" =~ ^[0-9]+$ ]] || ! kill -0 "$service_pid" 2>/dev/null; then
  rm -f "$SERVICE_PID_FILE"
  echo "服务进程已退出。"
  exit 0
fi

if [[ ! -r "/proc/${service_pid}/cmdline" ]] ||
   ! tr '\0' ' ' < "/proc/${service_pid}/cmdline" | grep -Fq '/RoboticsServiceProcess'; then
  echo "错误：PID ${service_pid} 与 RoboticsServiceProcess 不匹配，不终止。" >&2
  exit 1
fi

echo "停止 XRoboToolkit PC Service，PID=${service_pid}"
kill -TERM "$service_pid"
for ((attempt = 0; attempt < 10; ++attempt)); do
  if ! kill -0 "$service_pid" 2>/dev/null; then
    rm -f "$SERVICE_PID_FILE"
    exit 0
  fi
  sleep 0.5
done

echo "服务在 5 秒内未退出，请检查进程后手动处理：PID=${service_pid}" >&2
exit 1
