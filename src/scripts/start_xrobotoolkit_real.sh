#!/usr/bin/env bash
set -Eeuo pipefail

SERVICE_ROOT="${XROBO_SERVICE_ROOT:-/opt/apps/roboticsservice}"
SERVICE_LAUNCHER="${SERVICE_ROOT}/runService.sh"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/qiling_xrobotoolkit"
SERVICE_PID_FILE="${RUNTIME_DIR}/robotics_service.pid"
SERVICE_LOG="${RUNTIME_DIR}/robotics_service.log"

if [[ ! -x "$SERVICE_LAUNCHER" ]]; then
  echo "错误：找不到或无法执行 XRoboToolkit 服务入口：${SERVICE_LAUNCHER}" >&2
  exit 1
fi

mkdir -p "$RUNTIME_DIR"
umask 077

service_pid="$(pgrep -n -f '/RoboticsServiceProcess([[:space:]]|$)' || true)"
if [[ -n "$service_pid" ]]; then
  echo "RoboticsServiceProcess 已运行，PID=${service_pid}；本脚本不接管已有进程。"
  exit 0
fi
rm -f "$SERVICE_PID_FILE"

# Use the vendor launcher so its runtime library and Qt settings stay intact.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
cd "$SERVICE_ROOT"
nohup "$SERVICE_LAUNCHER" >>"$SERVICE_LOG" 2>&1 </dev/null &

for ((attempt = 0; attempt < 20; ++attempt)); do
  service_pid="$(pgrep -n -f '/RoboticsServiceProcess([[:space:]]|$)' || true)"
  if [[ -n "$service_pid" ]]; then
    echo "$service_pid" > "$SERVICE_PID_FILE"
    echo "XRoboToolkit PC Service 已启动，PID=${service_pid}"
    echo "日志：${SERVICE_LOG}"
    exit 0
  fi
  sleep 0.5
done

echo "错误：未发现 RoboticsServiceProcess，请检查日志：${SERVICE_LOG}" >&2
exit 1
