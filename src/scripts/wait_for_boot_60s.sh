#!/usr/bin/env bash
set -Eeuo pipefail

# Wait for 60 seconds of actual system uptime. Starting/restarting the unit
# later in the same boot does not add another unnecessary 60-second delay.
read -r uptime_seconds _ < /proc/uptime
elapsed_seconds="${uptime_seconds%%.*}"
remaining_seconds=$((60 - elapsed_seconds))
if ((remaining_seconds > 0)); then
  echo "NX 开机已 ${elapsed_seconds}s；等待 ${remaining_seconds}s 后启动真机服务。"
  sleep "$remaining_seconds"
fi
