#!/usr/bin/env bash
# ROS process_prefix: keep RPC logging off the launching terminal's pipe.
set -euo pipefail
if [[ $# -eq 0 ]]; then
  exit 2
fi
umask 077
task_rpc_log_dir="${FRANKA_RPC_LOG_DIR:-$HOME/.ros/log/franka_rpc}"
mkdir -p -- "$task_rpc_log_dir"
task_rpc_log_file="$task_rpc_log_dir/rpc_$(date +%Y%m%d_%H%M%S)_$$.log"
exec "$@" >>"$task_rpc_log_file" 2>&1
