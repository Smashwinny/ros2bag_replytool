#!/usr/bin/env bash
set -euo pipefail

: "${ESKF_REPLAY_CACHE_DATABASE:?}"
: "${ESKF_REPLAY_CACHE_PROCESS_LOG:?}"
: "${ESKF_COMPARE_TARGET_YAML:?}"
repo=/home/hulk/mow_mow_agent/mowmow
tool=/home/hulk/ros2bag/progress_player
run_dir=/home/hulk/ros2bag/progress_player/cache_runtime/domain${ROS_DOMAIN_ID}-pid$$
mkdir -p "${run_dir}"

set +u
source "${repo}/docs/eskf_fusion/debug/eskf_compare_env.sh"
set -u
children=()
cleanup() {
  trap - EXIT INT TERM HUP
  ((${#children[@]})) && kill -TERM "${children[@]}" 2>/dev/null || true
  wait "${children[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM HUP

python3 "${tool}/cached_replay_adapter.py" \
  --database "${ESKF_REPLAY_CACHE_DATABASE}" \
  --process-log "${ESKF_REPLAY_CACHE_PROCESS_LOG}" \
  --target-yaml "${ESKF_COMPARE_TARGET_YAML}" \
  --ros-args -p use_sim_time:=true >"${run_dir}/adapter.log" 2>&1 &
children+=("$!")

rviz_config="${run_dir}/cached-domain${ROS_DOMAIN_ID}.rviz"
cp "${repo}/docs/eskf_fusion/debug/eskf_old_new_compare.rviz" "${rviz_config}"
rviz2 -d "${rviz_config}" --ros-args -p use_sim_time:=true \
  >"${run_dir}/rviz.log" 2>&1 &
children+=("$!")

python3 "${tool}/cached_process_inspector.py" \
  --title "${ESKF_COMPARE_STATUS_TITLE:-ESKF Cached Process · Domain ${ROS_DOMAIN_ID}}" \
  --geometry "${ESKF_COMPARE_STATUS_GEOMETRY:-720x118+0+0}" \
  >"${run_dir}/status.log" 2>&1 &
children+=("$!")

wait
