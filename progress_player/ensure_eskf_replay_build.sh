#!/usr/bin/env bash
set -euo pipefail

repo=/home/hulk/mow_mow_agent/mowmow
tool=/home/hulk/ros2bag/progress_player/build_artifact_guard.py
state=/home/hulk/ros2bag/progress_player/build_state/eskf_compare.json
install=${repo}/tmp/eskf_compare_install/eskf_location
panel_install=${repo}/tmp/eskf_compare_install/eskf_sensor_trust_panel

exec python3 "${tool}" --state "${state}" \
  --source "${repo}/mowmow_location/eskf_location/CMakeLists.txt" \
  --source "${repo}/mowmow_location/eskf_location/include" \
  --source "${repo}/mowmow_location/eskf_location/src" \
  --source "${repo}/mowmow_location/eskf_location/config/eskf.yaml" \
  --source "${repo}/mowmow_manager/mowing_bt/config/work_params.yaml" \
  --source "${repo}/docs/eskf_fusion/debug/prepare_eskf_compare.sh" \
  --source "${repo}/docs/eskf_fusion/debug/generate_eskf_replay_config.py" \
  --source "${repo}/docs/eskf_fusion/debug/generate_eskf_replay_runtime_params.py" \
  --source "${repo}/docs/eskf_fusion/debug/check_eskf_replay_parity.py" \
  --source "${repo}/docs/eskf_fusion/debug/eskf_sensor_trust_panel" \
  --source "${repo}/docs/eskf_fusion/evidence/imu_calibration_20260803_identity_rotation_experiment.yaml" \
  --artifact "${install}/lib/eskf_location/eskf_location_node" \
  --artifact "${install}/share/eskf_location/config/eskf.yaml" \
  --artifact "${repo}/tmp/eskf_compare_generated/eskf_runtime_params.yaml" \
  --artifact "${panel_install}/lib/libeskf_sensor_trust_panel.so" \
  -- bash "${repo}/docs/eskf_fusion/debug/prepare_eskf_compare.sh"
