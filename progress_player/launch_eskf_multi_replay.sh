#!/usr/bin/env bash
set -euo pipefail

player=/home/hulk/ros2bag/rosbag_progress_player.py
profile=/home/hulk/ros2bag/progress_player/eskf_compare.yaml
base_domain=${ESKF_MULTI_BASE_DOMAIN:-180}

default_bags=(
  /home/hulk/ros2bag/rosbag2_2026_08_07-15_56_44
  /home/hulk/ros2bag/rosbag2_2026_08_07-16_40_39
  /home/hulk/ros2bag/bag/rosbag2_2026_08_13-15_08_48
)
if (($#)); then
  bags=("$@")
else
  bags=("${default_bags[@]}")
fi

if ((${#bags[@]} < 1 || ${#bags[@]} > 3)); then
  echo "ERROR: provide between one and three rosbag directories." >&2
  exit 2
fi
for bag in "${bags[@]}"; do
  if [[ ! -f "${bag}/metadata.yaml" ]]; then
    echo "ERROR: missing ${bag}/metadata.yaml" >&2
    exit 2
  fi
done
for command in python3 pgrep ps wmctrl xdotool xrandr; do
  if ! command -v "${command}" >/dev/null; then
    echo "ERROR: missing desktop command: ${command}" >&2
    exit 2
  fi
done
if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is not set; run this script in the graphical desktop terminal." >&2
  exit 2
fi

set +u
source /home/hulk/mow_mow_agent/mowmow/docs/eskf_fusion/debug/eskf_compare_env.sh
set -u
screen=$(xrandr --current | awk '
  / connected/ {
    for (i=1; i<=NF; ++i) {
      if ($i ~ /^[0-9]+x[0-9]+\+[0-9]+\+[0-9]+$/) {
        split($i, offset, "+"); split(offset[1], size, "x");
        area=size[1]*size[2];
        if (area > best) {best=area; w=size[1]; h=size[2]; x=offset[2]; y=offset[3]}
      }
    }
  }
  END {if (best) print w, h, x, y}')
read -r screen_w screen_h screen_x screen_y <<<"${screen:-1920 1080 0 0}"
columns=${#bags[@]}
column_w=$((screen_w / columns))
player_h=310
rviz_h=$((screen_h - player_h))
status_h=76

players=()
descendants() {
  local parent=$1 child
  while read -r child; do
    [[ -z "${child}" ]] && continue
    echo "${child}"
    descendants "${child}"
  done < <(pgrep -P "${parent}" 2>/dev/null || true)
}
cleanup() {
  trap - EXIT INT TERM HUP
  if ((${#players[@]})); then
    kill -TERM "${players[@]}" 2>/dev/null || true
    wait "${players[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

for index in "${!bags[@]}"; do
  bag=${bags[$index]}
  label=$(basename "${bag}")
  domain=$((base_domain + index))
  x=$((screen_x + column_w * index))
  width=${column_w}
  if ((index == columns - 1)); then width=$((screen_w - x)); fi
  progress_title="[${index}:${label}] ESKF Progress · Domain ${domain}"
  rviz_title="[${index}:${label}] RViz2 · Domain ${domain}"
  status_title="[${index}:${label}] Sensor Trust · Domain ${domain}"

  ESKF_COMPARE_BAG="${bag}" ESKF_COMPARE_DOMAIN_ID="${domain}" \
    ESKF_COMPARE_RVIZ_COMPACT=1 \
    ESKF_COMPARE_STATUS_TITLE="${status_title}" \
    ESKF_COMPARE_STATUS_GEOMETRY="${width}x${status_h}+${x}+$((screen_y + rviz_h - status_h))" \
    ROS_DOMAIN_ID="${domain}" ROS_LOCALHOST_ONLY=1 \
    python3 "${player}" "${bag}" --profile "${profile}" \
      --title "${progress_title}" --geometry "${width}x${player_h}+${x}+$((screen_y + rviz_h))" &
  players+=("$!")

  player_pid=${players[$index]}
  rviz_pid=""
  rviz_id=""
  for _ in {1..300}; do
    while read -r candidate_pid; do
      if [[ "$(ps -p "${candidate_pid}" -o comm= 2>/dev/null)" == "rviz2" ]]; then
        rviz_pid=${candidate_pid}
        rviz_id=$(xdotool search --onlyvisible --pid "${rviz_pid}" 2>/dev/null | head -1 || true)
        [[ -n "${rviz_id}" ]] && break
      fi
    done < <(descendants "${player_pid}")
    [[ -n "${rviz_id}" ]] && break
    sleep 0.1
  done

  if [[ -n "${rviz_id}" ]]; then
    # RViz may replace its title once after loading the config; wait for that
    # initialization before assigning the stable bag/domain title.
    sleep 2
    xdotool set_window --name "${rviz_title}" "${rviz_id}" 2>/dev/null || true
    rviz_hex=$(printf '0x%x' "${rviz_id}")
    wmctrl -i -r "${rviz_hex}" -e "0,${x},${screen_y},${width},${rviz_h}" \
      2>/dev/null || true
  else
    echo "WARN: RViz2 window was not found for ${label}." >&2
  fi
  progress_hex=$({ wmctrl -l 2>/dev/null || true; } | \
    awk -v title="${progress_title}" 'index($0,title) {print $1; exit}')
  if [[ -n "${progress_hex}" ]]; then
    wmctrl -i -r "${progress_hex}" \
      -e "0,${x},$((screen_y + rviz_h)),${width},${player_h}" 2>/dev/null || true
  else
    echo "WARN: progress window was not found for ${label}." >&2
  fi
done

echo "Started ${#bags[@]} isolated ESKF replay desktops on Domains ${base_domain}..$((base_domain + columns - 1))."
echo "Each column is one bag: RViz2, floating sensor status and matching progress bar."
echo "Close the progress windows or press Ctrl-C here to stop every replay stack."
wait
