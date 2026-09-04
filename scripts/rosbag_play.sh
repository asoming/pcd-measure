#!/usr/bin/env bash
set -euo pipefail

ros_version="${1:?ROS version is required}"
bag_path="${2:?bag path is required}"
playback_rate="${3:?playback rate is required}"
loop_enabled="${4:-0}"
clock_enabled="${5:-1}"

if [[ "${ros_version}" != "1" && "${ros_version}" != "2" ]]; then
  echo "ROS 版本必须是 1 或 2。" >&2
  exit 2
fi
if [[ ! -e "${bag_path}" ]]; then
  echo "bag 不存在：${bag_path}" >&2
  exit 2
fi
if ! [[ "${playback_rate}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
  ! awk -v rate="${playback_rate}" 'BEGIN { exit !(rate > 0) }'; then
  echo "回放倍率必须大于 0。" >&2
  exit 2
fi

source_if_present() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "${setup_file}"
    set -u
    return 0
  fi
  return 1
}

if [[ -n "${POINT_CLOUD_WORKBENCH_ROS_SETUP:-}" ]]; then
  source_if_present "${POINT_CLOUD_WORKBENCH_ROS_SETUP}" || {
    echo "指定的 ROS 环境脚本不存在：${POINT_CLOUD_WORKBENCH_ROS_SETUP}" >&2
    exit 3
  }
fi

if [[ "${ros_version}" == "2" ]]; then
  if ! command -v ros2 >/dev/null 2>&1; then
    preferred_distro="${ROS_DISTRO:-humble}"
    source_if_present "/opt/ros/${preferred_distro}/setup.bash" || true
  fi
  if ! command -v ros2 >/dev/null 2>&1; then
    for setup_file in /opt/ros/*/setup.bash; do
      [[ "${setup_file}" == */noetic/* ]] && continue
      source_if_present "${setup_file}" || continue
      command -v ros2 >/dev/null 2>&1 && break
    done
  fi
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "没有找到 ROS2 环境，无法执行 ros2 bag play。" >&2
    exit 3
  fi
  arguments=(bag play "${bag_path}" --rate "${playback_rate}" --disable-keyboard-controls)
  [[ "${loop_enabled}" == "1" ]] && arguments+=(--loop)
  [[ "${clock_enabled}" == "1" ]] && arguments+=(--clock 100)
  exec ros2 "${arguments[@]}"
fi

if ! command -v rosbag >/dev/null 2>&1; then
  source_if_present "/opt/ros/noetic/setup.bash" || true
fi
if ! command -v rosbag >/dev/null 2>&1; then
  echo "没有找到 ROS1 环境，无法执行 rosbag play。诊断仍可使用。" >&2
  exit 3
fi
arguments=(play "${bag_path}" --rate "${playback_rate}")
[[ "${loop_enabled}" == "1" ]] && arguments+=(--loop)
[[ "${clock_enabled}" == "1" ]] && arguments+=(--clock)
exec rosbag "${arguments[@]}"
