#!/usr/bin/env bash
set -euo pipefail

python_command="${1:?Python executable is required}"
shift

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

if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_if_present "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source_if_present /opt/ros/humble/setup.bash
else
  for setup_file in /opt/ros/*/setup.bash; do
    [[ "${setup_file}" == */noetic/* ]] && continue
    source_if_present "${setup_file}" && break
  done
fi

if [[ -n "${PCD_MEASURE_ROS_SETUP:-}" ]]; then
  source_if_present "${PCD_MEASURE_ROS_SETUP}" || {
    echo "指定的 ROS 环境脚本不存在：${PCD_MEASURE_ROS_SETUP}" >&2
    exit 3
  }
fi

exec "${python_command}" "$@"
