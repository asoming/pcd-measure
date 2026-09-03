#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_workspace="${1:?ROS workspace is required}"
shift

if [[ ! -f "${ros_workspace}/install/setup.bash" ]]; then
  echo "找不到 ROS2 工作空间环境：${ros_workspace}/install/setup.bash" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "${ros_workspace}/install/setup.bash"
set -u
exec /usr/bin/python3 "${project_dir}/tools/olx_record.py" "$@"
