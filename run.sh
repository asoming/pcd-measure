#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
binary_path="${project_dir}/build/bin/pcd_measure"

export PCD_MEASURE_PROJECT_DIR="${project_dir}"
if [[ -x "${project_dir}/.rosbag-venv/bin/python" ]]; then
  export PCD_MEASURE_ROSBAG_PYTHON="${project_dir}/.rosbag-venv/bin/python"
fi
if [[ -x "${project_dir}/tools/map_to_ply" ]]; then
  export PCD_MEASURE_MAP_TO_PLY="${project_dir}/tools/map_to_ply"
fi

if [[ ! -x "${binary_path}" ]]; then
  "${project_dir}/build.sh"
fi

exec "${binary_path}" "$@"
