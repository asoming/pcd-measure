#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_jobs="${POINT_CLOUD_WORKBENCH_BUILD_JOBS:-}"

if [[ -z "${build_jobs}" ]]; then
  processor_count="$(nproc)"
  if (( processor_count > 2 )); then
    build_jobs=2
  else
    build_jobs="${processor_count}"
  fi
fi
if [[ ! "${build_jobs}" =~ ^[1-9][0-9]*$ ]]; then
  echo "POINT_CLOUD_WORKBENCH_BUILD_JOBS 必须是正整数。" >&2
  exit 2
fi

cmake -S "${project_dir}" -B "${project_dir}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${project_dir}/build" -j"${build_jobs}"

echo "构建完成：${project_dir}/build/bin/point_cloud_workbench"
