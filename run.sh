#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
binary_path="${project_dir}/build/bin/pcd_measure"

if [[ ! -x "${binary_path}" ]]; then
  cmake -S "${project_dir}" -B "${project_dir}/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "${project_dir}/build" -j"$(nproc)"
fi

exec "${binary_path}" "$@"
