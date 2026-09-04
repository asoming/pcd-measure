#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_command="${POINT_CLOUD_WORKBENCH_SYSTEM_PYTHON:-python3}"
environment_dir="${POINT_CLOUD_WORKBENCH_ROSBAG_VENV:-${project_dir}/.rosbag-venv}"

if ! command -v "${python_command}" >/dev/null 2>&1; then
  echo "没有找到 Python 3：${python_command}" >&2
  exit 2
fi

if [[ ! -x "${environment_dir}/bin/python" ]]; then
  if ! "${python_command}" -m venv "${environment_dir}"; then
    echo "无法创建 Python 虚拟环境。Ubuntu 请先安装：sudo apt install python3-venv" >&2
    exit 3
  fi
fi

"${environment_dir}/bin/python" -m pip install --disable-pip-version-check --upgrade pip
"${environment_dir}/bin/python" -m pip install --disable-pip-version-check \
  -r "${project_dir}/requirements-rosbag.txt"

echo "ROS bag 深度解析环境已就绪：${environment_dir}"
"${environment_dir}/bin/python" - <<'PY'
import rosbags
import yaml
print("rosbags / PyYAML 导入测试通过")
PY
