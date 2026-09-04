#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
template_path="${project_dir}/点云测量工具.desktop"
system_dependencies="${project_dir}/scripts/system_dependencies.sh"

if [[ ! -f "${template_path}" ]]; then
  echo "找不到桌面图标模板：${template_path}" >&2
  exit 2
fi

if [[ ! -x "${system_dependencies}" ]]; then
  echo "找不到系统依赖检查器：${system_dependencies}" >&2
  exit 2
fi

missing_system_packages="$(${system_dependencies} missing)"
if [[ -n "${missing_system_packages}" ]]; then
  echo "检测到缺失的系统依赖："
  while IFS= read -r package_name; do
    printf '  %s\n' "${package_name}"
  done <<< "${missing_system_packages}"
  if [[ "${PCD_MEASURE_SKIP_SYSTEM_SETUP:-0}" == "1" ]]; then
    echo "已跳过系统依赖安装，无法继续编译。" >&2
    exit 3
  fi
  if (( EUID == 0 )); then
    "${system_dependencies}" install
  elif command -v sudo >/dev/null 2>&1; then
    echo "需要 sudo 权限安装固定的软件包列表。"
    sudo -- "${system_dependencies}" install
  else
    echo "没有找到 sudo。请以管理员身份运行：${system_dependencies} install" >&2
    exit 3
  fi
fi

if [[ ! -x "${project_dir}/build/bin/pcd_measure" ]]; then
  echo "首次安装，正在编译点云测量工具……"
  "${project_dir}/build.sh"
fi

if [[ "${PCD_MEASURE_SKIP_ROSBAG_SETUP:-0}" != "1" &&
  ! -x "${project_dir}/.rosbag-venv/bin/python" ]]; then
  echo "正在安装 ROS1/MCAP 离线解析支持……"
  if ! "${project_dir}/scripts/setup_rosbag_tools.sh"; then
    echo "警告：ROS bag 可选依赖安装失败；ROS2 SQLite 基础诊断仍可使用。" >&2
    echo "稍后可运行：./scripts/setup_rosbag_tools.sh" >&2
  fi
fi

if [[ "${PCD_MEASURE_SKIP_ODIN_MAP_SETUP:-0}" != "1" &&
  ! -x "${project_dir}/tools/map_to_ply" ]]; then
  echo "正在安装 MAPV0001 BIN 点云导入支持……"
  if ! "${project_dir}/scripts/setup_odin_map_tools.sh"; then
    echo "警告：Odin BIN 可选解码器安装失败；PCD、PLY 和 OLX 仍可使用。" >&2
    echo "稍后可运行：./scripts/setup_odin_map_tools.sh" >&2
  fi
fi

desktop_dir="${PCD_MEASURE_DESKTOP_DIR:-}"
if [[ -z "${desktop_dir}" ]] && command -v xdg-user-dir >/dev/null 2>&1; then
  desktop_dir="$(xdg-user-dir DESKTOP)"
fi
if [[ -z "${desktop_dir}" ]]; then
  desktop_dir="${HOME:?}/Desktop"
fi

applications_dir="${PCD_MEASURE_APPLICATIONS_DIR:-${XDG_DATA_HOME:-${HOME:?}/.local/share}/applications}"
desktop_launcher="${desktop_dir}/点云测量工具.desktop"
menu_launcher="${applications_dir}/pcd-measure.desktop"
temporary_launcher="$(mktemp)"
trap 'rm -f -- "${temporary_launcher}"' EXIT

escaped_project_dir="${project_dir//\\/\\\\}"
escaped_project_dir="${escaped_project_dir//&/\\&}"
escaped_project_dir="${escaped_project_dir//|/\\|}"
sed "s|@PROJECT_DIR@|${escaped_project_dir}|g" \
  "${template_path}" > "${temporary_launcher}"

mkdir -p -- "${desktop_dir}" "${applications_dir}"
install -m 755 "${temporary_launcher}" "${desktop_launcher}"
install -m 755 "${temporary_launcher}" "${menu_launcher}"

if command -v gio >/dev/null 2>&1; then
  gio set "${desktop_launcher}" metadata::trusted true >/dev/null 2>&1 || true
fi
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "${applications_dir}" >/dev/null 2>&1 || true
fi

echo "安装完成：${desktop_launcher}"
echo "现在可以双击桌面的“点云测量工具”启动。"
echo "如果图标首次显示为普通文本文件，请右键它并选择“允许启动”。"
