#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
template_path="${project_dir}/PCD 点云测量工具.desktop"

if [[ ! -f "${template_path}" ]]; then
  echo "找不到桌面图标模板：${template_path}" >&2
  exit 2
fi

if [[ ! -x "${project_dir}/build/bin/pcd_measure" ]]; then
  echo "首次安装，正在编译 PCD 点云测量工具……"
  "${project_dir}/build.sh"
fi

desktop_dir="${PCD_MEASURE_DESKTOP_DIR:-}"
if [[ -z "${desktop_dir}" ]] && command -v xdg-user-dir >/dev/null 2>&1; then
  desktop_dir="$(xdg-user-dir DESKTOP)"
fi
if [[ -z "${desktop_dir}" ]]; then
  desktop_dir="${HOME:?}/Desktop"
fi

applications_dir="${XDG_DATA_HOME:-${HOME:?}/.local/share}/applications"
desktop_launcher="${desktop_dir}/PCD 点云测量工具.desktop"
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
echo "现在可以双击桌面的“PCD 点云测量工具”启动。"
echo "如果图标首次显示为普通文本文件，请右键它并选择“允许启动”。"
