#!/usr/bin/env bash
set -euo pipefail

readonly required_packages=(
  git
  curl
  build-essential
  cmake
  python3-venv
  python3-pip
  python3-numpy
  qtbase5-dev
  libpcl-dev
  libvtk9-dev
  libvtk9-qt-dev
)

package_is_installed() {
  local package_name="$1"
  local package_status
  package_status="$(/usr/bin/dpkg-query -W -f='${db:Status-Abbrev}' \
    "${package_name}" 2>/dev/null || true)"
  [[ "${package_status}" == "ii " ]]
}

list_missing_packages() {
  local package_name
  for package_name in "${required_packages[@]}"; do
    if ! package_is_installed "${package_name}"; then
      printf '%s\n' "${package_name}"
    fi
  done
}

install_missing_packages() {
  if (( EUID != 0 )); then
    echo "系统依赖安装必须以 root 身份运行。" >&2
    exit 4
  fi

  mapfile -t missing_packages < <(list_missing_packages)
  if (( ${#missing_packages[@]} == 0 )); then
    echo "系统编译依赖已经完整，无需安装。"
    return
  fi

  echo "即将安装缺失的软件包：${missing_packages[*]}"
  /usr/bin/apt-get update
  DEBIAN_FRONTEND=noninteractive /usr/bin/apt-get install -y -- "${missing_packages[@]}"
}

case "${1:-}" in
  list)
    printf '%s\n' "${required_packages[@]}"
    ;;
  missing)
    list_missing_packages
    ;;
  check)
    mapfile -t missing_packages < <(list_missing_packages)
    if (( ${#missing_packages[@]} > 0 )); then
      echo "缺少系统软件包：${missing_packages[*]}" >&2
      exit 3
    fi
    echo "系统编译依赖已经完整。"
    ;;
  install)
    install_missing_packages
    ;;
  *)
    echo "用法：$0 {list|missing|check|install}" >&2
    exit 2
    ;;
esac
