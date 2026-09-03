#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
architecture="$(uname -m)"

case "${architecture}" in
  x86_64|amd64)
    asset_name="map_to_ply_amd64"
    expected_sha256="4f13acdfb28555d52509666820d91094f8c2f3a4fead3c47018678d5a47539ab"
    ;;
  aarch64|arm64)
    asset_name="map_to_ply_arm64"
    expected_sha256="5ef5db0a07ad825d6d65fb985ce43a9be7bb3c9c200f6e3d2c30c69e3b34250d"
    ;;
  *)
    echo "当前 CPU 架构不受官方 map_to_ply 支持：${architecture}" >&2
    exit 2
    ;;
esac

if ! command -v curl >/dev/null 2>&1; then
  echo "缺少 curl，请先安装：sudo apt install curl" >&2
  exit 3
fi
if ! command -v sha256sum >/dev/null 2>&1; then
  echo "缺少 sha256sum，无法校验下载文件。" >&2
  exit 3
fi

target_path="${project_dir}/tools/map_to_ply"
download_path="$(mktemp "${target_path}.download.XXXXXX")"
trap 'rm -f -- "${download_path}"' EXIT

download_url="https://raw.githubusercontent.com/ManifoldTechLtd/wiki/master/docs/odin_series/odin1/assets/code/${asset_name}"
echo "正在从 Manifold Tech 官方仓库下载 ${asset_name}……"
curl -fL --retry 3 --connect-timeout 15 "${download_url}" -o "${download_path}"

actual_sha256="$(sha256sum "${download_path}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${expected_sha256}" ]]; then
  echo "安全校验失败，下载文件未安装。" >&2
  echo "期望：${expected_sha256}" >&2
  echo "实际：${actual_sha256}" >&2
  exit 4
fi

install -m 755 "${download_path}" "${target_path}"
echo "Odin MAPV0001 解码器已安装：${target_path}"
echo "SHA-256：${actual_sha256}"
