#!/usr/bin/env bash
set -euo pipefail

project_dir="${1:?project directory is required}"
analyzer="${2:?analyzer path is required}"
application="${3:?application path is required}"
fixture="${project_dir}/tests/fixtures/xyz_no_color_ascii.pcd"
test_dir="$(mktemp -d /tmp/point-cloud-workbench-package-test.XXXXXX)"

cleanup() {
  case "${test_dir}" in
    /tmp/point-cloud-workbench-package-test.*) rm -rf -- "${test_dir}" ;;
  esac
}
trap cleanup EXIT

[[ "$(basename -- "${analyzer}")" == "point_cloud_analyze" ]]
[[ "$(basename -- "${application}")" == "point_cloud_workbench" ]]

bash -n "${project_dir}/build.sh"
bash -n "${project_dir}/run.sh"
bash -n "${project_dir}/install_desktop.sh"
bash -n "${project_dir}/scripts/verify_features.sh"
bash -n "${project_dir}/scripts/rosbag_play.sh"
bash -n "${project_dir}/scripts/rosbag_diagnose.sh"
bash -n "${project_dir}/scripts/setup_rosbag_tools.sh"
bash -n "${project_dir}/scripts/setup_odin_map_tools.sh"
bash -n "${project_dir}/scripts/system_dependencies.sh"
bash -n "${project_dir}/tests/rosbag_playback_test.sh"
test -x "${project_dir}/scripts/system_dependencies.sh"
"${project_dir}/tools/rosbag_diagnose.py" --help >/dev/null

expected_system_packages=$'git\ncurl\nbuild-essential\ncmake\npython3-venv\npython3-pip\npython3-numpy\nqtbase5-dev\nlibpcl-dev\nlibvtk9-dev\nlibvtk9-qt-dev'
actual_system_packages="$("${project_dir}/scripts/system_dependencies.sh" list)"
if [[ "${actual_system_packages}" != "${expected_system_packages}" ]]; then
  echo "system dependency allowlist changed unexpectedly" >&2
  exit 1
fi
"${project_dir}/scripts/system_dependencies.sh" check >/dev/null
if (( EUID != 0 )); then
  if "${project_dir}/scripts/system_dependencies.sh" install \
    >"${test_dir}/unprivileged-install.out" 2>"${test_dir}/unprivileged-install.err"; then
    echo "system dependency installer unexpectedly ran without root" >&2
    exit 1
  fi
  grep -Fq "root" "${test_dir}/unprivileged-install.err"
fi
if "${project_dir}/scripts/system_dependencies.sh" unsupported \
  >"${test_dir}/invalid-system-command.out" 2>"${test_dir}/invalid-system-command.err"; then
  echo "system dependency helper unexpectedly accepted an invalid command" >&2
  exit 1
fi

if "${analyzer}" >"${test_dir}/missing-argument.out" 2>"${test_dir}/missing-argument.err"; then
  echo "point_cloud_analyze unexpectedly accepted an empty argument list" >&2
  exit 1
fi
if "${analyzer}" --max-display-points 0 "${fixture}" \
  >"${test_dir}/invalid-limit.out" 2>"${test_dir}/invalid-limit.err"; then
  echo "point_cloud_analyze unexpectedly accepted a zero display limit" >&2
  exit 1
fi
"${analyzer}" --max-display-points 3 "${fixture}" >"${test_dir}/analysis.json"
python3 - "${test_dir}/analysis.json" "${test_dir}/startup.pcworkbench" "${fixture}" <<'PY'
import json
import pathlib
import sys

result = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert result["points"]["finite"] == 12
assert result["points"]["displayed"] <= 3
assert result["points"]["display_downsampled"] is True
pathlib.Path(sys.argv[2]).write_text(json.dumps({
    "format": "point-cloud-workbench-project",
    "version": 8,
    "cloud_path": str(pathlib.Path(sys.argv[3]).resolve()),
}), encoding="utf-8")
PY

mkdir -p "${test_dir}/home" "${test_dir}/config" "${test_dir}/data"
mkdir -p "${test_dir}/config/PCD Tools"
printf '[General]\nmaximumDisplayPoints=123456\n' \
  >"${test_dir}/config/PCD Tools/PCD Measure.conf"
for startup_file in "${fixture}" "${test_dir}/startup.pcworkbench"; do
  env HOME="${test_dir}/home" \
    XDG_CONFIG_HOME="${test_dir}/config" \
    XDG_DATA_HOME="${test_dir}/data" \
    POINT_CLOUD_WORKBENCH_TEST_EXIT_AFTER_LOAD=1 \
    timeout 10 "${application}" "${startup_file}" \
      >"${test_dir}/startup.out" 2>"${test_dir}/startup.err"
done
new_settings="${test_dir}/config/Point Cloud Workbench/Point Cloud Workbench.conf"
test -s "${new_settings}"
grep -Fq 'maximumDisplayPoints=123456' "${new_settings}"

for invalid_jobs in 0 invalid; do
  if POINT_CLOUD_WORKBENCH_BUILD_JOBS="${invalid_jobs}" "${project_dir}/build.sh" \
    >"${test_dir}/invalid-jobs.out" 2>"${test_dir}/invalid-jobs.err"; then
    echo "build.sh unexpectedly accepted an invalid job count" >&2
    exit 1
  fi
done

desktop_dir="${test_dir}/Desktop"
applications_dir="${test_dir}/applications"
legacy_desktop_launcher="${desktop_dir}/点云测量工具.desktop"
legacy_menu_launcher="${applications_dir}/pcd-measure.desktop"
mkdir -p "${desktop_dir}" "${applications_dir}"
printf '[Desktop Entry]\nExec="%s/run.sh" %%F\n' "${project_dir}" \
  >"${legacy_desktop_launcher}"
printf '[Desktop Entry]\nName=点云工作台\nExec="%s/run.sh" %%F\nIcon=%s/assets/pcd_measure.svg\n' \
  "${project_dir}" "${project_dir}" >"${legacy_menu_launcher}"
POINT_CLOUD_WORKBENCH_DESKTOP_DIR="${desktop_dir}" \
POINT_CLOUD_WORKBENCH_APPLICATIONS_DIR="${applications_dir}" \
POINT_CLOUD_WORKBENCH_SKIP_SYSTEM_SETUP=1 \
POINT_CLOUD_WORKBENCH_SKIP_ROSBAG_SETUP=1 \
POINT_CLOUD_WORKBENCH_SKIP_ODIN_MAP_SETUP=1 \
  "${project_dir}/install_desktop.sh" >"${test_dir}/install.log"

desktop_launcher="${desktop_dir}/点云工作台.desktop"
menu_launcher="${applications_dir}/point-cloud-workbench.desktop"
test -x "${desktop_launcher}"
test -x "${menu_launcher}"
test ! -e "${legacy_desktop_launcher}"
test ! -e "${legacy_menu_launcher}"
grep -Fq "Name=点云工作台" "${desktop_launcher}"
grep -Fq "Exec=\"${project_dir}/run.sh\" %F" "${desktop_launcher}"
grep -Fq "Icon=${project_dir}/assets/point_cloud_workbench.svg" "${desktop_launcher}"
grep -Fq "Comment=查看、测量、采集与分析点云，回放并诊断 ROS bag" "${desktop_launcher}"
if grep -Eiq '^Name=.*odin' "${desktop_launcher}"; then
  echo "product name must not contain Odin" >&2
  exit 1
fi

grep -Fq './install_desktop.sh' "${project_dir}/README.md"
grep -Fq '点云工作台' "${project_dir}/README.md"
grep -Fq '环境自检与一键安装' "${project_dir}/README.md"
grep -Fq 'https://github.com/asoming/point-cloud-workbench.git' "${project_dir}/README.md"
if grep -Fq 'github.com/asoming/pcd-measure' "${project_dir}/README.md"; then
  echo "README still points to the legacy GitHub repository" >&2
  exit 1
fi
test -s "${project_dir}/docs/images/environment-setup.png"
