#!/usr/bin/env bash
set -euo pipefail

project_dir="${1:?project directory is required}"
analyzer="${2:?analyzer path is required}"
application="${3:?application path is required}"
fixture="${project_dir}/tests/fixtures/xyz_no_color_ascii.pcd"
test_dir="$(mktemp -d /tmp/pcd-measure-package-test.XXXXXX)"

cleanup() {
  case "${test_dir}" in
    /tmp/pcd-measure-package-test.*) rm -rf -- "${test_dir}" ;;
  esac
}
trap cleanup EXIT

bash -n "${project_dir}/build.sh"
bash -n "${project_dir}/run.sh"
bash -n "${project_dir}/install_desktop.sh"
bash -n "${project_dir}/scripts/verify_features.sh"
bash -n "${project_dir}/scripts/rosbag_play.sh"
bash -n "${project_dir}/scripts/rosbag_diagnose.sh"
bash -n "${project_dir}/scripts/setup_rosbag_tools.sh"
bash -n "${project_dir}/scripts/setup_odin_map_tools.sh"
bash -n "${project_dir}/tests/rosbag_playback_test.sh"
"${project_dir}/tools/rosbag_diagnose.py" --help >/dev/null

if "${analyzer}" >"${test_dir}/missing-argument.out" 2>"${test_dir}/missing-argument.err"; then
  echo "pcd_analyze unexpectedly accepted an empty argument list" >&2
  exit 1
fi
if "${analyzer}" --max-display-points 0 "${fixture}" \
  >"${test_dir}/invalid-limit.out" 2>"${test_dir}/invalid-limit.err"; then
  echo "pcd_analyze unexpectedly accepted a zero display limit" >&2
  exit 1
fi
"${analyzer}" --max-display-points 3 "${fixture}" >"${test_dir}/analysis.json"
python3 - "${test_dir}/analysis.json" "${test_dir}/startup.pcdmeasure" "${fixture}" <<'PY'
import json
import pathlib
import sys

result = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert result["points"]["finite"] == 12
assert result["points"]["displayed"] <= 3
assert result["points"]["display_downsampled"] is True
pathlib.Path(sys.argv[2]).write_text(json.dumps({
    "format": "pcd-measure-project",
    "version": 7,
    "pcd_path": str(pathlib.Path(sys.argv[3]).resolve()),
}), encoding="utf-8")
PY

mkdir -p "${test_dir}/home" "${test_dir}/config" "${test_dir}/data"
for startup_file in "${fixture}" "${test_dir}/startup.pcdmeasure"; do
  env HOME="${test_dir}/home" \
    XDG_CONFIG_HOME="${test_dir}/config" \
    XDG_DATA_HOME="${test_dir}/data" \
    PCD_MEASURE_TEST_EXIT_AFTER_LOAD=1 \
    timeout 10 "${application}" "${startup_file}" \
      >"${test_dir}/startup.out" 2>"${test_dir}/startup.err"
done

for invalid_jobs in 0 invalid; do
  if PCD_MEASURE_BUILD_JOBS="${invalid_jobs}" "${project_dir}/build.sh" \
    >"${test_dir}/invalid-jobs.out" 2>"${test_dir}/invalid-jobs.err"; then
    echo "build.sh unexpectedly accepted an invalid job count" >&2
    exit 1
  fi
done

desktop_dir="${test_dir}/Desktop"
applications_dir="${test_dir}/applications"
PCD_MEASURE_DESKTOP_DIR="${desktop_dir}" \
PCD_MEASURE_APPLICATIONS_DIR="${applications_dir}" \
PCD_MEASURE_SKIP_ROSBAG_SETUP=1 \
PCD_MEASURE_SKIP_ODIN_MAP_SETUP=1 \
  "${project_dir}/install_desktop.sh" >"${test_dir}/install.log"

desktop_launcher="${desktop_dir}/点云测量工具.desktop"
menu_launcher="${applications_dir}/pcd-measure.desktop"
test -x "${desktop_launcher}"
test -x "${menu_launcher}"
grep -Fq "Name=点云测量工具" "${desktop_launcher}"
grep -Fq "Exec=\"${project_dir}/run.sh\" %F" "${desktop_launcher}"
grep -Fq "Icon=${project_dir}/assets/pcd_measure.svg" "${desktop_launcher}"
grep -Fq "Comment=查看、测量与分析 PCD、PLY、BIN 和 OLX 点云，诊断 ROS bag" "${desktop_launcher}"
if grep -Eiq '^Name=.*odin' "${desktop_launcher}"; then
  echo "product name must not contain Odin" >&2
  exit 1
fi

grep -Fq './install_desktop.sh' "${project_dir}/README.md"
grep -Fq '点云测量工具' "${project_dir}/README.md"
