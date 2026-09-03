#!/usr/bin/env bash
set -euo pipefail

project_dir="${1:?project directory is required}"
test_dir="$(mktemp -d /tmp/pcd-measure-rosbag-playback.XXXXXX)"

cleanup() {
  case "${test_dir}" in
    /tmp/pcd-measure-rosbag-playback.*) rm -rf -- "${test_dir}" ;;
  esac
}
trap cleanup EXIT

mkdir -p "${test_dir}/bin" "${test_dir}/bag with spaces;[literal]"
touch "${test_dir}/recording.bag"
touch "${test_dir}/bag with spaces;[literal]/recording.db3"

cat >"${test_dir}/bin/ros2" <<'SH'
#!/usr/bin/env bash
printf '%s\n' "${FAKE_SETUP_MARKER:-unset}" >"${FAKE_OUTPUT:?}"
printf '%s\n' "$@" >>"${FAKE_OUTPUT:?}"
SH
cat >"${test_dir}/bin/rosbag" <<'SH'
#!/usr/bin/env bash
printf '%s\n' "$@" >"${FAKE_OUTPUT:?}"
SH
chmod +x "${test_dir}/bin/ros2" "${test_dir}/bin/rosbag"

cat >"${test_dir}/setup.bash" <<'SH'
export FAKE_SETUP_MARKER=workspace-loaded
SH

ros2_output="${test_dir}/ros2.args"
PATH="${test_dir}/bin:${PATH}" \
FAKE_OUTPUT="${ros2_output}" \
PCD_MEASURE_ROS_SETUP="${test_dir}/setup.bash" \
  "${project_dir}/scripts/rosbag_play.sh" \
    2 "${test_dir}/bag with spaces;[literal]/recording.db3" 0.5 1 1

python3 - "${ros2_output}" "${test_dir}/bag with spaces;[literal]/recording.db3" <<'PY'
import pathlib
import sys

actual = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
expected = [
    "workspace-loaded", "bag", "play", sys.argv[2], "--rate", "0.5",
    "--disable-keyboard-controls", "--loop", "--clock", "100",
]
assert actual == expected, (actual, expected)
PY

ros1_output="${test_dir}/ros1.args"
PATH="${test_dir}/bin:${PATH}" FAKE_OUTPUT="${ros1_output}" \
  "${project_dir}/scripts/rosbag_play.sh" 1 "${test_dir}/recording.bag" 2 0 1
python3 - "${ros1_output}" "${test_dir}/recording.bag" <<'PY'
import pathlib
import sys

actual = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
assert actual == ["play", sys.argv[2], "--rate", "2", "--clock"], actual
PY

if PATH="${test_dir}/bin:${PATH}" FAKE_OUTPUT="${test_dir}/invalid.args" \
  "${project_dir}/scripts/rosbag_play.sh" 2 "${test_dir}/recording.bag" 0 0 0; then
  echo "zero playback rate was unexpectedly accepted" >&2
  exit 1
fi
if PATH="${test_dir}/bin:${PATH}" FAKE_OUTPUT="${test_dir}/invalid.args" \
  PCD_MEASURE_ROS_SETUP="${test_dir}/missing-setup.bash" \
  "${project_dir}/scripts/rosbag_play.sh" 2 "${test_dir}/recording.bag" 1 0 0; then
  echo "missing ROS setup was unexpectedly accepted" >&2
  exit 1
fi
