#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pcd_path="${1:-}"

if [[ -z "$pcd_path" || ! -f "$pcd_path" ]]; then
  echo "用法：$0 /path/to/reference_cloud.pcd" >&2
  exit 2
fi

cmake --build "$project_dir/build" -j2

verify_dir="$(mktemp -d /tmp/pcd-measure-verify.XXXXXX)"
cleanup() {
  case "$verify_dir" in
    /tmp/pcd-measure-verify.*) rm -rf -- "$verify_dir" ;;
  esac
}
trap cleanup EXIT

"$project_dir/build/bin/pcd_analyze" "$pcd_path" > "$verify_dir/full.json"
"$project_dir/build/bin/pcd_analyze" --max-display-points 100000 \
  "$pcd_path" > "$verify_dir/lod.json"
"$project_dir/build/bin/pcd_analyze" \
  "$project_dir/tests/fixtures/xyz_no_color_ascii.pcd" > "$verify_dir/no_color.json"
"$project_dir/build/bin/pcd_analyze" \
  "$project_dir/tests/fixtures/invalid_points_ascii.pcd" > "$verify_dir/invalid.json"

python3 - "$verify_dir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
full = json.loads((root / "full.json").read_text())
lod = json.loads((root / "lod.json").read_text())
no_color = json.loads((root / "no_color.json").read_text())
invalid = json.loads((root / "invalid.json").read_text())

assert full["points"]["header"] == 1_514_634
assert full["points"]["finite"] == 1_514_634
assert full["points"]["non_black"] == 1_511_348
length, width, height = full["robust_oriented_box"]["extent_major_minor_height_m"]
assert abs(length - 19.3464) < 0.01
assert abs(width - 11.3436) < 0.01
assert abs(height - 3.9527) < 0.01
assert abs(full["robust_oriented_box"]["diagonal_3d_m"] - 22.7724) < 0.01
assert full["lowest_point_m"][2] == full["raw_aabb"]["min_m"][2]
assert full["highest_point_m"][2] == full["raw_aabb"]["max_m"][2]
assert lod["points"]["display_downsampled"] is True
assert lod["points"]["displayed"] <= 100_000
assert lod["points"]["finite"] == full["points"]["finite"]
assert lod["robust_oriented_box"] == full["robust_oriented_box"]
assert no_color["points"]["non_black"] == 0
assert invalid["points"]["invalid"] == 2

print("核心统计与显示降采样：通过")
print(f"全量点数 {full['points']['finite']:,}，降采样显示 {lod['points']['displayed']:,}")
print(f"推荐尺寸 {length:.3f} × {width:.3f} × {height:.3f} m")
PY

ctest --test-dir "$project_dir/build" --output-on-failure
PCD_MEASURE_TEST_PCD="$pcd_path" \
PCD_MEASURE_TEST_REPORT_PATH="$verify_dir/reference-report.pdf" \
PCD_MEASURE_TEST_ACTUAL_SCREENSHOT="$verify_dir/reference-ui.png" \
PCD_MEASURE_TEST_ACTUAL_ANNOTATION_SCREENSHOT="$verify_dir/reference-annotations.png" \
PCD_MEASURE_TEST_UI_SCREENSHOT="$verify_dir/minimum-ui.png" \
  "$project_dir/build/bin/pcd_gui_test"
PCD_MEASURE_TEST_PCD="$pcd_path" \
  "$project_dir/build/bin/pcd_real_workflow_test"
test -s "$verify_dir/reference-report.pdf"
test -s "$verify_dir/reference-ui.png"
test -s "$verify_dir/reference-annotations.png"
test -s "$verify_dir/minimum-ui.png"
echo "点云工作台全部自动验收通过。"
