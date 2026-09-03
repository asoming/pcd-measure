#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <QTest>

#include "analysis_tools.h"

namespace
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr make_cloud(
  const std::vector<std::array<float, 3>> & coordinates)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (std::size_t index = 0; index < coordinates.size(); ++index) {
    pcl::PointXYZRGB point;
    point.x = coordinates[index][0];
    point.y = coordinates[index][1];
    point.z = coordinates[index][2];
    point.r = static_cast<std::uint8_t>(index % 255);
    point.g = 120;
    point.b = 220;
    cloud->push_back(point);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

bool close_to(double actual, double expected, double tolerance = 1e-6)
{
  return std::abs(actual - expected) <= tolerance;
}

}  // namespace

class AnalysisToolsTest : public QObject
{
  Q_OBJECT

private slots:
  void boxSelectionValidationAndInverse()
  {
    const auto cloud = make_cloud({
      {0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F}, {2.0F, 2.0F, 2.0F},
      {3.0F, 3.0F, 3.0F}});
    BoxSelection box{0.0, 2.0, 0.0, 2.0, 0.0, 2.0, false};
    QString error;
    QVERIFY(valid_box_selection(box, &error));
    auto selected = select_box_cloud(cloud, box, &error);
    QCOMPARE(selected->size(), std::size_t(3));
    QCOMPARE(selected->back().x, 2.0F);

    box.inverted = true;
    selected = select_box_cloud(cloud, box, &error);
    QCOMPARE(selected->size(), std::size_t(1));
    QCOMPARE(selected->front().x, 3.0F);

    box.min_x = 2.0;
    box.max_x = 1.0;
    QVERIFY(!valid_box_selection(box, &error));
    QVERIFY(!error.isEmpty());
    box = BoxSelection{0.0, 0.0, 0.0, 1.0, 0.0, 1.0, false};
    QVERIFY(!valid_box_selection(box));
    box = BoxSelection{0.0, 1.0, 0.0, 1.0, 0.0, 1.0, false};
    box.max_z = std::numeric_limits<double>::quiet_NaN();
    QVERIFY(!valid_box_selection(box));
    QVERIFY(select_box_cloud(nullptr, BoxSelection{0.0, 1.0, 0.0, 1.0, 0.0, 1.0})
      ->empty());
  }

  void polygonSelectionConcaveBoundaryAndInvalid()
  {
    std::vector<pcl::PointXYZ> concave{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(3.0F, 0.0F, 0.0F),
      pcl::PointXYZ(3.0F, 3.0F, 0.0F), pcl::PointXYZ(1.5F, 1.5F, 0.0F),
      pcl::PointXYZ(0.0F, 3.0F, 0.0F)};
    QVERIFY(point_in_polygon_xy(0.5, 0.5, concave));
    QVERIFY(point_in_polygon_xy(3.0, 1.0, concave));
    QVERIFY(!point_in_polygon_xy(1.5, 2.5, concave));

    const auto cloud = make_cloud({
      {0.5F, 0.5F, 0.0F}, {3.0F, 1.0F, 0.0F}, {1.5F, 2.5F, 0.0F},
      {0.5F, 0.5F, 5.0F}});
    PolygonSelection selection{concave, -1.0, 1.0, false};
    QString error;
    QVERIFY(valid_polygon_selection(selection, &error));
    auto selected = select_polygon_cloud(cloud, selection, &error);
    QCOMPARE(selected->size(), std::size_t(2));
    selection.inverted = true;
    selected = select_polygon_cloud(cloud, selection, &error);
    QCOMPARE(selected->size(), std::size_t(2));

    PolygonSelection reversed = selection;
    std::reverse(reversed.vertices.begin(), reversed.vertices.end());
    reversed.inverted = false;
    QCOMPARE(select_polygon_cloud(cloud, reversed)->size(), std::size_t(2));

    PolygonSelection crossing{{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 2.0F, 0.0F),
      pcl::PointXYZ(0.0F, 2.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F)}};
    QVERIFY(!valid_polygon_selection(crossing, &error));
    QVERIFY(error.contains(QStringLiteral("自交")) || error.contains(QStringLiteral("面积")));
    PolygonSelection line{{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F),
      pcl::PointXYZ(2.0F, 0.0F, 0.0F)}};
    QVERIFY(!valid_polygon_selection(line));

    PolygonSelection duplicate{{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F),
      pcl::PointXYZ(2.0F, 0.0F, 0.0F), pcl::PointXYZ(0.0F, 2.0F, 0.0F)}};
    QVERIFY(!valid_polygon_selection(duplicate, &error));
    QVERIFY(error.contains(QStringLiteral("重复")));
    PolygonSelection overlapping{{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(3.0F, 0.0F, 0.0F),
      pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 2.0F, 0.0F),
      pcl::PointXYZ(0.0F, 2.0F, 0.0F)}};
    QVERIFY(!valid_polygon_selection(overlapping, &error));
    selection.min_z = 1.0;
    selection.max_z = 1.0;
    QVERIFY(!valid_polygon_selection(selection));
    selection.min_z = std::numeric_limits<double>::quiet_NaN();
    selection.max_z = 2.0;
    QVERIFY(!valid_polygon_selection(selection));
  }

  void planeAndRegionAnalysis()
  {
    std::vector<std::array<float, 3>> points;
    for (int x = 0; x < 5; ++x) {
      for (int y = 0; y < 4; ++y) {
        points.push_back({static_cast<float>(x), static_cast<float>(y),
          static_cast<float>(x + 2 * y + 1)});
      }
    }
    const auto cloud = make_cloud(points);
    const PlaneFitResult plane = fit_plane_pca(cloud);
    QVERIFY2(plane.valid, qPrintable(plane.error));
    const double norm = std::sqrt(6.0);
    QVERIFY(close_to(std::abs(plane.normal[0]), 1.0 / norm, 1e-5));
    QVERIFY(close_to(std::abs(plane.normal[1]), 2.0 / norm, 1e-5));
    QVERIFY(close_to(plane.normal[2], 1.0 / norm, 1e-5));
    QVERIFY(close_to(plane.rms, 0.0, 1e-5));

    const RegionAnalysisResult region = analyze_region(cloud);
    QVERIFY(region.valid);
    QCOMPARE(region.point_count, points.size());
    QVERIFY(close_to(region.bounds.size_x(), 4.0));
    QVERIFY(close_to(region.bounds.size_y(), 3.0));
    QVERIFY(close_to(region.bounds.min_z, 1.0));
    QVERIFY(std::isfinite(region.estimated_spacing));
    QVERIFY(std::isfinite(region.density_xy));

    const auto line = make_cloud({{0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F},
      {2.0F, 2.0F, 2.0F}});
    QVERIFY(!fit_plane_pca(line).valid);
    QVERIFY(!analyze_region(make_cloud({{0.0F, 0.0F, 0.0F}})).valid);

    std::vector<std::array<float, 3>> vertical_points;
    for (int y = 0; y < 4; ++y) {
      for (int z = 0; z < 5; ++z) vertical_points.push_back({2.0F, y * 0.2F, z * 0.3F});
    }
    const PlaneFitResult vertical = fit_plane_pca(make_cloud(vertical_points));
    QVERIFY(vertical.valid);
    QVERIFY(close_to(std::abs(vertical.normal[0]), 1.0, 1e-5));
    QVERIFY(close_to(vertical.slope_degrees, 90.0, 1e-5));

    auto noisy = make_cloud(points);
    noisy->back().z += 4.0F;
    const PlaneFitResult noisy_plane = fit_plane_pca(noisy);
    QVERIFY(noisy_plane.valid);
    QVERIFY(noisy_plane.rms > 0.1);
    QVERIFY(noisy_plane.maximum_absolute_residual > noisy_plane.mean_absolute_residual);
  }

  void histogramConstantAndBoundaries()
  {
    const auto cloud = make_cloud({
      {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 2.0F},
      {0.0F, 0.0F, 3.0F}, {0.0F, 0.0F, 4.0F}});
    const HistogramResult histogram = build_height_histogram(cloud, 4);
    QVERIFY(histogram.valid);
    QCOMPARE(histogram.bins.size(), std::size_t(4));
    std::size_t total = 0;
    for (const HistogramBin & bin : histogram.bins) total += bin.count;
    QCOMPARE(total, cloud->size());
    QCOMPARE(histogram.bins.back().count, std::size_t(2));

    const HistogramResult constant = build_height_histogram(
      make_cloud({{0.0F, 0.0F, 2.0F}, {1.0F, 0.0F, 2.0F}}), 5);
    QVERIFY(constant.valid);
    QCOMPARE(constant.bins.front().count, std::size_t(2));
    QVERIFY(!build_height_histogram(cloud, 0).valid);
    QVERIFY(!build_height_histogram(cloud, 501).valid);
    const HistogramResult one_bin = build_height_histogram(cloud, 1);
    QVERIFY(one_bin.valid);
    QCOMPARE(one_bin.bins.front().count, cloud->size());
    QCOMPARE(build_height_histogram(cloud, 500).bins.size(), std::size_t(500));
    QVERIFY(!build_height_histogram(nullptr, 5).valid);
  }

  void elevationProfile()
  {
    const auto cloud = make_cloud({
      {0.0F, 0.0F, 0.0F}, {0.5F, 0.1F, 1.0F}, {1.0F, -0.1F, 2.0F},
      {1.5F, 0.0F, 3.0F}, {2.0F, 0.0F, 4.0F}, {1.0F, 2.0F, 99.0F}});
    const std::vector<pcl::PointXYZ> path{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F)};
    const ProfileResult profile = build_elevation_profile(cloud, path, 0.4, 1.0);
    QVERIFY2(profile.valid, qPrintable(profile.error));
    QVERIFY(close_to(profile.path_length, 2.0));
    QCOMPARE(profile.selected_points, std::size_t(5));
    QCOMPARE(profile.bins.size(), std::size_t(2));
    QCOMPARE(profile.bins[0].point_count, std::size_t(2));
    QCOMPARE(profile.bins[1].point_count, std::size_t(3));
    QVERIFY(close_to(profile.bins[0].z_mean, 0.5));
    QVERIFY(close_to(profile.bins[1].z_max, 4.0));

    const std::vector<pcl::PointXYZ> path_with_duplicate{
      path[0], path[0], path[1]};
    QVERIFY(build_elevation_profile(cloud, path_with_duplicate, 0.4, 1.0).valid);
    QVERIFY(!build_elevation_profile(cloud, {path[0]}, 0.4, 1.0).valid);
    QVERIFY(!build_elevation_profile(cloud, path, 0.0, 1.0).valid);
    QVERIFY(!build_elevation_profile(cloud, path, 0.4, 0.0).valid);

    const auto sparse = make_cloud({
      {0.0F, 0.0F, 1.0F}, {4.0F, 0.0F, 5.0F},
      {2.0F, 2.0F, 9.0F}});
    const std::vector<pcl::PointXYZ> folded{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(4.0F, 0.0F, 0.0F),
      pcl::PointXYZ(4.0F, 2.0F, 0.0F)};
    const ProfileResult folded_profile = build_elevation_profile(sparse, folded, 0.2, 1.0);
    QVERIFY(folded_profile.valid);
    QCOMPARE(folded_profile.bins.size(), std::size_t(6));
    QVERIFY(std::any_of(folded_profile.bins.begin(), folded_profile.bins.end(),
      [](const ProfileBin & value) { return value.point_count == 0; }));
    QVERIFY(!build_elevation_profile(cloud, path, 0.4, 1e-300).valid);
    std::vector<pcl::PointXYZ> invalid_path = path;
    invalid_path.back().x = std::numeric_limits<float>::infinity();
    QVERIFY(!build_elevation_profile(cloud, invalid_path, 0.4, 1.0).valid);
  }

  void volumeEstimation()
  {
    const std::vector<pcl::PointXYZ> boundary{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F),
      pcl::PointXYZ(2.0F, 2.0F, 0.0F), pcl::PointXYZ(0.0F, 2.0F, 0.0F)};
    const auto high = make_cloud({
      {0.5F, 0.5F, 2.0F}, {1.5F, 0.5F, 2.0F},
      {0.5F, 1.5F, 2.0F}, {1.5F, 1.5F, 2.0F}});
    const VolumeResult volume = estimate_grid_volume(high, boundary, 0.0, 1.0);
    QVERIFY2(volume.valid, qPrintable(volume.error));
    QCOMPARE(volume.occupied_cells, std::size_t(4));
    QCOMPARE(volume.possible_cells, std::size_t(4));
    QVERIFY(close_to(volume.boundary_area, 4.0));
    QVERIFY(close_to(volume.covered_area, 4.0));
    QVERIFY(close_to(volume.volume_above, 8.0));
    QVERIFY(close_to(volume.volume_below, 0.0));
    QVERIFY(close_to(volume.net_volume, 8.0));

    const auto mixed = make_cloud({
      {0.5F, 0.5F, 2.0F}, {1.5F, 0.5F, -1.0F},
      {0.5F, 1.5F, 2.0F}, {1.5F, 1.5F, -1.0F}});
    const VolumeResult mixed_volume = estimate_grid_volume(mixed, boundary, 0.0, 1.0);
    QVERIFY(close_to(mixed_volume.volume_above, 4.0));
    QVERIFY(close_to(mixed_volume.volume_below, 2.0));
    QVERIFY(close_to(mixed_volume.net_volume, 2.0));
    QVERIFY(!estimate_grid_volume(mixed, boundary, 0.0, 0.0).valid);
    QVERIFY(!estimate_grid_volume(mixed, boundary, 0.0, 0.0001).valid);

    const VolumeResult below = estimate_grid_volume(mixed, boundary, 3.0, 1.0);
    QVERIFY(below.valid);
    QVERIFY(close_to(below.volume_above, 0.0));
    QVERIFY(close_to(below.volume_below, 10.0));
    const VolumeResult baseline = estimate_grid_volume(
      make_cloud({{0.5F, 0.5F, 2.0F}, {1.5F, 0.5F, 2.0F},
        {0.5F, 1.5F, 2.0F}, {1.5F, 1.5F, 2.0F}}), boundary, 2.0, 1.0);
    QVERIFY(baseline.valid);
    QVERIFY(close_to(baseline.net_volume, 0.0));

    std::vector<pcl::PointXYZ> concave{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F),
      pcl::PointXYZ(2.0F, 1.0F, 0.0F), pcl::PointXYZ(1.0F, 1.0F, 0.0F),
      pcl::PointXYZ(1.0F, 2.0F, 0.0F), pcl::PointXYZ(0.0F, 2.0F, 0.0F)};
    const auto concave_cloud = make_cloud({
      {0.5F, 0.5F, 2.0F}, {1.5F, 0.5F, 2.0F}, {0.5F, 1.5F, 2.0F}});
    const VolumeResult concave_volume = estimate_grid_volume(
      concave_cloud, concave, 0.0, 1.0);
    QVERIFY2(concave_volume.valid, qPrintable(concave_volume.error));
    QVERIFY(close_to(concave_volume.boundary_area, 3.0));
    QCOMPARE(concave_volume.possible_cells, std::size_t(3));
    QCOMPARE(concave_volume.occupied_cells, std::size_t(3));
    QVERIFY(close_to(concave_volume.net_volume, 6.0));
    std::reverse(concave.begin(), concave.end());
    const VolumeResult reversed_concave = estimate_grid_volume(
      concave_cloud, concave, 0.0, 1.0);
    QVERIFY(reversed_concave.valid);
    QVERIFY(close_to(reversed_concave.net_volume, concave_volume.net_volume));

    const auto outside = make_cloud({{10.0F, 10.0F, 1.0F}, {11.0F, 10.0F, 1.0F},
      {10.0F, 11.0F, 1.0F}});
    QVERIFY(!estimate_grid_volume(outside, boundary, 0.0, 1.0).valid);
    const std::vector<pcl::PointXYZ> crossing{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 2.0F, 0.0F),
      pcl::PointXYZ(0.0F, 2.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F)};
    QVERIFY(!estimate_grid_volume(mixed, crossing, 0.0, 1.0).valid);
    QVERIFY(!estimate_grid_volume(mixed, boundary, 0.0, 1e-300).valid);
  }

  void statisticalOutlierFilter()
  {
    std::vector<std::array<float, 3>> points;
    for (int x = 0; x < 10; ++x) {
      for (int y = 0; y < 10; ++y) {
        points.push_back({x * 0.1F, y * 0.1F, 0.0F});
      }
    }
    points.push_back({100.0F, 100.0F, 100.0F});
    const auto cloud = make_cloud(points);
    const OutlierFilterResult result = filter_statistical_outliers(cloud, 8, 1.0);
    QVERIFY2(result.valid, qPrintable(result.error));
    QVERIFY(result.kept->size() >= std::size_t(90));
    QVERIFY(!result.removed->empty());
    bool found_far_point = false;
    for (const pcl::PointXYZRGB & point : *result.removed) {
      if (point.x > 50.0F) found_far_point = true;
    }
    QVERIFY(found_far_point);
    QCOMPARE(result.kept->size() + result.removed->size(), cloud->size());
    QVERIFY(!filter_statistical_outliers(cloud, 1, 1.0).valid);
    QVERIFY(!filter_statistical_outliers(cloud, 8, 0.0).valid);
    QVERIFY(!filter_statistical_outliers(cloud, static_cast<int>(cloud->size()), 1.0).valid);
    QVERIFY(!filter_statistical_outliers(make_cloud({{0.0F, 0.0F, 0.0F}}), 2, 1.0).valid);
  }

  void dominantPlaneExtraction()
  {
    std::vector<std::array<float, 3>> points;
    for (int x = 0; x < 10; ++x) {
      for (int y = 0; y < 10; ++y) {
        points.push_back({x * 0.1F, y * 0.1F, 0.0F});
      }
    }
    for (int index = 0; index < 10; ++index) {
      points.push_back({index * 0.1F, 0.2F, 2.0F + index * 0.1F});
    }
    const auto cloud = make_cloud(points);
    const DominantPlaneResult result = extract_dominant_plane(cloud, 0.01, 500);
    QVERIFY2(result.valid, qPrintable(result.error));
    QCOMPARE(result.source_points, cloud->size());
    QVERIFY(result.inlier_points >= std::size_t(100));
    QVERIFY(result.inlier_ratio > 0.85);
    QVERIFY(close_to(std::abs(result.coefficients[2]), 1.0, 1e-5));
    QVERIFY(close_to(result.coefficients[3], 0.0, 1e-5));
    QCOMPARE(result.inliers->size() + result.remainder->size(), cloud->size());
    QVERIFY(!extract_dominant_plane(cloud, 0.0).valid);
    QVERIFY(!extract_dominant_plane(make_cloud({{0.0F, 0.0F, 0.0F}}), 0.1).valid);

    std::vector<std::array<float, 3>> vertical;
    for (int y = 0; y < 10; ++y) {
      for (int z = 0; z < 10; ++z) vertical.push_back({1.5F, y * 0.1F, z * 0.1F});
    }
    const DominantPlaneResult vertical_result = extract_dominant_plane(
      make_cloud(vertical), 0.001, 300);
    QVERIFY(vertical_result.valid);
    QVERIFY(close_to(std::abs(vertical_result.coefficients[0]), 1.0, 1e-4));

    std::vector<std::array<float, 3>> no_plane;
    for (int index = 1; index <= 200; ++index) {
      const float value = static_cast<float>(index) * 0.0137F;
      no_plane.push_back({value, value * value, value * value * value});
    }
    QVERIFY(!extract_dominant_plane(make_cloud(no_plane), 1e-8, 1000).valid);
  }

  void nonFiniteCloudsAreRejected()
  {
    auto cloud = make_cloud({{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F},
      {0.0F, 1.0F, 0.0F}, {1.0F, 1.0F, 0.0F}});
    cloud->back().z = std::numeric_limits<float>::quiet_NaN();
    QVERIFY(!fit_plane_pca(cloud).valid);
    QVERIFY(!analyze_region(cloud).valid);
    QVERIFY(!build_height_histogram(cloud, 4).valid);
    const std::vector<pcl::PointXYZ> path{pcl::PointXYZ(), pcl::PointXYZ(1.0F, 0.0F, 0.0F)};
    QVERIFY(!build_elevation_profile(cloud, path, 1.0, 0.5).valid);
    const std::vector<pcl::PointXYZ> boundary{
      pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F),
      pcl::PointXYZ(1.0F, 1.0F, 0.0F), pcl::PointXYZ(0.0F, 1.0F, 0.0F)};
    QVERIFY(!estimate_grid_volume(cloud, boundary, 0.0, 0.5).valid);
    QVERIFY(!filter_statistical_outliers(cloud, 2, 1.0).valid);
    QVERIFY(!extract_dominant_plane(cloud, 0.01).valid);
  }
};

QTEST_APPLESS_MAIN(AnalysisToolsTest)

#include "analysis_tools_test.moc"
