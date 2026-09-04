#include <algorithm>
#include <cmath>

#include <QFileInfo>
#include <QTemporaryDir>
#include <QTest>

#include <pcl/io/pcd_io.h>

#include "analysis_tools.h"
#include "cloud_data.h"
#include "comparison_tools.h"
#include "transform_tools.h"

class RealWorkflowTest : public QObject
{
  Q_OBJECT

private slots:
  void completeReferenceWorkflow()
  {
    const QString path = QString::fromLocal8Bit(qgetenv("POINT_CLOUD_WORKBENCH_TEST_PCD"));
    if (path.isEmpty() || !QFileInfo::exists(path)) {
      QSKIP("POINT_CLOUD_WORKBENCH_TEST_PCD is not set to an existing reference cloud");
    }
    const qint64 original_size = QFileInfo(path).size();
    const CloudLoadResult cloud = load_pcd_and_analyze(path, 100000);
    QVERIFY2(cloud.ok(), qPrintable(cloud.error));
    QCOMPARE(cloud.metrics.finite_points, std::size_t(1514634));
    QVERIFY(cloud.metrics.has_rgb);
    QVERIFY(cloud.display_cloud->size() <= std::size_t(100000));

    const RegionAnalysisResult whole = analyze_region(cloud.cloud);
    QVERIFY2(whole.valid, qPrintable(whole.error));
    QCOMPARE(whole.point_count, cloud.metrics.finite_points);
    QVERIFY(std::isfinite(whole.estimated_spacing));

    const Bounds3d & bounds = cloud.metrics.raw_bounds;
    const double x_span = bounds.size_x();
    const double y_span = bounds.size_y();
    BoxSelection box{
      bounds.min_x + x_span * 0.2, bounds.max_x - x_span * 0.2,
      bounds.min_y + y_span * 0.2, bounds.max_y - y_span * 0.2,
      bounds.min_z, bounds.max_z, false};
    QString error;
    const auto region = select_box_cloud(cloud.cloud, box, &error);
    QVERIFY2(error.isEmpty(), qPrintable(error));
    QVERIFY(region->size() > std::size_t(1000));
    QVERIFY(region->size() < cloud.cloud->size());
    box.inverted = true;
    const auto inverse = select_box_cloud(cloud.cloud, box, &error);
    QCOMPARE(region->size() + inverse->size(), cloud.cloud->size());

    const HistogramResult histogram = build_height_histogram(region, 50);
    QVERIFY2(histogram.valid, qPrintable(histogram.error));
    std::size_t histogram_count = 0;
    for (const HistogramBin & bin : histogram.bins) histogram_count += bin.count;
    QCOMPARE(histogram_count, region->size());

    const double middle_y = (box.min_y + box.max_y) * 0.5;
    const std::vector<pcl::PointXYZ> profile_path{
      pcl::PointXYZ(box.min_x, middle_y, 0.0), pcl::PointXYZ(box.max_x, middle_y, 0.0)};
    const ProfileResult profile = build_elevation_profile(region, profile_path, 0.4, 0.25);
    QVERIFY2(profile.valid, qPrintable(profile.error));
    QVERIFY(profile.selected_points > 0);
    QVERIFY(!profile.bins.empty());

    const std::vector<pcl::PointXYZ> area{
      pcl::PointXYZ(box.min_x, box.min_y, 0.0), pcl::PointXYZ(box.max_x, box.min_y, 0.0),
      pcl::PointXYZ(box.max_x, box.max_y, 0.0), pcl::PointXYZ(box.min_x, box.max_y, 0.0)};
    const VolumeResult volume = estimate_grid_volume(
      region, area, whole.centroid[2], 0.5);
    QVERIFY2(volume.valid, qPrintable(volume.error));
    QVERIFY(volume.occupied_cells > 0);
    QVERIFY(std::isfinite(volume.net_volume));

    const OutlierFilterResult filtered = filter_statistical_outliers(cloud.display_cloud, 20, 1.0);
    QVERIFY2(filtered.valid, qPrintable(filtered.error));
    QCOMPARE(filtered.kept->size() + filtered.removed->size(), cloud.display_cloud->size());
    QVERIFY(filtered.kept->size() > cloud.display_cloud->size() / 2);

    const DominantPlaneResult plane = extract_dominant_plane(cloud.display_cloud, 0.08, 1500);
    QVERIFY2(plane.valid, qPrintable(plane.error));
    QVERIFY(plane.inlier_points > 100);
    QCOMPARE(plane.inliers->size() + plane.remainder->size(), cloud.display_cloud->size());

    CloudTransformParameters parameters;
    parameters.translation_x = 0.04;
    parameters.translation_y = -0.03;
    parameters.translation_z = 0.02;
    parameters.rotation_z_degrees = 0.5;
    const Eigen::Matrix4f known = cloud_transform_matrix(parameters);
    const auto moved_sample = transform_point_cloud(cloud.display_cloud, known);
    CloudComparisonOptions comparison_options;
    comparison_options.voxel_size = 0.08;
    comparison_options.maximum_iterations = 120;
    comparison_options.maximum_correspondence_distance = 0.35;
    comparison_options.difference_threshold = 0.05;
    comparison_options.maximum_display_points = 50000;
    const CloudComparisonResult comparison = compare_point_clouds(
      cloud.display_cloud, moved_sample, comparison_options);
    QVERIFY2(comparison.valid, qPrintable(comparison.error));
    QVERIFY(comparison.converged);
    QVERIFY(comparison.statistics.rmse < 0.05);

    const auto transformed_full = transform_point_cloud(cloud.cloud, known);
    QCOMPARE(transformed_full->size(), cloud.cloud->size());
    QCOMPARE(transformed_full->front().r, cloud.cloud->front().r);
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString region_path = directory.filePath(QStringLiteral("region.pcd"));
    QCOMPARE(pcl::io::savePCDFileBinaryCompressed(region_path.toStdString(), *region), 0);
    const CloudLoadResult region_reload = load_pcd_and_analyze(region_path, 50000);
    QVERIFY2(region_reload.ok(), qPrintable(region_reload.error));
    QCOMPARE(region_reload.cloud->size(), region->size());
    QVERIFY(region_reload.metrics.has_rgb);
    const QString summary_path = directory.filePath(QStringLiteral("comparison.json"));
    QVERIFY2(write_comparison_summary_json(summary_path, comparison, path,
      QStringLiteral("generated-moving-sample.pcd"), &error), qPrintable(error));
    QVERIFY(QFileInfo(summary_path).size() > 200);

    QCOMPARE(QFileInfo(path).size(), original_size);
  }
};

QTEST_GUILESS_MAIN(RealWorkflowTest)

#include "real_workflow_test.moc"
