#include <cmath>
#include <limits>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTest>

#include <pcl/common/transforms.h>

#include "comparison_tools.h"

namespace
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr make_asymmetric_cloud()
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 9; ++y) {
      pcl::PointXYZRGB point;
      point.x = static_cast<float>(x) * 0.12F;
      point.y = static_cast<float>(y) * 0.11F;
      point.z = 0.025F * static_cast<float>(x) +
        0.004F * static_cast<float>(y * y) + (x > 10 && y < 3 ? 0.2F : 0.0F);
      point.r = static_cast<std::uint8_t>(20 + x * 8);
      point.g = static_cast<std::uint8_t>(30 + y * 15);
      point.b = 180;
      cloud->push_back(point);
    }
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

bool near(double actual, double expected, double tolerance)
{
  return std::abs(actual - expected) <= tolerance;
}

}  // namespace

class ComparisonToolsTest : public QObject
{
  Q_OBJECT

private slots:
  void optionValidation()
  {
    CloudComparisonOptions options;
    QString error;
    QVERIFY(valid_comparison_options(options, &error));
    options.voxel_size = -1.0;
    QVERIFY(!valid_comparison_options(options, &error));
    options.voxel_size = std::numeric_limits<double>::quiet_NaN();
    QVERIFY(!valid_comparison_options(options, &error));
    options.voxel_size = 0.0;
    options.maximum_iterations = 0;
    QVERIFY(!valid_comparison_options(options, &error));
    options.maximum_iterations = 10;
    options.maximum_correspondence_distance = 0.0;
    QVERIFY(!valid_comparison_options(options, &error));
    options.maximum_correspondence_distance = 0.2;
    options.difference_threshold = -0.1;
    QVERIFY(!valid_comparison_options(options, &error));
    options.difference_threshold = 0.0;
    options.maximum_display_points = 0;
    QVERIFY(!valid_comparison_options(options, &error));
    options.maximum_display_points = 100;
    options.maximum_iterations = 100001;
    QVERIFY(!valid_comparison_options(options, &error));
    options.maximum_iterations = 20;
    options.maximum_correspondence_distance = std::numeric_limits<double>::infinity();
    QVERIFY(!valid_comparison_options(options, &error));
    options.maximum_correspondence_distance = 0.2;
    options.difference_threshold = std::numeric_limits<double>::quiet_NaN();
    QVERIFY(!valid_comparison_options(options, &error));

    options = CloudComparisonOptions{};
    options.voxel_size = 0.0;
    options.maximum_iterations = 1;
    options.maximum_correspondence_distance = 1e-12;
    options.difference_threshold = 0.0;
    options.maximum_display_points = 1;
    QVERIFY2(valid_comparison_options(options, &error), qPrintable(error));
    options.maximum_iterations = 100000;
    options.maximum_correspondence_distance = 1e9;
    options.difference_threshold = 1e9;
    options.maximum_display_points = std::numeric_limits<std::size_t>::max();
    QVERIFY2(valid_comparison_options(options, &error), qPrintable(error));
  }

  void heatColors()
  {
    const auto zero = difference_heat_color(0.0, 1.0);
    const auto third = difference_heat_color(1.0 / 3.0, 1.0);
    const auto two_thirds = difference_heat_color(2.0 / 3.0, 1.0);
    const auto maximum = difference_heat_color(2.0, 1.0);
    QCOMPARE(zero, (std::array<std::uint8_t, 3>{30, 105, 255}));
    QCOMPARE(third, (std::array<std::uint8_t, 3>{20, 220, 210}));
    QCOMPARE(two_thirds, (std::array<std::uint8_t, 3>{255, 205, 35}));
    QCOMPARE(maximum, (std::array<std::uint8_t, 3>{255, 55, 40}));
    QCOMPARE(difference_heat_color(0.0, 0.0), zero);
    QCOMPARE(difference_heat_color(0.1, 0.0), maximum);
    QCOMPARE(difference_heat_color(std::numeric_limits<double>::quiet_NaN(), 1.0), maximum);
    QCOMPARE(difference_heat_color(-0.1, 1.0), maximum);
  }

  void identicalCloudDirectComparison()
  {
    const auto cloud = make_asymmetric_cloud();
    CloudComparisonOptions options;
    options.run_icp = false;
    options.difference_threshold = 0.0;
    const CloudComparisonResult result = compare_point_clouds(cloud, cloud, options);
    QVERIFY2(result.valid, qPrintable(result.error));
    QVERIFY(result.transform.isApprox(Eigen::Matrix4f::Identity(), 1e-7F));
    QCOMPARE(result.statistics.point_count, cloud->size());
    QCOMPARE(result.statistics.over_threshold_count, std::size_t(0));
    QVERIFY(near(result.statistics.maximum, 0.0, 1e-7));
    QCOMPARE(result.aligned_cloud->size(), cloud->size());
    QCOMPARE(result.heatmap_cloud->front().r, std::uint8_t(30));
  }

  void knownRigidIcp()
  {
    const auto reference = make_asymmetric_cloud();
    Eigen::Matrix4f known = Eigen::Matrix4f::Identity();
    const float angle = 5.0F * static_cast<float>(std::acos(-1.0)) / 180.0F;
    known(0, 0) = std::cos(angle);
    known(0, 1) = -std::sin(angle);
    known(1, 0) = std::sin(angle);
    known(1, 1) = std::cos(angle);
    known(0, 3) = 0.24F;
    known(1, 3) = -0.17F;
    known(2, 3) = 0.08F;
    auto moving = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::transformPointCloud(*reference, *moving, known);

    CloudComparisonOptions options;
    options.run_icp = true;
    options.centroid_prealign = true;
    options.voxel_size = 0.0;
    options.maximum_iterations = 200;
    options.maximum_correspondence_distance = 0.35;
    options.difference_threshold = 0.02;
    const CloudComparisonResult result = compare_point_clouds(reference, moving, options);
    QVERIFY2(result.valid, qPrintable(result.error));
    QVERIFY(result.converged);
    QVERIFY2(result.statistics.rmse < 0.01, qPrintable(QString::number(result.statistics.rmse, 'g', 12)));
    const Eigen::Matrix4f identity = result.transform * known;
    QVERIFY2(identity.isApprox(Eigen::Matrix4f::Identity(), 0.02F), "Recovered transform is inaccurate");
  }

  void directDistancesAndThresholdBoundary()
  {
    auto reference = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    auto moving = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (int index = 0; index < 4; ++index) {
      pcl::PointXYZRGB point;
      point.x = static_cast<float>(index);
      point.y = 0.0F;
      point.z = 0.0F;
      reference->push_back(point);
      point.z = index == 0 ? 0.0F : static_cast<float>(index) * 0.1F;
      moving->push_back(point);
    }
    CloudComparisonOptions options;
    options.run_icp = false;
    options.difference_threshold = 0.2;
    const CloudComparisonResult result = compare_point_clouds(reference, moving, options);
    QVERIFY2(result.valid, qPrintable(result.error));
    QVERIFY(near(result.statistics.mean, 0.15, 1e-6));
    QVERIFY(near(result.statistics.rmse, std::sqrt(0.14 / 4.0), 1e-6));
    QVERIFY(near(result.statistics.median, 0.15, 1e-6));
    QVERIFY(near(result.statistics.p95, 0.285, 1e-6));
    QCOMPARE(result.statistics.over_threshold_count, std::size_t(1));
  }

  void invalidAndNoOverlap()
  {
    auto two = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    two->resize(2);
    CloudComparisonOptions options;
    QVERIFY(!compare_point_clouds(two, make_asymmetric_cloud(), options).valid);

    const auto reference = make_asymmetric_cloud();
    auto moving = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*reference);
    for (auto & point : *moving) point.x += 100.0F;
    options.centroid_prealign = false;
    options.voxel_size = 0.0;
    options.maximum_correspondence_distance = 0.001;
    const CloudComparisonResult result = compare_point_clouds(reference, moving, options);
    QVERIFY(!result.valid);
    QVERIFY(!result.error.isEmpty());

    options.centroid_prealign = true;
    options.maximum_correspondence_distance = 0.01;
    auto scaled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*reference);
    for (auto & point : *scaled) {
      point.x *= 100.0F;
      point.y *= 100.0F;
      point.z *= 100.0F;
    }
    QVERIFY(!compare_point_clouds(reference, scaled, options).valid);
    QVERIFY(!compare_point_clouds(nullptr, scaled, options).valid);

    options.voxel_size = 1000.0;
    options.maximum_correspondence_distance = 1.0;
    QVERIFY(!compare_point_clouds(reference, reference, options).valid);
  }

  void sanitizationAndDisplayLod()
  {
    auto reference = make_asymmetric_cloud();
    auto moving = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*reference);
    pcl::PointXYZRGB invalid;
    invalid.x = std::numeric_limits<float>::quiet_NaN();
    invalid.y = 0.0F;
    invalid.z = 0.0F;
    moving->push_back(invalid);
    CloudComparisonOptions options;
    options.run_icp = false;
    options.maximum_display_points = 10;
    const CloudComparisonResult result = compare_point_clouds(reference, moving, options);
    QVERIFY2(result.valid, qPrintable(result.error));
    QCOMPARE(result.statistics.point_count, reference->size());
    QVERIFY(result.display_heatmap_cloud->size() <= std::size_t(10));
    QCOMPARE(result.aligned_cloud->front().r, reference->front().r);
  }

  void exportRoundTrip()
  {
    const auto cloud = make_asymmetric_cloud();
    CloudComparisonOptions options;
    options.run_icp = false;
    options.difference_threshold = 0.01;
    const CloudComparisonResult result = compare_point_clouds(cloud, cloud, options);
    QVERIFY(result.valid);
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    QString error;
    const QString csv = directory.filePath(QStringLiteral("distances.csv"));
    const QString json = directory.filePath(QStringLiteral("summary.json"));
    QVERIFY2(write_comparison_distances_csv(csv, result, &error), qPrintable(error));
    QVERIFY2(write_comparison_summary_json(json, result, QStringLiteral("base.pcd"),
      QStringLiteral("second.pcd"), &error), qPrintable(error));
    QFile csv_file(csv);
    QVERIFY(csv_file.open(QIODevice::ReadOnly));
    const QList<QByteArray> lines = csv_file.readAll().split('\n');
    QCOMPARE(lines.size(), static_cast<int>(cloud->size()) + 2);
    QFile json_file(json);
    QVERIFY(json_file.open(QIODevice::ReadOnly));
    const QJsonObject root = QJsonDocument::fromJson(json_file.readAll()).object();
    QCOMPARE(root.value(QStringLiteral("format")).toString(), QStringLiteral("pcd-cloud-comparison"));
    QCOMPARE(root.value(QStringLiteral("transform")).toArray().size(), 4);
    QCOMPARE(root.value(QStringLiteral("statistics")).toObject()
      .value(QStringLiteral("point_count")).toInt(), static_cast<int>(cloud->size()));

    CloudComparisonResult invalid;
    QVERIFY(!write_comparison_distances_csv(
      directory.filePath(QStringLiteral("invalid.csv")), invalid, &error));
    QVERIFY(!write_comparison_summary_json(
      directory.filePath(QStringLiteral("invalid.json")), invalid,
      QStringLiteral("base.pcd"), QStringLiteral("second.pcd"), &error));
    QVERIFY(!write_comparison_distances_csv(directory.path(), result, &error));
    QVERIFY(!write_comparison_summary_json(directory.path(), result,
      QStringLiteral("base.pcd"), QStringLiteral("second.pcd"), &error));
  }
};

QTEST_GUILESS_MAIN(ComparisonToolsTest)

#include "comparison_tools_test.moc"
