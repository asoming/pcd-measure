#include <cmath>
#include <limits>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTest>

#include "transform_tools.h"

class TransformToolsTest : public QObject
{
  Q_OBJECT

private slots:
  void validationAndAngleNormalization()
  {
    CloudTransformParameters parameters;
    QString error;
    QVERIFY(valid_cloud_transform(parameters, &error));
    parameters.uniform_scale = 0.0;
    QVERIFY(!valid_cloud_transform(parameters, &error));
    parameters.uniform_scale = -1.0;
    QVERIFY(!valid_cloud_transform(parameters, &error));
    parameters.uniform_scale = 1.0;
    parameters.translation_x = std::numeric_limits<double>::infinity();
    QVERIFY(!valid_cloud_transform(parameters, &error));
    parameters.translation_x = 0.0;
    parameters.uniform_scale = 1000001.0;
    QVERIFY(!valid_cloud_transform(parameters, &error));
    QCOMPARE(normalized_degrees(360.0), 0.0);
    QCOMPARE(normalized_degrees(540.0), 180.0);
    QCOMPARE(normalized_degrees(-540.0), 180.0);
    QCOMPARE(normalized_degrees(-181.0), 179.0);
    QVERIFY(std::isnan(normalized_degrees(std::numeric_limits<double>::quiet_NaN())));
  }

  void matrixAndPointTransformation()
  {
    CloudTransformParameters parameters;
    parameters.translation_x = 10.0;
    parameters.translation_y = -2.0;
    parameters.translation_z = 3.0;
    parameters.rotation_z_degrees = 90.0;
    parameters.uniform_scale = 2.0;
    const Eigen::Matrix4f matrix = cloud_transform_matrix(parameters);
    const Eigen::Vector4f value = matrix * Eigen::Vector4f(1.0F, 2.0F, 3.0F, 1.0F);
    QVERIFY(std::abs(value.x() - 6.0F) < 1e-5F);
    QVERIFY(std::abs(value.y() - 0.0F) < 1e-5F);
    QVERIFY(std::abs(value.z() - 9.0F) < 1e-5F);
    QVERIFY(std::abs(value.w() - 1.0F) < 1e-6F);

    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointXYZRGB point;
    point.x = 1.0F;
    point.y = 2.0F;
    point.z = 3.0F;
    point.r = 12;
    point.g = 34;
    point.b = 56;
    cloud->push_back(point);
    const auto transformed = transform_point_cloud(cloud, matrix);
    QCOMPARE(transformed->size(), std::size_t(1));
    QCOMPARE(transformed->front().r, std::uint8_t(12));
    QVERIFY(std::abs(transformed->front().x - 6.0F) < 1e-5F);
  }

  void invalidAndNonFiniteCloud()
  {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointXYZRGB valid;
    valid.x = 1.0F;
    valid.y = 2.0F;
    valid.z = 3.0F;
    cloud->push_back(valid);
    pcl::PointXYZRGB invalid = valid;
    invalid.x = std::numeric_limits<float>::quiet_NaN();
    cloud->push_back(invalid);
    const auto output = transform_point_cloud(cloud, Eigen::Matrix4f::Identity());
    QCOMPARE(output->size(), std::size_t(1));
    Eigen::Matrix4f bad = Eigen::Matrix4f::Identity();
    bad(0, 0) = std::numeric_limits<float>::quiet_NaN();
    QVERIFY(transform_point_cloud(cloud, bad)->empty());
  }

  void measurementTransformation()
  {
    MeasurementRecord record = calculate_measurement(MeasurementKind::Segment,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(3.0F, 4.0F, 0.0F)}, 7);
    record.name = QStringLiteral("保留名称");
    record.note = QStringLiteral("备注");
    CloudTransformParameters parameters;
    parameters.translation_x = 5.0;
    parameters.uniform_scale = 2.0;
    const MeasurementRecord transformed = transform_measurement_record(
      record, cloud_transform_matrix(parameters));
    QVERIFY(transformed.valid);
    QCOMPARE(transformed.id, 7);
    QCOMPARE(transformed.name, record.name);
    QCOMPARE(transformed.note, record.note);
    QVERIFY(std::abs(transformed.distance_3d - 10.0) < 1e-6);
    QVERIFY(std::abs(transformed.point_a.x - 5.0F) < 1e-6F);

    const Eigen::Matrix4f scale_two = cloud_transform_matrix(
      CloudTransformParameters{0.0, 0.0, 0.0, 0.0, 2.0});
    const MeasurementRecord area = transform_measurement_record(calculate_measurement(
      MeasurementKind::Area,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F),
       pcl::PointXYZ(1.0F, 1.0F, 0.0F), pcl::PointXYZ(0.0F, 1.0F, 0.0F)}, 8), scale_two);
    QVERIFY(area.valid);
    QVERIFY(std::abs(area.area_planar - 4.0) < 1e-6);
    const MeasurementRecord angle = transform_measurement_record(calculate_measurement(
      MeasurementKind::Angle,
      {pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(),
       pcl::PointXYZ(0.0F, 1.0F, 0.0F)}, 9), scale_two);
    QVERIFY(angle.valid);
    QVERIFY(std::abs(angle.angle_degrees - 90.0) < 1e-6);
    const MeasurementRecord circle = transform_measurement_record(calculate_measurement(
      MeasurementKind::Circle,
      {pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(0.0F, 1.0F, 0.0F),
       pcl::PointXYZ(-1.0F, 0.0F, 0.0F)}, 10), scale_two);
    QVERIFY(circle.valid);
    QVERIFY(std::abs(circle.radius - 2.0) < 1e-6);
  }

  void inverseRoundTrip()
  {
    CloudTransformParameters parameters;
    parameters.translation_x = -4.2;
    parameters.translation_y = 8.1;
    parameters.translation_z = 2.0;
    parameters.rotation_z_degrees = -37.0;
    parameters.uniform_scale = 0.25;
    const Eigen::Matrix4f matrix = cloud_transform_matrix(parameters);
    const Eigen::Vector4f source(8.0F, -1.0F, 5.0F, 1.0F);
    const Eigen::Vector4f recovered = matrix.inverse() * (matrix * source);
    QVERIFY(recovered.isApprox(source, 1e-5F));
  }

  void matrixJsonRoundTrip()
  {
    CloudTransformParameters parameters;
    parameters.translation_x = 1.0;
    parameters.rotation_z_degrees = 450.0;
    parameters.uniform_scale = 1.5;
    const Eigen::Matrix4f matrix = cloud_transform_matrix(parameters);
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("matrix.json"));
    QString error;
    QVERIFY2(write_transform_matrix_json(path, matrix, parameters, &error), qPrintable(error));
    QFile file(path);
    QVERIFY(file.open(QIODevice::ReadOnly));
    const QJsonObject root = QJsonDocument::fromJson(file.readAll()).object();
    QCOMPARE(root.value(QStringLiteral("format")).toString(), QStringLiteral("pcd-transform-matrix"));
    QCOMPARE(root.value(QStringLiteral("matrix")).toArray().size(), 4);
    QCOMPARE(root.value(QStringLiteral("parameters")).toObject()
      .value(QStringLiteral("rotation_z_degrees")).toDouble(), 90.0);
    QVERIFY(!write_transform_matrix_json(directory.path(), matrix, parameters, &error));
    Eigen::Matrix4f invalid = matrix;
    invalid(2, 2) = std::numeric_limits<float>::quiet_NaN();
    QVERIFY(!write_transform_matrix_json(
      directory.filePath(QStringLiteral("invalid.json")), invalid, parameters, &error));
  }
};

QTEST_GUILESS_MAIN(TransformToolsTest)

#include "transform_tools_test.moc"
