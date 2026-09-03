#include <cmath>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTest>

#include <pcl/io/pcd_io.h>

#include "cloud_data.h"

class CloudDataTest : public QObject
{
  Q_OBJECT

private:
  QString fixture_directory_ = QStringLiteral(PCD_MEASURE_TEST_FIXTURE_DIR);

  static pcl::PointCloud<pcl::PointXYZRGB> colored_cloud(int count)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (int index = 0; index < count; ++index) {
      pcl::PointXYZRGB point;
      point.x = static_cast<float>(index % 10) * 0.2F;
      point.y = static_cast<float>((index / 10) % 10) * 0.3F;
      point.z = static_cast<float>(index) * 0.01F;
      point.r = static_cast<std::uint8_t>(index % 255);
      point.g = 80;
      point.b = 160;
      cloud.push_back(point);
    }
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
    return cloud;
  }

private slots:
  void fixtureFormatsAndInvalidPoints()
  {
    const CloudLoadResult xyz = load_pcd_and_analyze(
      fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd"));
    QVERIFY2(xyz.ok(), qPrintable(xyz.error));
    QCOMPARE(xyz.metrics.header_points, std::size_t(12));
    QCOMPARE(xyz.metrics.finite_points, std::size_t(12));
    QVERIFY(xyz.encoding.startsWith(QStringLiteral("ascii")));
    QVERIFY(!xyz.metrics.has_rgb);
    QVERIFY(!xyz.metrics.has_rgba);

    const CloudLoadResult rgba = load_pcd_and_analyze(
      fixture_directory_ + QStringLiteral("/rgba_ascii.pcd"));
    QVERIFY2(rgba.ok(), qPrintable(rgba.error));
    QVERIFY(rgba.metrics.has_rgba);
    QCOMPARE(rgba.metrics.non_black_points, std::size_t(12));

    const CloudLoadResult invalid = load_pcd_and_analyze(
      fixture_directory_ + QStringLiteral("/invalid_points_ascii.pcd"));
    QVERIFY2(invalid.ok(), qPrintable(invalid.error));
    QCOMPARE(invalid.metrics.header_points, std::size_t(14));
    QCOMPARE(invalid.metrics.invalid_points, std::size_t(2));
    QVERIFY(invalid.cloud->is_dense);
  }

  void binaryCompressedAndLod()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const auto source = colored_cloud(100);
    const QString binary = directory.filePath(QStringLiteral("binary.pcd"));
    const QString compressed = directory.filePath(QStringLiteral("compressed.pcd"));
    QCOMPARE(pcl::io::savePCDFileBinary(binary.toStdString(), source), 0);
    QCOMPARE(pcl::io::savePCDFileBinaryCompressed(compressed.toStdString(), source), 0);

    const CloudLoadResult binary_result = load_pcd_and_analyze(binary, 1000);
    QVERIFY2(binary_result.ok(), qPrintable(binary_result.error));
    QVERIFY(binary_result.encoding.startsWith(QStringLiteral("binary（")));
    QCOMPARE(binary_result.metrics.finite_points, std::size_t(100));
    QVERIFY(binary_result.metrics.has_rgb);
    QCOMPARE(binary_result.cloud->at(17).g, std::uint8_t(80));

    const CloudLoadResult compressed_result = load_pcd_and_analyze(compressed, 7);
    QVERIFY2(compressed_result.ok(), qPrintable(compressed_result.error));
    QVERIFY(compressed_result.encoding.startsWith(QStringLiteral("binary_compressed")));
    QCOMPARE(compressed_result.metrics.finite_points, std::size_t(100));
    QVERIFY(compressed_result.metrics.display_downsampled);
    QVERIFY(compressed_result.display_cloud->size() <= std::size_t(7));
    QCOMPARE(compressed_result.metrics.displayed_points, compressed_result.display_cloud->size());

    const CloudLoadResult one_display = load_pcd_and_analyze(compressed, 1);
    QVERIFY(one_display.ok());
    QCOMPARE(one_display.display_cloud->size(), std::size_t(1));
    const CloudLoadResult invalid_limit = load_pcd_and_analyze(compressed, 0);
    QVERIFY(!invalid_limit.ok());
    QVERIFY(invalid_limit.error.contains(QStringLiteral("显示点数")));
  }

  void malformedAndDegenerateFiles()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString missing_xyz = directory.filePath(QStringLiteral("missing_xyz.pcd"));
    QFile fields(missing_xyz);
    QVERIFY(fields.open(QIODevice::WriteOnly));
    fields.write("# .PCD v0.7\nVERSION 0.7\nFIELDS intensity\nSIZE 4\nTYPE F\nCOUNT 1\n"
      "WIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n1\n2\n3\n");
    fields.close();
    const CloudLoadResult missing = load_pcd_and_analyze(missing_xyz);
    QVERIFY(!missing.ok());
    QVERIFY(missing.error.contains(QStringLiteral("缺少")));

    const QString corrupt = directory.filePath(QStringLiteral("corrupt.pcd"));
    QFile corrupt_file(corrupt);
    QVERIFY(corrupt_file.open(QIODevice::WriteOnly));
    corrupt_file.write("this is not a pcd\n");
    corrupt_file.close();
    QVERIFY(!load_pcd_and_analyze(corrupt).ok());
    QVERIFY(!load_pcd_and_analyze(directory.filePath(QStringLiteral("missing.pcd"))).ok());

    pcl::PointCloud<pcl::PointXYZRGB> two = colored_cloud(2);
    const QString tiny = directory.filePath(QStringLiteral("tiny.pcd"));
    QCOMPARE(pcl::io::savePCDFileASCII(tiny.toStdString(), two), 0);
    const CloudLoadResult tiny_result = load_pcd_and_analyze(tiny);
    QVERIFY(!tiny_result.ok());
    QVERIFY(tiny_result.error.contains(QStringLiteral("不足 3")));
  }

  void unicodeAndWhitespacePath()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString nested = directory.filePath(QStringLiteral("中文 空格,目录"));
    QVERIFY(QDir().mkpath(nested));
    const QString target = QDir(nested).filePath(QStringLiteral("点 云,测试.pcd"));
    QVERIFY(QFile::copy(fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd"), target));
    const CloudLoadResult result = load_pcd_and_analyze(target);
    QVERIFY2(result.ok(), qPrintable(result.error));
    QCOMPARE(result.metrics.finite_points, std::size_t(12));
    QCOMPARE(QFileInfo(result.path).fileName(), QStringLiteral("点 云,测试.pcd"));

    QString deep = nested;
    for (int index = 0; index < 8; ++index) {
      deep = QDir(deep).filePath(QStringLiteral("很长的点云路径_%1_abcdefghijklmnop").arg(index));
    }
    QVERIFY(QDir().mkpath(deep));
    const QString long_target = QDir(deep).filePath(QStringLiteral("长路径点云.pcd"));
    QVERIFY(QFile::copy(fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd"), long_target));
    const CloudLoadResult long_result = load_pcd_and_analyze(long_target);
    QVERIFY2(long_result.ok(), qPrintable(long_result.error));
    QCOMPARE(long_result.metrics.finite_points, std::size_t(12));
  }

  void intensityAndRobustDimensions()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString intensity_path = directory.filePath(QStringLiteral("intensity.pcd"));
    QFile intensity_file(intensity_path);
    QVERIFY(intensity_file.open(QIODevice::WriteOnly));
    const QByteArray intensity_data =
      "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
      "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
      "WIDTH 4\nHEIGHT 1\nPOINTS 4\nDATA ascii\n"
      "0 0 0 10\n1 0 0 20\n0 1 1 30\n1 1 1 40\n";
    QCOMPARE(intensity_file.write(intensity_data), static_cast<qint64>(intensity_data.size()));
    intensity_file.close();
    const CloudLoadResult intensity = load_pcd_and_analyze(intensity_path);
    QVERIFY2(intensity.ok(), qPrintable(intensity.error));
    QVERIFY(intensity.metrics.has_intensity);
    QVERIFY(!intensity.metrics.has_rgb);
    QCOMPARE(intensity.cloud->front().r, std::uint8_t(205));

    pcl::PointCloud<pcl::PointXYZRGB> rectangle;
    const double angle = 30.0 * std::acos(-1.0) / 180.0;
    for (int ix = 0; ix <= 100; ++ix) {
      const double x = -5.0 + ix * 0.1;
      for (int iy = 0; iy <= 40; ++iy) {
        const double y = -2.0 + iy * 0.1;
        for (int iz = 0; iz <= 3; ++iz) {
          pcl::PointXYZRGB point;
          point.x = static_cast<float>(x * std::cos(angle) - y * std::sin(angle));
          point.y = static_cast<float>(x * std::sin(angle) + y * std::cos(angle));
          point.z = static_cast<float>(iz);
          point.r = 40;
          point.g = 150;
          point.b = 230;
          rectangle.push_back(point);
        }
      }
    }
    pcl::PointXYZRGB outlier;
    outlier.x = 1000.0F;
    outlier.y = -1000.0F;
    outlier.z = 500.0F;
    rectangle.push_back(outlier);
    rectangle.width = static_cast<std::uint32_t>(rectangle.size());
    rectangle.height = 1;
    rectangle.is_dense = true;
    const QString rectangle_path = directory.filePath(QStringLiteral("rotated_with_outlier.pcd"));
    QCOMPARE(pcl::io::savePCDFileBinaryCompressed(rectangle_path.toStdString(), rectangle), 0);
    const CloudLoadResult dimensions = load_pcd_and_analyze(rectangle_path, 500);
    QVERIFY2(dimensions.ok(), qPrintable(dimensions.error));
    QVERIFY(dimensions.metrics.raw_bounds.diagonal() > 1000.0);
    QVERIFY(std::abs(dimensions.metrics.oriented.major_size - 9.9) < 0.2);
    QVERIFY(std::abs(dimensions.metrics.oriented.minor_size - 4.0) < 0.2);
    QVERIFY(std::abs(dimensions.metrics.oriented.height - 3.0) < 0.1);
    QVERIFY(std::abs(dimensions.metrics.oriented.yaw_degrees - 30.0) < 1.0);
    QVERIFY(dimensions.display_cloud->size() <= std::size_t(500));
  }

  void symmetricAndDuplicatePointCloudsStayFinite()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    pcl::PointCloud<pcl::PointXYZRGB> square;
    for (int x = -10; x <= 10; ++x) {
      for (int y = -10; y <= 10; ++y) {
        pcl::PointXYZRGB point;
        point.x = x * 0.1F;
        point.y = y * 0.1F;
        point.z = 0.0F;
        square.push_back(point);
      }
    }
    square.width = static_cast<std::uint32_t>(square.size());
    square.height = 1;
    square.is_dense = true;
    const QString square_path = directory.filePath(QStringLiteral("symmetric.pcd"));
    QCOMPARE(pcl::io::savePCDFileBinary(square_path.toStdString(), square), 0);
    const CloudLoadResult symmetric = load_pcd_and_analyze(square_path);
    QVERIFY2(symmetric.ok(), qPrintable(symmetric.error));
    QVERIFY(std::isfinite(symmetric.metrics.oriented.major_size));
    QVERIFY(std::isfinite(symmetric.metrics.oriented.minor_size));
    QVERIFY(std::isfinite(symmetric.metrics.oriented.yaw_degrees));

    pcl::PointCloud<pcl::PointXYZRGB> duplicates;
    for (int index = 0; index < 4; ++index) {
      pcl::PointXYZRGB point;
      point.x = index == 3 ? 1.0F : 0.0F;
      point.y = index == 2 ? 1.0F : 0.0F;
      point.z = 0.0F;
      duplicates.push_back(point);
    }
    duplicates.width = 4;
    duplicates.height = 1;
    const QString duplicate_path = directory.filePath(QStringLiteral("duplicates.pcd"));
    QCOMPARE(pcl::io::savePCDFileASCII(duplicate_path.toStdString(), duplicates), 0);
    const CloudLoadResult duplicate_result = load_pcd_and_analyze(duplicate_path);
    QVERIFY2(duplicate_result.ok(), qPrintable(duplicate_result.error));
    QVERIFY(std::isfinite(duplicate_result.metrics.oriented.diagonal_3d));
  }

  void jsonIsFiniteAndComplete()
  {
    const CloudLoadResult result = load_pcd_and_analyze(
      fixture_directory_ + QStringLiteral("/xyz_no_color_ascii.pcd"));
    QVERIFY(result.ok());
    const QByteArray json = cloud_result_to_json(result).toUtf8();
    QJsonParseError error;
    const QJsonDocument document = QJsonDocument::fromJson(json, &error);
    QCOMPARE(error.error, QJsonParseError::NoError);
    QCOMPARE(document.object().value(QStringLiteral("points")).toObject()
      .value(QStringLiteral("finite")).toInt(), 12);
    QVERIFY(!json.contains("nan"));
    QVERIFY(!json.contains("inf"));
  }
};

QTEST_GUILESS_MAIN(CloudDataTest)

#include "cloud_data_test.moc"
