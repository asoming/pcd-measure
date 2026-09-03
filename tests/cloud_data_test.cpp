#include <cmath>
#include <cstring>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTest>
#include <QtEndian>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

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

  static void append_u32(QByteArray & bytes, std::uint32_t value)
  {
    const quint32 little = qToLittleEndian<quint32>(value);
    bytes.append(reinterpret_cast<const char *>(&little), sizeof(little));
  }

  static void append_float(QByteArray & bytes, float value)
  {
    quint32 bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    append_u32(bytes, bits);
  }

  static void append_double(QByteArray & bytes, double value)
  {
    quint64 bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    const quint64 little = qToLittleEndian<quint64>(bits);
    bytes.append(reinterpret_cast<const char *>(&little), sizeof(little));
  }

  static QByteArray olx_fixture(int point_record_bytes)
  {
    QByteArray bytes;
    const std::array<std::array<float, 3>, 6> points{{
      {{0.0F, 0.0F, 0.0F}}, {{1.0F, 0.0F, 0.1F}}, {{0.0F, 1.0F, 0.2F}},
      {{1.0F, 1.0F, 0.3F}}, {{2.0F, 0.0F, 0.4F}}, {{0.0F, 2.0F, 0.5F}}}};
    for (std::uint32_t frame = 0; frame < 2; ++frame) {
      append_u32(bytes, frame + 7);
      append_double(bytes, 1000.0 + frame * 0.1);
      append_u32(bytes, 3);
      for (std::size_t local = 0; local < 3; ++local) {
        const std::size_t index = frame * 3 + local;
        append_float(bytes, points[index][0]);
        append_float(bytes, points[index][1]);
        append_float(bytes, points[index][2]);
        bytes.append(static_cast<char>(10 + index));
        bytes.append(static_cast<char>(40 + index));
        bytes.append(static_cast<char>(90 + index));
        if (point_record_bytes == 16) bytes.append(static_cast<char>(200 + index));
      }
    }
    return bytes;
  }

  static bool write_bytes(const QString & path, const QByteArray & bytes)
  {
    QFile file(path);
    return file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size();
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

  void plyAndMapFormatDetection()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const auto source = colored_cloud(30);
    const QString ply = directory.filePath(QStringLiteral("彩色 sample.ply"));
    QCOMPARE(pcl::io::savePLYFileBinary(ply.toStdString(), source), 0);
    QCOMPARE(detect_cloud_source_kind(ply), CloudSourceKind::Ply);
    const CloudLoadResult ply_result = load_cloud_and_analyze(ply);
    QVERIFY2(ply_result.ok(), qPrintable(ply_result.error));
    QCOMPARE(ply_result.metrics.finite_points, std::size_t(30));
    QCOMPARE(ply_result.source_format, QStringLiteral("PLY 点云"));

    QByteArray map_header("MAPV0001", 8);
    append_u32(map_header, 5);
    map_header.append(QByteArray(64, '\0'));
    const QString map = directory.filePath(QStringLiteral("sample.bin"));
    QVERIFY(write_bytes(map, map_header));
    QCOMPARE(detect_cloud_source_kind(map), CloudSourceKind::OdinMapBin);

    const bool had_project_dir = qEnvironmentVariableIsSet("PCD_MEASURE_PROJECT_DIR");
    const QByteArray old_project_dir = qgetenv("PCD_MEASURE_PROJECT_DIR");
    qputenv("PCD_MEASURE_PROJECT_DIR", QFile::encodeName(directory.path()));
    const CloudLoadResult map_result = load_cloud_and_analyze(map);
    if (had_project_dir) qputenv("PCD_MEASURE_PROJECT_DIR", old_project_dir);
    else qunsetenv("PCD_MEASURE_PROJECT_DIR");
    QVERIFY(!map_result.ok());
    QVERIFY(map_result.error.contains(QStringLiteral("解码")));

    const QString other_bin = directory.filePath(QStringLiteral("OdinPose.bin"));
    QVERIFY(write_bytes(other_bin, QByteArray(40, '\0')));
    QCOMPARE(detect_cloud_source_kind(other_bin), CloudSourceKind::Unknown);
    QVERIFY(load_cloud_and_analyze(other_bin).error.contains(QStringLiteral("配套文件")));
  }

  void mapConverterFailureAndUnsupportedVersion()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    QByteArray map_header("MAPV0001", 8);
    append_u32(map_header, 5);
    map_header.append(QByteArray(64, '\0'));
    const QString map = directory.filePath(QStringLiteral("map.bin"));
    QVERIFY(write_bytes(map, map_header));

    const QString converter = directory.filePath(QStringLiteral("fake converter.sh"));
    QVERIFY(write_bytes(converter, QByteArray("#!/bin/sh\necho decoder-broken >&2\nexit 7\n")));
    QVERIFY(QFile::setPermissions(converter,
      QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner));
    const bool had_converter = qEnvironmentVariableIsSet("PCD_MEASURE_MAP_TO_PLY");
    const QByteArray old_converter = qgetenv("PCD_MEASURE_MAP_TO_PLY");
    qputenv("PCD_MEASURE_MAP_TO_PLY", QFile::encodeName(converter));
    const CloudLoadResult failed = load_cloud_and_analyze(map);
    if (had_converter) qputenv("PCD_MEASURE_MAP_TO_PLY", old_converter);
    else qunsetenv("PCD_MEASURE_MAP_TO_PLY");
    QVERIFY(!failed.ok());
    QVERIFY(failed.error.contains(QStringLiteral("退出码 7")));
    QVERIFY(failed.error.contains(QStringLiteral("decoder-broken")));

    QByteArray future_header("MAPV0001", 8);
    append_u32(future_header, 6);
    future_header.append(QByteArray(64, '\0'));
    const QString future = directory.filePath(QStringLiteral("future.bin"));
    QVERIFY(write_bytes(future, future_header));
    const CloudLoadResult unsupported = load_cloud_and_analyze(future);
    QVERIFY(!unsupported.ok());
    QVERIFY(unsupported.error.contains(QStringLiteral("v6")));

    const QString short_map = directory.filePath(QStringLiteral("short.bin"));
    QVERIFY(write_bytes(short_map, QByteArray("MAPV0001", 8)));
    QCOMPARE(detect_cloud_source_kind(short_map), CloudSourceKind::Unknown);
  }

  void olxRgbAndRgbaLayoutsWithCompanions()
  {
    for (const int record_bytes : {15, 16}) {
      QTemporaryDir directory;
      QVERIFY(directory.isValid());
      const QString olx = directory.filePath(QStringLiteral("MT 测试_%1.olx").arg(record_bytes));
      QVERIFY(write_bytes(olx, olx_fixture(record_bytes)));

      QVERIFY(write_bytes(directory.filePath(QStringLiteral("OdinPose.bin")), QByteArray(80, '\0')));
      QByteArray image_stream;
      for (std::uint32_t frame = 0; frame < 2; ++frame) {
        append_u32(image_stream, frame);
        append_double(image_stream, 1000.0 + frame * 0.1);
        append_u32(image_stream, 4);
        image_stream.append("TEST", 4);
      }
      QVERIFY(write_bytes(directory.filePath(QStringLiteral("OdinImage.bin")), image_stream));
      QVERIFY(QDir().mkpath(directory.filePath(QStringLiteral("image"))));
      QVERIFY(write_bytes(directory.filePath(QStringLiteral("image/cam_in_ex.txt")),
        QByteArray("calibration\n")));

      QCOMPARE(detect_cloud_source_kind(olx), CloudSourceKind::OdinOlx);
      const CloudLoadResult result = load_cloud_and_analyze(olx, 4);
      QVERIFY2(result.ok(), qPrintable(result.error));
      QCOMPARE(result.olx_point_record_bytes, record_bytes);
      QCOMPARE(result.frames.size(), std::size_t(2));
      QCOMPARE(result.metrics.header_points, std::size_t(6));
      QCOMPARE(result.metrics.finite_points, std::size_t(6));
      QCOMPARE(result.frames.front().id, std::uint32_t(7));
      QCOMPARE(result.frames.back().point_offset, std::size_t(3));
      QCOMPARE(result.cloud->at(4).r, std::uint8_t(14));
      QCOMPARE(result.cloud->at(4).g, std::uint8_t(44));
      QCOMPARE(result.cloud->at(4).b, std::uint8_t(94));
      QCOMPARE(result.pose_frame_count, std::size_t(2));
      QCOMPARE(result.image_frames.size(), std::size_t(2));
      QCOMPARE(result.image_frames.back().payload_size, std::uint32_t(4));
      QVERIFY(result.metrics.display_downsampled);
      QVERIFY(result.display_cloud->size() <= std::size_t(4));
      QVERIFY(result.encoding.contains(QStringLiteral("%1 B/点").arg(record_bytes)));
    }
  }

  void olxRejectsTruncationAndInvalidTimestamp()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    QByteArray truncated = olx_fixture(16);
    truncated.chop(1);
    const QString truncated_path = directory.filePath(QStringLiteral("truncated.olx"));
    QVERIFY(write_bytes(truncated_path, truncated));
    const CloudLoadResult truncated_result = load_cloud_and_analyze(truncated_path);
    QVERIFY(!truncated_result.ok());
    QVERIFY(truncated_result.error.contains(QStringLiteral("无法识别 OLX")));

    QByteArray invalid;
    append_u32(invalid, 0);
    append_double(invalid, std::numeric_limits<double>::quiet_NaN());
    append_u32(invalid, 3);
    invalid.append(QByteArray(3 * 16, '\0'));
    const QString invalid_path = directory.filePath(QStringLiteral("invalid-time.olx"));
    QVERIFY(write_bytes(invalid_path, invalid));
    QVERIFY(!load_cloud_and_analyze(invalid_path).ok());

    const QString empty_path = directory.filePath(QStringLiteral("empty.olx"));
    QVERIFY(write_bytes(empty_path, QByteArray()));
    QVERIFY(load_cloud_and_analyze(empty_path).error.contains(QStringLiteral("过短")));
  }

  void olxZeroFrameInvalidPointAndCompanionWarnings()
  {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    QByteArray bytes;
    append_u32(bytes, 99);
    append_double(bytes, 999.9);
    append_u32(bytes, 0);
    bytes.append(olx_fixture(16));

    const quint32 nan_bits = qToLittleEndian<quint32>(0x7FC00000U);
    std::memcpy(bytes.data() + 16 + 16 + 8, &nan_bits, sizeof(nan_bits));
    const qint64 second_frame_timestamp_offset = 16 + 16 + 3 * 16 + 4;
    double earlier_timestamp = 999.0;
    quint64 earlier_bits = 0;
    std::memcpy(&earlier_bits, &earlier_timestamp, sizeof(earlier_bits));
    earlier_bits = qToLittleEndian<quint64>(earlier_bits);
    std::memcpy(bytes.data() + second_frame_timestamp_offset, &earlier_bits, sizeof(earlier_bits));

    const QString olx = directory.filePath(QStringLiteral("warnings.olx"));
    QVERIFY(write_bytes(olx, bytes));
    QVERIFY(write_bytes(directory.filePath(QStringLiteral("OdinPose.bin")), QByteArray(41, '\0')));
    QByteArray broken_image;
    append_u32(broken_image, 1);
    append_double(broken_image, 1000.0);
    append_u32(broken_image, 100);
    broken_image.append('X');
    QVERIFY(write_bytes(directory.filePath(QStringLiteral("OdinImage.bin")), broken_image));
    QVERIFY(QDir().mkpath(directory.filePath(QStringLiteral("image"))));
    QVERIFY(write_bytes(directory.filePath(QStringLiteral("image/000001.jpg")), QByteArray("not-an-image")));

    const CloudLoadResult result = load_cloud_and_analyze(olx);
    QVERIFY2(result.ok(), qPrintable(result.error));
    QCOMPARE(result.frames.size(), std::size_t(3));
    QCOMPARE(result.frames.front().point_count, std::size_t(0));
    QCOMPARE(result.metrics.header_points, std::size_t(6));
    QCOMPARE(result.metrics.finite_points, std::size_t(5));
    QCOMPARE(result.metrics.invalid_points, std::size_t(1));
    QCOMPARE(result.pose_frame_count, std::size_t(1));
    QCOMPARE(result.image_frames.size(), std::size_t(0));
    QCOMPARE(result.image_paths.size(), 1);
    const QString warnings = result.source_warnings.join(QLatin1Char('\n'));
    QVERIFY(warnings.contains(QStringLiteral("无效坐标")));
    QVERIFY(warnings.contains(QStringLiteral("时间戳倒退")));
    QVERIFY(warnings.contains(QStringLiteral("尾部不完整")));
    QVERIFY(warnings.contains(QStringLiteral("OdinImage.bin")));
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
