#include "cloud_data.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <vector>

#include <QFile>
#include <QFileInfo>
#include <QCoreApplication>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QStandardPaths>
#include <QStringList>
#include <QTemporaryDir>
#include <QtEndian>

#include <Eigen/Eigenvalues>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/common/point_tests.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace
{

constexpr double kRobustLower = 0.005;
constexpr double kRobustUpper = 0.995;
constexpr double kPcaCropLower = 0.001;
constexpr double kPcaCropUpper = 0.999;

double quantile(const std::vector<float> & sorted_values, double probability)
{
  if (sorted_values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double index = probability * static_cast<double>(sorted_values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower);
  return static_cast<double>(sorted_values[lower]) * (1.0 - fraction) +
         static_cast<double>(sorted_values[upper]) * fraction;
}

Bounds3d percentile_bounds(
  const std::vector<float> & xs,
  const std::vector<float> & ys,
  const std::vector<float> & zs,
  double lower,
  double upper)
{
  return Bounds3d{
    quantile(xs, lower), quantile(xs, upper),
    quantile(ys, lower), quantile(ys, upper),
    quantile(zs, lower), quantile(zs, upper)};
}

bool inside(const pcl::PointXYZRGB & point, const Bounds3d & bounds)
{
  return point.x >= bounds.min_x && point.x <= bounds.max_x &&
         point.y >= bounds.min_y && point.y <= bounds.max_y &&
         point.z >= bounds.min_z && point.z <= bounds.max_z;
}

QString read_pcd_encoding(const QString & path)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    return QStringLiteral("未知");
  }

  for (int line_number = 0; line_number < 100 && !file.atEnd(); ++line_number) {
    const QByteArray line = file.readLine().trimmed();
    if (line.toUpper().startsWith("DATA ")) {
      const QString value = QString::fromLatin1(line.mid(5)).trimmed().toLower();
      if (value == QStringLiteral("binary_compressed")) {
        return QStringLiteral("binary_compressed（压缩二进制）");
      }
      if (value == QStringLiteral("binary")) {
        return QStringLiteral("binary（二进制）");
      }
      if (value == QStringLiteral("ascii")) {
        return QStringLiteral("ascii（文本）");
      }
      return value;
    }
  }
  return QStringLiteral("未知");
}

QString read_ply_encoding(const QString & path)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    return QStringLiteral("未知");
  }
  for (int line_number = 0; line_number < 100 && !file.atEnd(); ++line_number) {
    const QByteArray line = file.readLine().trimmed();
    if (!line.toLower().startsWith("format ")) continue;
    if (line.contains("binary_little_endian")) {
      return QStringLiteral("binary_little_endian（二进制小端）");
    }
    if (line.contains("binary_big_endian")) {
      return QStringLiteral("binary_big_endian（二进制大端）");
    }
    if (line.contains("ascii")) {
      return QStringLiteral("ascii（文本）");
    }
    return QString::fromLatin1(line.mid(7));
  }
  return QStringLiteral("未知");
}

double estimate_point_spacing(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
  if (!cloud || cloud->size() < 2) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
  tree.setInputCloud(cloud);

  constexpr std::size_t maximum_samples = 3000;
  const std::size_t stride = std::max<std::size_t>(1, cloud->size() / maximum_samples);
  std::vector<float> distances;
  distances.reserve(std::min(maximum_samples, cloud->size()));
  std::vector<int> indices(2);
  std::vector<float> squared_distances(2);

  for (std::size_t i = 0; i < cloud->size(); i += stride) {
    if (tree.nearestKSearch((*cloud)[i], 2, indices, squared_distances) == 2 &&
      squared_distances[1] > 1e-12F)
    {
      distances.push_back(std::sqrt(squared_distances[1]));
    }
    if (distances.size() >= maximum_samples) {
      break;
    }
  }

  if (distances.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(distances.begin(), distances.end());
  return quantile(distances, 0.5);
}

QJsonArray vector_to_json(double a, double b, double c)
{
  return QJsonArray{a, b, c};
}

QJsonObject bounds_to_json(const Bounds3d & bounds)
{
  QJsonObject object;
  object.insert(QStringLiteral("min_m"), vector_to_json(bounds.min_x, bounds.min_y, bounds.min_z));
  object.insert(QStringLiteral("max_m"), vector_to_json(bounds.max_x, bounds.max_y, bounds.max_z));
  object.insert(QStringLiteral("extent_m"), vector_to_json(bounds.size_x(), bounds.size_y(), bounds.size_z()));
  object.insert(QStringLiteral("diagonal_m"), bounds.diagonal());
  return object;
}

quint32 little_u32(const char * data)
{
  return qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(data));
}

float little_float(const char * data)
{
  const quint32 bits = little_u32(data);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double little_double(const char * data)
{
  const quint64 bits = qFromLittleEndian<quint64>(
    reinterpret_cast<const uchar *>(data));
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool has_odin_map_magic(const QString & path, quint32 * version = nullptr)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) return false;
  const QByteArray header = file.read(12);
  if (header.size() != 12 || header.left(8) != QByteArrayLiteral("MAPV0001")) return false;
  if (version) *version = little_u32(header.constData() + 8);
  return true;
}

QString project_directory()
{
  const QString configured = qEnvironmentVariable("POINT_CLOUD_WORKBENCH_PROJECT_DIR").trimmed();
  if (!configured.isEmpty()) return QFileInfo(configured).absoluteFilePath();

  QDir directory(QCoreApplication::applicationDirPath());
  if (directory.dirName() == QStringLiteral("bin")) {
    directory.cdUp();
    if (directory.dirName() == QStringLiteral("build")) directory.cdUp();
  }
  return directory.absolutePath();
}

QString find_map_converter()
{
  QStringList candidates;
  const QString configured = qEnvironmentVariable("POINT_CLOUD_WORKBENCH_MAP_TO_PLY").trimmed();
  if (!configured.isEmpty()) candidates.append(configured);

  const QString root = project_directory();
  candidates.append(QDir(root).filePath(QStringLiteral("tools/map_to_ply")));
  candidates.append(QDir(root).filePath(QStringLiteral("tools/map_to_ply_amd64")));
  candidates.append(QDir(root).filePath(QStringLiteral("tools/map_to_ply_arm64")));
  const QString from_path = QStandardPaths::findExecutable(QStringLiteral("map_to_ply"));
  if (!from_path.isEmpty()) candidates.append(from_path);

  candidates.removeDuplicates();
  for (const QString & candidate : candidates) {
    const QFileInfo info(candidate);
    if (info.exists() && info.isFile() && info.isExecutable()) {
      return info.absoluteFilePath();
    }
  }
  return QString();
}

bool convert_odin_map(
  const QString & input_path,
  const QString & output_path,
  QString * converter_path,
  QString * error)
{
  const QString executable = find_map_converter();
  if (executable.isEmpty()) {
    if (error) {
      *error = QStringLiteral(
        "检测到 Odin MAPV0001 地图，但尚未安装官方解码器。\n\n"
        "请在项目目录运行：\n./scripts/setup_odin_map_tools.sh\n\n"
        "安装后重新打开该 BIN；也可通过 POINT_CLOUD_WORKBENCH_MAP_TO_PLY 指定解码器路径。");
    }
    return false;
  }

  QProcess process;
  process.setProcessChannelMode(QProcess::SeparateChannels);
  process.start(executable, QStringList{input_path, output_path});
  if (!process.waitForStarted(5000)) {
    if (error) *error = QStringLiteral("无法启动 Odin 地图解码器：%1").arg(executable);
    return false;
  }
  if (!process.waitForFinished(10 * 60 * 1000)) {
    process.kill();
    process.waitForFinished();
    if (error) *error = QStringLiteral("Odin 地图解码超时（10 分钟）。");
    return false;
  }
  if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
    QString details = QString::fromLocal8Bit(process.readAllStandardError()).trimmed();
    if (details.isEmpty()) {
      details = QString::fromLocal8Bit(process.readAllStandardOutput()).trimmed();
    }
    if (error) {
      *error = QStringLiteral("Odin 地图解码失败（退出码 %1）：%2")
        .arg(process.exitCode()).arg(details.isEmpty() ? QStringLiteral("未知错误") : details);
    }
    return false;
  }
  if (!QFileInfo(output_path).isFile() || QFileInfo(output_path).size() <= 0) {
    if (error) *error = QStringLiteral("Odin 地图解码器未生成有效 PLY 文件。");
    return false;
  }
  if (converter_path) *converter_path = executable;
  return true;
}

struct OlxLayoutProbe
{
  bool valid = false;
  int point_record_bytes = 0;
  std::size_t frame_count = 0;
  std::size_t point_count = 0;
  qint64 failure_offset = 0;
};

OlxLayoutProbe probe_olx_layout(QFile & file, int point_record_bytes)
{
  OlxLayoutProbe probe;
  probe.point_record_bytes = point_record_bytes;
  if (!file.seek(0)) return probe;

  constexpr quint32 kMaximumPointsPerFrame = 50000000U;
  while (file.pos() < file.size()) {
    probe.failure_offset = file.pos();
    const QByteArray header = file.read(16);
    if (header.size() != 16) return probe;

    const double timestamp = little_double(header.constData() + 4);
    const quint32 points = little_u32(header.constData() + 12);
    if (!std::isfinite(timestamp) || timestamp < 0.0 || points > kMaximumPointsPerFrame) {
      return probe;
    }
    const quint64 payload_bytes = static_cast<quint64>(points) *
      static_cast<quint64>(point_record_bytes);
    const quint64 remaining_bytes = static_cast<quint64>(file.size() - file.pos());
    if (payload_bytes > remaining_bytes ||
      !file.seek(file.pos() + static_cast<qint64>(payload_bytes)))
    {
      return probe;
    }
    ++probe.frame_count;
    probe.point_count += points;
  }
  probe.valid = file.pos() == file.size() && probe.frame_count > 0 && probe.point_count > 0;
  return probe;
}

void index_olx_images(CloudLoadResult & result)
{
  const QString image_stream = QDir(result.session_root).filePath(QStringLiteral("OdinImage.bin"));
  QFile file(image_stream);
  if (!file.exists() || !file.open(QIODevice::ReadOnly) || file.size() == 0) return;

  constexpr quint32 kMaximumImageBytes = 256U * 1024U * 1024U;
  while (file.pos() < file.size()) {
    const qint64 frame_offset = file.pos();
    const QByteArray header = file.read(16);
    if (header.size() != 16) {
      result.source_warnings.append(
        QStringLiteral("OdinImage.bin 在字节 %1 处截断，已忽略后续图像。")
          .arg(frame_offset));
      break;
    }
    CloudImageFrameInfo frame;
    frame.id = little_u32(header.constData());
    frame.timestamp_seconds = little_double(header.constData() + 4);
    frame.payload_size = little_u32(header.constData() + 12);
    frame.payload_offset = static_cast<std::uint64_t>(file.pos());
    if (!std::isfinite(frame.timestamp_seconds) || frame.timestamp_seconds < 0.0 ||
      frame.payload_size > kMaximumImageBytes ||
      static_cast<quint64>(frame.payload_size) > static_cast<quint64>(file.size() - file.pos()))
    {
      result.source_warnings.append(
        QStringLiteral("OdinImage.bin 在字节 %1 处包含无效帧，已忽略后续图像。")
          .arg(frame_offset));
      break;
    }
    result.image_frames.push_back(frame);
    if (!file.seek(file.pos() + static_cast<qint64>(frame.payload_size))) break;
  }
  if (!result.image_frames.empty()) result.image_stream_path = image_stream;
}

bool load_olx_frames(
  const QString & path,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  CloudLoadResult & result,
  std::size_t & source_point_count)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    result.error = QStringLiteral("无法读取 OLX 文件：%1").arg(path);
    return false;
  }
  if (file.size() < 16) {
    result.error = QStringLiteral("OLX 文件过短，缺少完整帧头。");
    return false;
  }

  const OlxLayoutProbe rgba_probe = probe_olx_layout(file, 16);
  const OlxLayoutProbe rgb_probe = rgba_probe.valid ? OlxLayoutProbe{} : probe_olx_layout(file, 15);
  const OlxLayoutProbe layout = rgba_probe.valid ? rgba_probe : rgb_probe;
  if (!layout.valid) {
    result.error = QStringLiteral(
      "无法识别 OLX 帧布局。文件可能不完整，或不是受支持的 OdinViewer/ROS Driver 录制；"
      "16 字节 RGBA 探测停在 %1，15 字节 RGB 探测停在 %2。")
      .arg(rgba_probe.failure_offset).arg(rgb_probe.failure_offset);
    return false;
  }
  result.olx_point_record_bytes = layout.point_record_bytes;
  if (!file.seek(0)) {
    result.error = QStringLiteral("无法重新定位 OLX 数据流。");
    return false;
  }

  cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  cloud->reserve(std::min<std::size_t>(layout.point_count, 10000000U));

  constexpr quint32 kMaximumPointsPerFrame = 50000000U;
  constexpr quint32 kChunkPoints = 65536U;
  std::size_t invalid_points = 0;
  double first_timestamp = 0.0;
  double last_timestamp = 0.0;
  std::size_t non_monotonic_timestamps = 0;

  while (!file.atEnd()) {
    const qint64 frame_offset = file.pos();
    const QByteArray header = file.read(16);
    if (header.isEmpty()) break;
    if (header.size() != 16) {
      result.error = QStringLiteral("OLX 在字节 %1 处截断：帧头不完整。").arg(frame_offset);
      return false;
    }

    const quint32 frame_id = little_u32(header.constData());
    const double timestamp = little_double(header.constData() + 4);
    const quint32 points = little_u32(header.constData() + 12);
    const quint64 payload_bytes = static_cast<quint64>(points) *
      static_cast<quint64>(layout.point_record_bytes);
    const quint64 remaining_bytes = static_cast<quint64>(file.size() - file.pos());
    if (!std::isfinite(timestamp) || timestamp < 0.0) {
      result.error = QStringLiteral("OLX 在字节 %1 处包含无效时间戳。").arg(frame_offset);
      return false;
    }
    if (points > kMaximumPointsPerFrame || payload_bytes > remaining_bytes) {
      result.error = QStringLiteral(
        "OLX 在字节 %1 处的帧长度无效或数据已截断（声明 %2 点）。")
        .arg(frame_offset).arg(points);
      return false;
    }

    CloudFrameInfo frame;
    frame.id = frame_id;
    frame.timestamp_seconds = timestamp;
    frame.point_offset = cloud->size();
    frame.source_point_count = points;
    source_point_count += points;

    quint32 consumed = 0;
    while (consumed < points) {
      const quint32 chunk_points = std::min(kChunkPoints, points - consumed);
      const qint64 chunk_bytes = static_cast<qint64>(chunk_points) *
        layout.point_record_bytes;
      const QByteArray bytes = file.read(chunk_bytes);
      if (bytes.size() != chunk_bytes) {
        result.error = QStringLiteral("OLX 在帧 %1 的点数据中途截断。").arg(frame_id);
        return false;
      }
      for (quint32 index = 0; index < chunk_points; ++index) {
        const char * record = bytes.constData() + static_cast<std::size_t>(index) *
          static_cast<std::size_t>(layout.point_record_bytes);
        pcl::PointXYZRGB point;
        point.x = little_float(record);
        point.y = little_float(record + 4);
        point.z = little_float(record + 8);
        point.r = static_cast<std::uint8_t>(record[12]);
        point.g = static_cast<std::uint8_t>(record[13]);
        point.b = static_cast<std::uint8_t>(record[14]);
        if (pcl::isFinite(point)) {
          cloud->push_back(point);
        } else {
          ++invalid_points;
        }
      }
      consumed += chunk_points;
    }
    frame.point_count = cloud->size() - frame.point_offset;
    result.frames.push_back(frame);
    if (result.frames.size() == 1) first_timestamp = timestamp;
    if (result.frames.size() > 1 && timestamp < last_timestamp) {
      ++non_monotonic_timestamps;
    }
    last_timestamp = timestamp;
  }

  if (result.frames.empty() || cloud->empty()) {
    result.error = QStringLiteral("OLX 中没有可显示的彩色点云帧。");
    return false;
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;

  const QFileInfo source(path);
  result.session_root = source.absolutePath();
  index_olx_images(result);
  const QDir image_directory(QDir(result.session_root).filePath(QStringLiteral("image")));
  const QFileInfoList images = image_directory.entryInfoList(
    QStringList{QStringLiteral("*.jpg"), QStringLiteral("*.jpeg"), QStringLiteral("*.png")},
    QDir::Files | QDir::Readable, QDir::Name);
  for (const QFileInfo & image : images) result.image_paths.append(image.absoluteFilePath());

  const QFileInfo pose_file(QDir(result.session_root).filePath(QStringLiteral("OdinPose.bin")));
  if (pose_file.isFile() && pose_file.size() > 0) {
    constexpr qint64 kPoseFrameBytes = 4 + 8 + 7 * 4;
    result.pose_frame_count = static_cast<std::size_t>(pose_file.size() / kPoseFrameBytes);
    if (pose_file.size() % kPoseFrameBytes != 0) {
      result.source_warnings.append(QStringLiteral("OdinPose.bin 尾部不完整。"));
    }
  }

  QStringList missing;
  const QDir session(result.session_root);
  if (!session.exists(QStringLiteral("OdinPose.bin"))) missing.append(QStringLiteral("OdinPose.bin"));
  if (!session.exists(QStringLiteral("OdinImage.bin"))) missing.append(QStringLiteral("OdinImage.bin"));
  if (!session.exists(QStringLiteral("image/cam_in_ex.txt"))) {
    missing.append(QStringLiteral("image/cam_in_ex.txt"));
  }
  if (images.isEmpty() && result.image_frames.empty()) {
    missing.append(QStringLiteral("相机帧（OdinImage.bin 或 image/*.jpg）"));
  }
  if (!missing.isEmpty()) {
    result.source_warnings.append(
      QStringLiteral("会话配套数据缺少：%1；点云仍可读取。").arg(missing.join(QStringLiteral("、"))));
  }

  const double duration = std::max(0.0, last_timestamp - first_timestamp);
  const std::size_t image_count = !result.image_frames.empty() ?
    result.image_frames.size() : static_cast<std::size_t>(result.image_paths.size());
  result.source_details = QStringLiteral("%1 帧 · %2 秒 · %3 位姿 · %4 相机帧 · %5 B/点")
    .arg(static_cast<qulonglong>(result.frames.size()))
    .arg(duration, 0, 'f', 2)
    .arg(static_cast<qulonglong>(result.pose_frame_count))
    .arg(static_cast<qulonglong>(image_count))
    .arg(layout.point_record_bytes);
  if (invalid_points > 0) {
    result.source_warnings.append(QStringLiteral("已跳过 %1 个无效坐标点。").arg(invalid_points));
  }
  if (non_monotonic_timestamps > 0) {
    result.source_warnings.append(
      QStringLiteral("检测到 %1 次点云时间戳倒退。").arg(non_monotonic_timestamps));
  }
  return true;
}

}  // namespace

double Bounds3d::diagonal() const
{
  return std::sqrt(size_x() * size_x() + size_y() * size_y() + size_z() * size_z());
}

CloudSourceKind detect_cloud_source_kind(const QString & path)
{
  const QFileInfo info(path);
  if (!info.exists() || !info.isFile()) return CloudSourceKind::Unknown;
  const QString suffix = info.suffix().toLower();
  if (suffix == QStringLiteral("pcd")) return CloudSourceKind::Pcd;
  if (suffix == QStringLiteral("ply")) return CloudSourceKind::Ply;
  if (suffix == QStringLiteral("olx")) return CloudSourceKind::OdinOlx;
  if (suffix == QStringLiteral("bin") && has_odin_map_magic(path)) {
    return CloudSourceKind::OdinMapBin;
  }
  return CloudSourceKind::Unknown;
}

QString cloud_source_kind_label(CloudSourceKind kind)
{
  switch (kind) {
    case CloudSourceKind::Pcd: return QStringLiteral("PCD 点云");
    case CloudSourceKind::Ply: return QStringLiteral("PLY 点云");
    case CloudSourceKind::OdinMapBin: return QStringLiteral("Odin SLAM 地图 BIN");
    case CloudSourceKind::OdinOlx: return QStringLiteral("OdinViewer OLX 录制");
    case CloudSourceKind::Unknown: break;
  }
  return QStringLiteral("未知格式");
}

bool is_supported_cloud_file(const QString & path)
{
  return detect_cloud_source_kind(path) != CloudSourceKind::Unknown;
}

CloudLoadResult load_cloud_and_analyze(const QString & path, std::size_t maximum_display_points)
{
  CloudLoadResult result;
  result.path = QFileInfo(path).absoluteFilePath();
  result.file_bytes = static_cast<std::uint64_t>(QFileInfo(path).size());

  if (maximum_display_points == 0) {
    result.error = QStringLiteral("最大显示点数必须大于 0。");
    return result;
  }

  if (!QFileInfo::exists(path)) {
    result.error = QStringLiteral("文件不存在：%1").arg(path);
    return result;
  }

  result.source_kind = detect_cloud_source_kind(path);
  result.source_format = cloud_source_kind_label(result.source_kind);
  pcl::PCLPointCloud2 blob;
  std::size_t source_header_points = 0;
  switch (result.source_kind) {
    case CloudSourceKind::Pcd:
      result.encoding = read_pcd_encoding(path);
      result.decoder = QStringLiteral("PCL PCD");
      if (pcl::io::loadPCDFile(path.toStdString(), blob) != 0) {
        result.error = QStringLiteral("无法读取 PCD 文件，请确认文件完整且格式受支持。");
        return result;
      }
      break;
    case CloudSourceKind::Ply:
      result.encoding = read_ply_encoding(path);
      result.decoder = QStringLiteral("PCL PLY");
      if (pcl::io::loadPLYFile(path.toStdString(), blob) != 0) {
        result.error = QStringLiteral("无法读取 PLY 文件，请确认文件完整且格式受支持。");
        return result;
      }
      break;
    case CloudSourceKind::OdinMapBin: {
      quint32 version = 0;
      has_odin_map_magic(path, &version);
      if (version < 1 || version > 5) {
        result.error = QStringLiteral("当前官方解码器不支持 MAPV0001 v%1（支持 v1–v5）。")
          .arg(version);
        return result;
      }
      QTemporaryDir temporary(QDir::tempPath() + QStringLiteral("/point-cloud-map-XXXXXX"));
      if (!temporary.isValid()) {
        result.error = QStringLiteral("无法创建 Odin 地图转换临时目录。");
        return result;
      }
      const QString ply_path = temporary.filePath(QStringLiteral("converted.ply"));
      QString converter;
      if (!convert_odin_map(path, ply_path, &converter, &result.error)) return result;
      if (pcl::io::loadPLYFile(ply_path.toStdString(), blob) != 0) {
        result.error = QStringLiteral("官方解码器已运行，但生成的 PLY 无法读取。");
        return result;
      }
      result.encoding = QStringLiteral("MAPV0001 v%1 → binary_little_endian PLY").arg(version);
      result.decoder = QStringLiteral("官方 map_to_ply v1.1.0");
      result.source_details = QStringLiteral("地图版本 v%1 · XYZ 地图点 · 原始 BIN 保持只读")
        .arg(version);
      break;
    }
    case CloudSourceKind::OdinOlx: {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr olx_cloud;
      if (!load_olx_frames(path, olx_cloud, result, source_header_points)) return result;
      pcl::toPCLPointCloud2(*olx_cloud, blob);
      result.encoding = result.olx_point_record_bytes == 16 ?
        QStringLiteral("little-endian · 帧头 16 B · XYZRGBA 16 B/点") :
        QStringLiteral("little-endian · 帧头 16 B · XYZRGB 15 B/点");
      result.decoder = QStringLiteral("内置 OLX 多帧解析器");
      break;
    }
    case CloudSourceKind::Unknown: {
      const QString suffix = QFileInfo(path).suffix().toLower();
      if (suffix == QStringLiteral("bin")) {
        result.error = QStringLiteral(
          "该 BIN 不是 Odin MAPV0001 地图。OdinPose.bin 和 OdinImage.bin 是 OLX 配套文件，"
          "请打开同目录的 .olx 主文件。");
      } else {
        result.error = QStringLiteral("不支持该点云格式；当前支持 PCD、PLY、Odin MAPV0001 BIN 和 OLX。");
      }
      return result;
    }
  }

  QStringList field_names;
  for (const auto & field : blob.fields) {
    field_names.append(QString::fromStdString(field.name));
  }
  result.fields = field_names.join(QStringLiteral(", "));
  result.metrics.has_rgb = field_names.contains(QStringLiteral("rgb"), Qt::CaseInsensitive);
  result.metrics.has_rgba = field_names.contains(QStringLiteral("rgba"), Qt::CaseInsensitive);
  result.metrics.has_intensity = field_names.contains(QStringLiteral("intensity"), Qt::CaseInsensitive);

  if (!field_names.contains(QStringLiteral("x"), Qt::CaseInsensitive) ||
    !field_names.contains(QStringLiteral("y"), Qt::CaseInsensitive) ||
    !field_names.contains(QStringLiteral("z"), Qt::CaseInsensitive))
  {
    result.error = QStringLiteral("点云缺少 x、y 或 z 坐标字段。");
    return result;
  }

  result.metrics.header_points = source_header_points > 0 ? source_header_points :
    static_cast<std::size_t>(blob.width) * blob.height;
  auto converted = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  if (result.metrics.has_rgb) {
    pcl::fromPCLPointCloud2(blob, *converted);
  } else if (result.metrics.has_rgba) {
    pcl::PointCloud<pcl::PointXYZRGBA> rgba_cloud;
    pcl::fromPCLPointCloud2(blob, rgba_cloud);
    converted->reserve(rgba_cloud.size());
    for (const auto & source : rgba_cloud) {
      pcl::PointXYZRGB target;
      target.x = source.x;
      target.y = source.y;
      target.z = source.z;
      target.r = source.r;
      target.g = source.g;
      target.b = source.b;
      converted->push_back(target);
    }
  } else {
    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
    pcl::fromPCLPointCloud2(blob, xyz_cloud);
    converted->reserve(xyz_cloud.size());
    for (const auto & source : xyz_cloud) {
      pcl::PointXYZRGB target;
      target.x = source.x;
      target.y = source.y;
      target.z = source.z;
      target.r = 205;
      target.g = 215;
      target.b = 225;
      converted->push_back(target);
    }
  }

  result.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  result.cloud->reserve(converted->size());

  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
  xs.reserve(converted->size());
  ys.reserve(converted->size());
  zs.reserve(converted->size());

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  for (const auto & point : *converted) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    if (result.cloud->empty()) {
      result.metrics.lowest_point = {point.x, point.y, point.z};
      result.metrics.highest_point = {point.x, point.y, point.z};
    } else {
      if (point.z < result.metrics.lowest_point[2]) {
        result.metrics.lowest_point = {point.x, point.y, point.z};
      }
      if (point.z > result.metrics.highest_point[2]) {
        result.metrics.highest_point = {point.x, point.y, point.z};
      }
    }
    result.cloud->push_back(point);
    xs.push_back(point.x);
    ys.push_back(point.y);
    zs.push_back(point.z);
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
    if ((result.metrics.has_rgb || result.metrics.has_rgba) &&
      (point.r != 0 || point.g != 0 || point.b != 0))
    {
      ++result.metrics.non_black_points;
    }
  }

  result.cloud->width = static_cast<std::uint32_t>(result.cloud->size());
  result.cloud->height = 1;
  result.cloud->is_dense = true;
  result.metrics.finite_points = result.cloud->size();
  result.metrics.invalid_points = result.metrics.header_points >= result.metrics.finite_points ?
    result.metrics.header_points - result.metrics.finite_points : 0;

  if (result.cloud->size() < 3) {
    result.error = QStringLiteral("有效点不足 3 个，无法分析。");
    return result;
  }

  const double count = static_cast<double>(result.cloud->size());
  result.metrics.centroid = {sum_x / count, sum_y / count, sum_z / count};

  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());
  std::sort(zs.begin(), zs.end());
  result.metrics.raw_bounds = percentile_bounds(xs, ys, zs, 0.0, 1.0);
  result.metrics.robust_axis_bounds = percentile_bounds(
    xs, ys, zs, kRobustLower, kRobustUpper);
  const Bounds3d pca_crop = result.cloud->size() >= 200 ?
    percentile_bounds(xs, ys, zs, kPcaCropLower, kPcaCropUpper) :
    result.metrics.raw_bounds;

  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  std::size_t pca_count = 0;
  for (const auto & point : *result.cloud) {
    if (!inside(point, pca_crop)) {
      continue;
    }
    mean += Eigen::Vector2d(point.x, point.y);
    ++pca_count;
  }

  if (pca_count < 3) {
    result.error = QStringLiteral("用于主方向分析的有效点不足。");
    return result;
  }
  mean /= static_cast<double>(pca_count);

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto & point : *result.cloud) {
    if (!inside(point, pca_crop)) {
      continue;
    }
    const Eigen::Vector2d offset = Eigen::Vector2d(point.x, point.y) - mean;
    covariance += offset * offset.transpose();
  }
  covariance /= static_cast<double>(pca_count - 1);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    result.error = QStringLiteral("主方向计算失败。");
    return result;
  }

  Eigen::Vector2d major_axis = solver.eigenvectors().col(1).normalized();
  if (major_axis.x() < 0.0) {
    major_axis = -major_axis;
  }
  const Eigen::Vector2d minor_axis(-major_axis.y(), major_axis.x());

  std::vector<float> major_values;
  std::vector<float> minor_values;
  std::vector<float> pca_zs;
  major_values.reserve(pca_count);
  minor_values.reserve(pca_count);
  pca_zs.reserve(pca_count);
  for (const auto & point : *result.cloud) {
    if (!inside(point, pca_crop)) {
      continue;
    }
    const Eigen::Vector2d offset = Eigen::Vector2d(point.x, point.y) - mean;
    major_values.push_back(static_cast<float>(offset.dot(major_axis)));
    minor_values.push_back(static_cast<float>(offset.dot(minor_axis)));
    pca_zs.push_back(point.z);
  }

  std::sort(major_values.begin(), major_values.end());
  std::sort(minor_values.begin(), minor_values.end());
  std::sort(pca_zs.begin(), pca_zs.end());

  OrientedBounds & oriented = result.metrics.oriented;
  oriented.major_min = quantile(major_values, kRobustLower);
  oriented.major_max = quantile(major_values, kRobustUpper);
  oriented.minor_min = quantile(minor_values, kRobustLower);
  oriented.minor_max = quantile(minor_values, kRobustUpper);
  oriented.z_min = quantile(pca_zs, kRobustLower);
  oriented.z_max = quantile(pca_zs, kRobustUpper);
  oriented.major_size = oriented.major_max - oriented.major_min;
  oriented.minor_size = oriented.minor_max - oriented.minor_min;
  oriented.height = oriented.z_max - oriented.z_min;
  oriented.horizontal_diagonal = std::hypot(oriented.major_size, oriented.minor_size);
  oriented.diagonal_3d = std::hypot(oriented.horizontal_diagonal, oriented.height);
  oriented.major_axis_x = major_axis.x();
  oriented.major_axis_y = major_axis.y();
  oriented.minor_axis_x = minor_axis.x();
  oriented.minor_axis_y = minor_axis.y();
  oriented.yaw_degrees = std::atan2(major_axis.y(), major_axis.x()) * 180.0 / std::acos(-1.0);
  oriented.points_used = pca_count;

  const double projected_center_major = (oriented.major_min + oriented.major_max) / 2.0;
  const double projected_center_minor = (oriented.minor_min + oriented.minor_max) / 2.0;
  const Eigen::Vector2d center = mean + projected_center_major * major_axis +
    projected_center_minor * minor_axis;
  oriented.center_x = center.x();
  oriented.center_y = center.y();
  oriented.center_z = (oriented.z_min + oriented.z_max) / 2.0;

  result.metrics.estimated_spacing = estimate_point_spacing(result.cloud);

  if (result.cloud->size() <= maximum_display_points) {
    result.display_cloud = result.cloud;
  } else {
    result.display_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    const std::size_t stride = static_cast<std::size_t>(
      std::ceil(static_cast<double>(result.cloud->size()) /
      static_cast<double>(maximum_display_points)));
    result.display_cloud->reserve(maximum_display_points);
    for (std::size_t i = 0; i < result.cloud->size(); i += stride) {
      result.display_cloud->push_back((*result.cloud)[i]);
    }
    result.display_cloud->width = static_cast<std::uint32_t>(result.display_cloud->size());
    result.display_cloud->height = 1;
    result.display_cloud->is_dense = true;
    result.metrics.display_downsampled = true;
  }
  result.metrics.displayed_points = result.display_cloud->size();
  return result;
}

CloudLoadResult load_pcd_and_analyze(const QString & path, std::size_t maximum_display_points)
{
  return load_cloud_and_analyze(path, maximum_display_points);
}

QString cloud_result_to_json(const CloudLoadResult & result, bool indented)
{
  QJsonObject root;
  root.insert(QStringLiteral("file"), result.path);
  root.insert(QStringLiteral("file_bytes"), static_cast<qint64>(result.file_bytes));
  root.insert(QStringLiteral("source_format"), result.source_format);
  root.insert(QStringLiteral("source_details"), result.source_details);
  root.insert(QStringLiteral("decoder"), result.decoder);
  root.insert(QStringLiteral("encoding"), result.encoding);
  root.insert(QStringLiteral("fields"), result.fields);
  if (!result.source_warnings.isEmpty()) {
    QJsonArray warnings;
    for (const QString & warning : result.source_warnings) warnings.append(warning);
    root.insert(QStringLiteral("warnings"), warnings);
  }
  if (!result.frames.empty()) {
    QJsonObject sequence;
    sequence.insert(QStringLiteral("frames"), static_cast<qint64>(result.frames.size()));
    sequence.insert(QStringLiteral("images"), static_cast<qint64>(
      !result.image_frames.empty() ? result.image_frames.size() :
      static_cast<std::size_t>(result.image_paths.size())));
    sequence.insert(QStringLiteral("poses"), static_cast<qint64>(result.pose_frame_count));
    sequence.insert(QStringLiteral("point_record_bytes"), result.olx_point_record_bytes);
    sequence.insert(QStringLiteral("first_timestamp_seconds"),
      result.frames.front().timestamp_seconds);
    sequence.insert(QStringLiteral("last_timestamp_seconds"),
      result.frames.back().timestamp_seconds);
    sequence.insert(QStringLiteral("duration_seconds"), std::max(0.0,
      result.frames.back().timestamp_seconds - result.frames.front().timestamp_seconds));
    root.insert(QStringLiteral("sequence"), sequence);
  }
  if (!result.error.isEmpty()) {
    root.insert(QStringLiteral("error"), result.error);
  }

  const CloudMetrics & metrics = result.metrics;
  QJsonObject points;
  points.insert(QStringLiteral("header"), static_cast<qint64>(metrics.header_points));
  points.insert(QStringLiteral("finite"), static_cast<qint64>(metrics.finite_points));
  points.insert(QStringLiteral("invalid"), static_cast<qint64>(metrics.invalid_points));
  points.insert(QStringLiteral("non_black"), static_cast<qint64>(metrics.non_black_points));
  points.insert(QStringLiteral("displayed"), static_cast<qint64>(metrics.displayed_points));
  points.insert(QStringLiteral("display_downsampled"), metrics.display_downsampled);
  root.insert(QStringLiteral("points"), points);
  root.insert(QStringLiteral("centroid_m"), vector_to_json(
    metrics.centroid[0], metrics.centroid[1], metrics.centroid[2]));
  root.insert(QStringLiteral("lowest_point_m"), vector_to_json(
    metrics.lowest_point[0], metrics.lowest_point[1], metrics.lowest_point[2]));
  root.insert(QStringLiteral("highest_point_m"), vector_to_json(
    metrics.highest_point[0], metrics.highest_point[1], metrics.highest_point[2]));
  root.insert(QStringLiteral("raw_height_difference_m"), metrics.raw_bounds.size_z());
  root.insert(QStringLiteral("estimated_spacing_m"), metrics.estimated_spacing);
  root.insert(QStringLiteral("raw_aabb"), bounds_to_json(metrics.raw_bounds));
  root.insert(QStringLiteral("robust_axis_aabb_p0_5_p99_5"), bounds_to_json(metrics.robust_axis_bounds));

  const OrientedBounds & oriented = metrics.oriented;
  QJsonObject oriented_json;
  oriented_json.insert(QStringLiteral("extent_major_minor_height_m"), vector_to_json(
    oriented.major_size, oriented.minor_size, oriented.height));
  oriented_json.insert(QStringLiteral("horizontal_diagonal_m"), oriented.horizontal_diagonal);
  oriented_json.insert(QStringLiteral("diagonal_3d_m"), oriented.diagonal_3d);
  oriented_json.insert(QStringLiteral("yaw_deg_from_x"), oriented.yaw_degrees);
  oriented_json.insert(QStringLiteral("center_m"), vector_to_json(
    oriented.center_x, oriented.center_y, oriented.center_z));
  oriented_json.insert(QStringLiteral("points_used"), static_cast<qint64>(oriented.points_used));
  oriented_json.insert(QStringLiteral("trim_per_side_percent"), kRobustLower * 100.0);
  oriented_json.insert(QStringLiteral("trim_total_per_axis_percent"),
    (kRobustLower + (1.0 - kRobustUpper)) * 100.0);
  const double excluded_percent = metrics.finite_points > 0 ?
    100.0 * static_cast<double>(metrics.finite_points - oriented.points_used) /
      static_cast<double>(metrics.finite_points) : 0.0;
  oriented_json.insert(QStringLiteral("pca_input_excluded_percent"), excluded_percent);
  root.insert(QStringLiteral("robust_oriented_box"), oriented_json);

  return QString::fromUtf8(QJsonDocument(root).toJson(
    indented ? QJsonDocument::Indented : QJsonDocument::Compact));
}
