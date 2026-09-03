#include "cloud_data.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <vector>

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>

#include <Eigen/Eigenvalues>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/common/point_tests.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
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

QString read_encoding(const QString & path)
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

}  // namespace

double Bounds3d::diagonal() const
{
  return std::sqrt(size_x() * size_x() + size_y() * size_y() + size_z() * size_z());
}

CloudLoadResult load_pcd_and_analyze(const QString & path, std::size_t maximum_display_points)
{
  CloudLoadResult result;
  result.path = QFileInfo(path).absoluteFilePath();
  result.file_bytes = static_cast<std::uint64_t>(QFileInfo(path).size());
  result.encoding = read_encoding(path);

  if (maximum_display_points == 0) {
    result.error = QStringLiteral("最大显示点数必须大于 0。");
    return result;
  }

  if (!QFileInfo::exists(path)) {
    result.error = QStringLiteral("文件不存在：%1").arg(path);
    return result;
  }

  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(path.toStdString(), blob) != 0) {
    result.error = QStringLiteral("无法读取 PCD 文件，请确认文件完整且格式受支持。");
    return result;
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
    result.error = QStringLiteral("PCD 缺少 x、y 或 z 坐标字段。");
    return result;
  }

  result.metrics.header_points = static_cast<std::size_t>(blob.width) * blob.height;
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

QString cloud_result_to_json(const CloudLoadResult & result, bool indented)
{
  QJsonObject root;
  root.insert(QStringLiteral("file"), result.path);
  root.insert(QStringLiteral("file_bytes"), static_cast<qint64>(result.file_bytes));
  root.insert(QStringLiteral("encoding"), result.encoding);
  root.insert(QStringLiteral("fields"), result.fields);
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
