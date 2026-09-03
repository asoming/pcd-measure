#include "comparison_tools.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QTextStream>

#include <pcl/common/centroid.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

namespace
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr finite_copy(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & source)
{
  auto result = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (!source) return result;
  result->reserve(source->size());
  for (const pcl::PointXYZRGB & point : *source) {
    if (pcl::isFinite(point)) result->push_back(point);
  }
  result->width = static_cast<std::uint32_t>(result->size());
  result->height = 1;
  result->is_dense = true;
  return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  double leaf_size)
{
  if (leaf_size <= 0.0) {
    return pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*cloud);
  }
  pcl::VoxelGrid<pcl::PointXYZRGB> filter;
  filter.setInputCloud(cloud);
  const float leaf = static_cast<float>(leaf_size);
  filter.setLeafSize(leaf, leaf, leaf);
  auto result = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  filter.filter(*result);
  return result;
}

double quantile(const std::vector<double> & sorted, double probability)
{
  if (sorted.empty()) return std::numeric_limits<double>::quiet_NaN();
  const double position = std::clamp(probability, 0.0, 1.0) *
    static_cast<double>(sorted.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return sorted[lower] * (1.0 - fraction) + sorted[upper] * fraction;
}

std::uint8_t blend(std::uint8_t first, std::uint8_t second, double amount)
{
  return static_cast<std::uint8_t>(std::lround(
    static_cast<double>(first) * (1.0 - amount) + static_cast<double>(second) * amount));
}

QJsonArray matrix_to_json(const Eigen::Matrix4f & matrix)
{
  QJsonArray rows;
  for (int row = 0; row < 4; ++row) {
    QJsonArray values;
    for (int column = 0; column < 4; ++column) values.append(matrix(row, column));
    rows.append(values);
  }
  return rows;
}

}  // namespace

bool valid_comparison_options(const CloudComparisonOptions & options, QString * error)
{
  if (!std::isfinite(options.voxel_size) || options.voxel_size < 0.0) {
    if (error) *error = QStringLiteral("体素边长必须是大于或等于 0 的有限数值。");
    return false;
  }
  if (options.maximum_iterations < 1 || options.maximum_iterations > 100000) {
    if (error) *error = QStringLiteral("ICP 最大迭代次数必须在 1–100000 之间。");
    return false;
  }
  if (!std::isfinite(options.maximum_correspondence_distance) ||
    options.maximum_correspondence_distance <= 0.0)
  {
    if (error) *error = QStringLiteral("最大对应距离必须是正的有限数值。");
    return false;
  }
  if (!std::isfinite(options.difference_threshold) || options.difference_threshold < 0.0) {
    if (error) *error = QStringLiteral("差异阈值必须是大于或等于 0 的有限数值。");
    return false;
  }
  if (options.maximum_display_points == 0) {
    if (error) *error = QStringLiteral("最大显示点数必须大于 0。");
    return false;
  }
  return true;
}

std::array<std::uint8_t, 3> difference_heat_color(double distance, double threshold)
{
  if (!std::isfinite(distance) || distance < 0.0) return {255, 55, 40};
  const double ratio = threshold > 0.0 ? std::clamp(distance / threshold, 0.0, 1.0) :
    (distance <= 1e-12 ? 0.0 : 1.0);
  const std::array<std::uint8_t, 3> blue{30, 105, 255};
  const std::array<std::uint8_t, 3> cyan{20, 220, 210};
  const std::array<std::uint8_t, 3> yellow{255, 205, 35};
  const std::array<std::uint8_t, 3> red{255, 55, 40};
  const auto mix = [](const auto & first, const auto & second, double amount) {
      return std::array<std::uint8_t, 3>{blend(first[0], second[0], amount),
        blend(first[1], second[1], amount), blend(first[2], second[2], amount)};
    };
  if (ratio <= 1.0 / 3.0) return mix(blue, cyan, ratio * 3.0);
  if (ratio <= 2.0 / 3.0) return mix(cyan, yellow, (ratio - 1.0 / 3.0) * 3.0);
  return mix(yellow, red, (ratio - 2.0 / 3.0) * 3.0);
}

CloudComparisonResult compare_point_clouds(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & reference_source,
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & moving_source,
  const CloudComparisonOptions & options)
{
  CloudComparisonResult result;
  result.icp_requested = options.run_icp;
  result.difference_threshold = options.difference_threshold;
  if (!valid_comparison_options(options, &result.error)) return result;
  const auto reference = finite_copy(reference_source);
  const auto moving = finite_copy(moving_source);
  if (reference->size() < 3 || moving->size() < 3) {
    result.error = QStringLiteral("基准点云和第二点云都至少需要 3 个有效点。");
    return result;
  }

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  if (options.run_icp) {
    auto reference_work = downsample(reference, options.voxel_size);
    auto moving_work = downsample(moving, options.voxel_size);
    if (reference_work->size() < 3 || moving_work->size() < 3) {
      result.error = QStringLiteral("体素降采样后有效点不足，请减小体素边长。");
      return result;
    }
    if (options.centroid_prealign) {
      Eigen::Vector4f reference_centroid;
      Eigen::Vector4f moving_centroid;
      pcl::compute3DCentroid(*reference_work, reference_centroid);
      pcl::compute3DCentroid(*moving_work, moving_centroid);
      transform.block<3, 1>(0, 3) =
        reference_centroid.head<3>() - moving_centroid.head<3>();
      pcl::transformPointCloud(*moving_work, *moving_work, transform);
    }

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(moving_work);
    icp.setInputTarget(reference_work);
    icp.setMaximumIterations(options.maximum_iterations);
    icp.setMaxCorrespondenceDistance(options.maximum_correspondence_distance);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-10);
    pcl::PointCloud<pcl::PointXYZRGB> aligned_work;
    icp.align(aligned_work);
    result.converged = icp.hasConverged();
    result.fitness_score = icp.getFitnessScore(options.maximum_correspondence_distance);
    if (!result.converged || !std::isfinite(result.fitness_score)) {
      result.error = QStringLiteral("ICP 未收敛；请检查重叠区域或放宽最大对应距离。");
      return result;
    }
    transform = icp.getFinalTransformation() * transform;
  } else {
    result.converged = true;
    result.fitness_score = 0.0;
  }

  result.transform = transform;
  result.aligned_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::transformPointCloud(*moving, *result.aligned_cloud, transform);
  pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
  tree.setInputCloud(reference);
  result.distances.reserve(result.aligned_cloud->size());
  std::vector<int> nearest_index(1);
  std::vector<float> squared_distance(1);
  std::size_t correspondence_count = 0;
  double sum = 0.0;
  double squared_sum = 0.0;
  for (const pcl::PointXYZRGB & point : *result.aligned_cloud) {
    if (tree.nearestKSearch(point, 1, nearest_index, squared_distance) != 1) {
      result.error = QStringLiteral("最近邻距离计算失败。");
      return result;
    }
    const double distance = std::sqrt(std::max(0.0F, squared_distance[0]));
    result.distances.push_back(distance);
    sum += distance;
    squared_sum += distance * distance;
    if (distance <= options.maximum_correspondence_distance) ++correspondence_count;
  }
  if (options.run_icp) {
    const std::size_t minimum_correspondences = std::max<std::size_t>(
      3, std::min(reference->size(), moving->size()) / 100);
    if (correspondence_count < minimum_correspondences) {
      result.error = QStringLiteral("ICP 虽结束，但有效重叠点不足，结果不可信。");
      return result;
    }
  }

  std::vector<double> sorted = result.distances;
  std::sort(sorted.begin(), sorted.end());
  DistanceStatistics & statistics = result.statistics;
  statistics.point_count = sorted.size();
  statistics.minimum = sorted.front();
  statistics.maximum = sorted.back();
  statistics.mean = sum / static_cast<double>(sorted.size());
  statistics.rmse = std::sqrt(squared_sum / static_cast<double>(sorted.size()));
  statistics.median = quantile(sorted, 0.5);
  statistics.p95 = quantile(sorted, 0.95);
  statistics.over_threshold_count = static_cast<std::size_t>(std::count_if(
    sorted.begin(), sorted.end(), [threshold = options.difference_threshold](double value) {
      const double tolerance = std::max(1e-12, std::abs(threshold) * 1e-6);
      return value > threshold + tolerance;
    }));
  statistics.over_threshold_ratio = static_cast<double>(statistics.over_threshold_count) /
    static_cast<double>(statistics.point_count);

  result.heatmap_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(*result.aligned_cloud);
  for (std::size_t index = 0; index < result.heatmap_cloud->size(); ++index) {
    const auto color = difference_heat_color(result.distances[index], options.difference_threshold);
    (*result.heatmap_cloud)[index].r = color[0];
    (*result.heatmap_cloud)[index].g = color[1];
    (*result.heatmap_cloud)[index].b = color[2];
  }
  if (result.heatmap_cloud->size() <= options.maximum_display_points) {
    result.display_heatmap_cloud = result.heatmap_cloud;
  } else {
    result.display_heatmap_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    const std::size_t stride = static_cast<std::size_t>(std::ceil(
      static_cast<double>(result.heatmap_cloud->size()) / options.maximum_display_points));
    result.display_heatmap_cloud->reserve(options.maximum_display_points);
    for (std::size_t index = 0; index < result.heatmap_cloud->size(); index += stride) {
      result.display_heatmap_cloud->push_back((*result.heatmap_cloud)[index]);
    }
    result.display_heatmap_cloud->width = static_cast<std::uint32_t>(result.display_heatmap_cloud->size());
    result.display_heatmap_cloud->height = 1;
    result.display_heatmap_cloud->is_dense = true;
  }
  result.valid = true;
  result.error.clear();
  return result;
}

bool write_comparison_distances_csv(
  const QString & path, const CloudComparisonResult & comparison, QString * error)
{
  if (!comparison.valid || !comparison.aligned_cloud ||
    comparison.aligned_cloud->size() != comparison.distances.size())
  {
    if (error) *error = QStringLiteral("没有有效的逐点差异结果。");
    return false;
  }
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    if (error) *error = QStringLiteral("无法写入：%1").arg(path);
    return false;
  }
  QTextStream stream(&file);
  stream.setCodec("UTF-8");
  stream << "index,x_m,y_m,z_m,distance_m,over_threshold\n";
  for (std::size_t index = 0; index < comparison.distances.size(); ++index) {
    const pcl::PointXYZRGB & point = (*comparison.aligned_cloud)[index];
    const double distance = comparison.distances[index];
    stream << index << ',' << QString::number(point.x, 'g', 10) << ','
           << QString::number(point.y, 'g', 10) << ',' << QString::number(point.z, 'g', 10)
           << ',' << QString::number(distance, 'g', 15) << ','
           << (distance > comparison.difference_threshold +
             std::max(1e-12, std::abs(comparison.difference_threshold) * 1e-6) ? 1 : 0) << '\n';
  }
  stream.flush();
  if (stream.status() != QTextStream::Ok || !file.commit()) {
    if (error) *error = QStringLiteral("CSV 写入失败：%1").arg(path);
    return false;
  }
  return true;
}

bool write_comparison_summary_json(
  const QString & path,
  const CloudComparisonResult & comparison,
  const QString & reference_path,
  const QString & moving_path,
  QString * error)
{
  if (!comparison.valid) {
    if (error) *error = QStringLiteral("没有有效的点云对比结果。");
    return false;
  }
  const DistanceStatistics & values = comparison.statistics;
  QJsonObject statistics{{QStringLiteral("point_count"), static_cast<qint64>(values.point_count)},
    {QStringLiteral("minimum_m"), values.minimum}, {QStringLiteral("mean_m"), values.mean},
    {QStringLiteral("rmse_m"), values.rmse}, {QStringLiteral("median_m"), values.median},
    {QStringLiteral("p95_m"), values.p95}, {QStringLiteral("maximum_m"), values.maximum},
    {QStringLiteral("over_threshold_count"), static_cast<qint64>(values.over_threshold_count)},
    {QStringLiteral("over_threshold_ratio"), values.over_threshold_ratio}};
  QJsonObject root{{QStringLiteral("format"), QStringLiteral("pcd-cloud-comparison")},
    {QStringLiteral("reference_path"), QFileInfo(reference_path).absoluteFilePath()},
    {QStringLiteral("moving_path"), QFileInfo(moving_path).absoluteFilePath()},
    {QStringLiteral("icp_requested"), comparison.icp_requested},
    {QStringLiteral("converged"), comparison.converged},
    {QStringLiteral("fitness_score"), comparison.fitness_score},
    {QStringLiteral("difference_threshold_m"), comparison.difference_threshold},
    {QStringLiteral("transform"), matrix_to_json(comparison.transform)},
    {QStringLiteral("statistics"), statistics}};
  QSaveFile file(path);
  const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
  if (!file.open(QIODevice::WriteOnly) || file.write(payload) != payload.size() || !file.commit())
  {
    if (error) *error = QStringLiteral("JSON 写入失败：%1").arg(path);
    return false;
  }
  return true;
}
