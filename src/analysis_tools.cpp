#include "analysis_tools.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <numeric>
#include <utility>

#include <Eigen/Eigenvalues>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace
{

constexpr double kEpsilon = 1e-12;

double quantile(const std::vector<double> & sorted_values, double probability)
{
  if (sorted_values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double index = std::clamp(probability, 0.0, 1.0) *
    static_cast<double>(sorted_values.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(index));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower);
  return sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction;
}

bool finite_point(const pcl::PointXYZRGB & point)
{
  return pcl::isFinite(point);
}

bool all_points_finite(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud)
{
  return cloud && std::all_of(cloud->begin(), cloud->end(),
    [](const pcl::PointXYZRGB & point) { return finite_point(point); });
}

void finalize_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

bool point_on_segment(
  double x,
  double y,
  const pcl::PointXYZ & first,
  const pcl::PointXYZ & second)
{
  const double dx = static_cast<double>(second.x) - first.x;
  const double dy = static_cast<double>(second.y) - first.y;
  const double cross = (x - first.x) * dy - (y - first.y) * dx;
  const double tolerance = 1e-9 * std::max(1.0, std::hypot(dx, dy));
  if (std::abs(cross) > tolerance) {
    return false;
  }
  const double dot = (x - first.x) * dx + (y - first.y) * dy;
  return dot >= -tolerance && dot <= dx * dx + dy * dy + tolerance;
}

double polygon_signed_area(const std::vector<pcl::PointXYZ> & polygon)
{
  double twice_area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const pcl::PointXYZ & first = polygon[index];
    const pcl::PointXYZ & second = polygon[(index + 1) % polygon.size()];
    twice_area += static_cast<double>(first.x) * second.y -
      static_cast<double>(second.x) * first.y;
  }
  return twice_area * 0.5;
}

int orientation(
  const pcl::PointXYZ & first,
  const pcl::PointXYZ & second,
  const pcl::PointXYZ & third)
{
  const double value = (static_cast<double>(second.y) - first.y) *
      (static_cast<double>(third.x) - second.x) -
    (static_cast<double>(second.x) - first.x) *
      (static_cast<double>(third.y) - second.y);
  if (std::abs(value) <= 1e-10) {
    return 0;
  }
  return value > 0.0 ? 1 : 2;
}

bool segments_intersect(
  const pcl::PointXYZ & a,
  const pcl::PointXYZ & b,
  const pcl::PointXYZ & c,
  const pcl::PointXYZ & d)
{
  const int first = orientation(a, b, c);
  const int second = orientation(a, b, d);
  const int third = orientation(c, d, a);
  const int fourth = orientation(c, d, b);
  if (first != second && third != fourth && first != 0 && second != 0 &&
    third != 0 && fourth != 0)
  {
    return true;
  }
  return (first == 0 && point_on_segment(c.x, c.y, a, b)) ||
    (second == 0 && point_on_segment(d.x, d.y, a, b)) ||
    (third == 0 && point_on_segment(a.x, a.y, c, d)) ||
    (fourth == 0 && point_on_segment(b.x, b.y, c, d));
}

bool self_intersects(const std::vector<pcl::PointXYZ> & polygon)
{
  for (std::size_t first = 0; first < polygon.size(); ++first) {
    const std::size_t first_next = (first + 1) % polygon.size();
    for (std::size_t second = first + 1; second < polygon.size(); ++second) {
      const std::size_t second_next = (second + 1) % polygon.size();
      if (first == second || first_next == second || second_next == first) {
        continue;
      }
      if (segments_intersect(
        polygon[first], polygon[first_next], polygon[second], polygon[second_next]))
      {
        return true;
      }
    }
  }
  return false;
}

double estimate_spacing(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud)
{
  if (!cloud || cloud->size() < 2) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
  tree.setInputCloud(cloud);
  constexpr std::size_t maximum_samples = 2000;
  const std::size_t stride = std::max<std::size_t>(1, cloud->size() / maximum_samples);
  std::vector<double> distances;
  std::vector<int> indices(2);
  std::vector<float> squared_distances(2);
  for (std::size_t index = 0; index < cloud->size(); index += stride) {
    if (tree.nearestKSearch((*cloud)[index], 2, indices, squared_distances) == 2 &&
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

OrientedBounds calculate_oriented_bounds(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const std::array<double, 3> & centroid)
{
  OrientedBounds bounds;
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const pcl::PointXYZRGB & point : *cloud) {
    const Eigen::Vector2d offset(point.x - centroid[0], point.y - centroid[1]);
    covariance += offset * offset.transpose();
  }
  if (cloud->size() > 1) {
    covariance /= static_cast<double>(cloud->size() - 1);
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  Eigen::Vector2d major(1.0, 0.0);
  if (solver.info() == Eigen::Success && solver.eigenvectors().allFinite()) {
    major = solver.eigenvectors().col(1).normalized();
  }
  if (major.x() < 0.0 || (std::abs(major.x()) <= kEpsilon && major.y() < 0.0)) {
    major = -major;
  }
  const Eigen::Vector2d minor(-major.y(), major.x());
  std::vector<double> major_values;
  std::vector<double> minor_values;
  std::vector<double> z_values;
  major_values.reserve(cloud->size());
  minor_values.reserve(cloud->size());
  z_values.reserve(cloud->size());
  for (const pcl::PointXYZRGB & point : *cloud) {
    const Eigen::Vector2d offset(point.x - centroid[0], point.y - centroid[1]);
    major_values.push_back(offset.dot(major));
    minor_values.push_back(offset.dot(minor));
    z_values.push_back(point.z);
  }
  std::sort(major_values.begin(), major_values.end());
  std::sort(minor_values.begin(), minor_values.end());
  std::sort(z_values.begin(), z_values.end());
  const double lower = cloud->size() >= 200 ? 0.005 : 0.0;
  const double upper = cloud->size() >= 200 ? 0.995 : 1.0;
  bounds.major_min = quantile(major_values, lower);
  bounds.major_max = quantile(major_values, upper);
  bounds.minor_min = quantile(minor_values, lower);
  bounds.minor_max = quantile(minor_values, upper);
  bounds.z_min = quantile(z_values, lower);
  bounds.z_max = quantile(z_values, upper);
  bounds.major_size = bounds.major_max - bounds.major_min;
  bounds.minor_size = bounds.minor_max - bounds.minor_min;
  if (bounds.minor_size > bounds.major_size) {
    std::swap(bounds.major_size, bounds.minor_size);
    std::swap(bounds.major_min, bounds.minor_min);
    std::swap(bounds.major_max, bounds.minor_max);
    major = minor;
  }
  bounds.height = bounds.z_max - bounds.z_min;
  bounds.horizontal_diagonal = std::hypot(bounds.major_size, bounds.minor_size);
  bounds.diagonal_3d = std::hypot(bounds.horizontal_diagonal, bounds.height);
  bounds.major_axis_x = major.x();
  bounds.major_axis_y = major.y();
  bounds.minor_axis_x = -major.y();
  bounds.minor_axis_y = major.x();
  bounds.yaw_degrees = std::atan2(major.y(), major.x()) * 180.0 / std::acos(-1.0);
  bounds.center_x = centroid[0];
  bounds.center_y = centroid[1];
  bounds.center_z = (bounds.z_min + bounds.z_max) * 0.5;
  bounds.points_used = cloud->size();
  return bounds;
}

struct ProfilePoint
{
  double station = 0.0;
  double z = 0.0;
};

}  // namespace

bool valid_box_selection(const BoxSelection & selection, QString * error)
{
  const std::array<double, 6> values{
    selection.min_x, selection.max_x, selection.min_y,
    selection.max_y, selection.min_z, selection.max_z};
  if (!std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })) {
    if (error) *error = QStringLiteral("三维框坐标必须是有限数值。");
    return false;
  }
  if (selection.min_x > selection.max_x || selection.min_y > selection.max_y ||
    selection.min_z > selection.max_z)
  {
    if (error) *error = QStringLiteral("三维框每个方向的最小值不能大于最大值。");
    return false;
  }
  if (selection.max_x - selection.min_x <= kEpsilon ||
    selection.max_y - selection.min_y <= kEpsilon ||
    selection.max_z - selection.min_z <= kEpsilon)
  {
    if (error) *error = QStringLiteral("三维框在 X、Y、Z 方向都必须有非零尺寸。");
    return false;
  }
  return true;
}

bool valid_polygon_selection(const PolygonSelection & selection, QString * error)
{
  if (selection.vertices.size() < 3) {
    if (error) *error = QStringLiteral("多边形至少需要 3 个点。");
    return false;
  }
  if (selection.min_z >= selection.max_z || std::isnan(selection.min_z) ||
    std::isnan(selection.max_z))
  {
    if (error) *error = QStringLiteral("多边形高度范围无效。");
    return false;
  }
  for (const pcl::PointXYZ & point : selection.vertices) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      if (error) *error = QStringLiteral("多边形坐标必须是有限数值。");
      return false;
    }
  }
  for (std::size_t index = 0; index < selection.vertices.size(); ++index) {
    const pcl::PointXYZ & first = selection.vertices[index];
    const pcl::PointXYZ & second = selection.vertices[(index + 1) % selection.vertices.size()];
    if (std::hypot(static_cast<double>(second.x) - first.x,
      static_cast<double>(second.y) - first.y) <= kEpsilon)
    {
      if (error) *error = QStringLiteral("多边形存在相邻重复点。");
      return false;
    }
  }
  if (std::abs(polygon_signed_area(selection.vertices)) <= kEpsilon) {
    if (error) *error = QStringLiteral("多边形面积为零或点近似共线。");
    return false;
  }
  if (self_intersects(selection.vertices)) {
    if (error) *error = QStringLiteral("多边形存在自交，请按边界顺序重新选点。");
    return false;
  }
  return true;
}

bool point_in_polygon_xy(double x, double y, const std::vector<pcl::PointXYZ> & polygon)
{
  if (polygon.size() < 3) {
    return false;
  }
  bool inside = false;
  for (std::size_t current = 0, previous = polygon.size() - 1;
    current < polygon.size(); previous = current++)
  {
    if (point_on_segment(x, y, polygon[previous], polygon[current])) {
      return true;
    }
    const double current_y = polygon[current].y;
    const double previous_y = polygon[previous].y;
    const bool crosses = (current_y > y) != (previous_y > y);
    if (crosses) {
      const double intersection_x =
        (static_cast<double>(polygon[previous].x) - polygon[current].x) *
        (y - current_y) / (previous_y - current_y) + polygon[current].x;
      if (x < intersection_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr select_box_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const BoxSelection & selection,
  QString * error)
{
  auto selected = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (!cloud) {
    if (error) *error = QStringLiteral("点云为空。");
    return selected;
  }
  if (!valid_box_selection(selection, error)) {
    return selected;
  }
  selected->reserve(cloud->size());
  for (const pcl::PointXYZRGB & point : *cloud) {
    if (!finite_point(point)) continue;
    const bool in_box = point.x >= selection.min_x && point.x <= selection.max_x &&
      point.y >= selection.min_y && point.y <= selection.max_y &&
      point.z >= selection.min_z && point.z <= selection.max_z;
    if (in_box != selection.inverted) {
      selected->push_back(point);
    }
  }
  finalize_cloud(selected);
  return selected;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr select_polygon_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const PolygonSelection & selection,
  QString * error)
{
  auto selected = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (!cloud) {
    if (error) *error = QStringLiteral("点云为空。");
    return selected;
  }
  if (!valid_polygon_selection(selection, error)) {
    return selected;
  }
  selected->reserve(cloud->size());
  for (const pcl::PointXYZRGB & point : *cloud) {
    if (!finite_point(point)) continue;
    const bool in_height = point.z >= selection.min_z && point.z <= selection.max_z;
    const bool in_polygon = in_height && point_in_polygon_xy(point.x, point.y, selection.vertices);
    if (in_polygon != selection.inverted) {
      selected->push_back(point);
    }
  }
  finalize_cloud(selected);
  return selected;
}

PlaneFitResult fit_plane_pca(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud)
{
  PlaneFitResult result;
  if (!cloud || cloud->size() < 3) {
    result.error = QStringLiteral("平面拟合至少需要 3 个点。");
    return result;
  }
  if (!all_points_finite(cloud)) {
    result.error = QStringLiteral("平面拟合输入包含 NaN 或 Inf 坐标。");
    return result;
  }
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const pcl::PointXYZRGB & point : *cloud) {
    centroid += Eigen::Vector3d(point.x, point.y, point.z);
  }
  centroid /= static_cast<double>(cloud->size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const pcl::PointXYZRGB & point : *cloud) {
    const Eigen::Vector3d offset = Eigen::Vector3d(point.x, point.y, point.z) - centroid;
    covariance += offset * offset.transpose();
  }
  covariance /= static_cast<double>(cloud->size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvectors().allFinite() ||
    solver.eigenvalues()[1] <= kEpsilon)
  {
    result.error = QStringLiteral("点近似共线，无法拟合稳定平面。");
    return result;
  }
  Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
  if (normal.z() < 0.0 || (std::abs(normal.z()) <= kEpsilon && normal.x() < 0.0)) {
    normal = -normal;
  }
  double squared_sum = 0.0;
  double absolute_sum = 0.0;
  double maximum = 0.0;
  for (const pcl::PointXYZRGB & point : *cloud) {
    const double residual = std::abs(
      (Eigen::Vector3d(point.x, point.y, point.z) - centroid).dot(normal));
    squared_sum += residual * residual;
    absolute_sum += residual;
    maximum = std::max(maximum, residual);
  }
  result.valid = true;
  result.centroid = {centroid.x(), centroid.y(), centroid.z()};
  result.normal = {normal.x(), normal.y(), normal.z()};
  result.d = -normal.dot(centroid);
  result.slope_degrees = std::acos(std::clamp(std::abs(normal.z()), 0.0, 1.0)) *
    180.0 / std::acos(-1.0);
  result.azimuth_degrees = std::atan2(normal.y(), normal.x()) * 180.0 / std::acos(-1.0);
  if (result.azimuth_degrees < 0.0) result.azimuth_degrees += 360.0;
  result.rms = std::sqrt(squared_sum / static_cast<double>(cloud->size()));
  result.mean_absolute_residual = absolute_sum / static_cast<double>(cloud->size());
  result.maximum_absolute_residual = maximum;
  return result;
}

RegionAnalysisResult analyze_region(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud)
{
  RegionAnalysisResult result;
  if (!cloud || cloud->size() < 3) {
    result.error = QStringLiteral("区域至少需要 3 个点。");
    return result;
  }
  if (!all_points_finite(cloud)) {
    result.error = QStringLiteral("区域包含 NaN 或 Inf 坐标。");
    return result;
  }
  result.point_count = cloud->size();
  const pcl::PointXYZRGB & first = cloud->front();
  result.bounds = Bounds3d{first.x, first.x, first.y, first.y, first.z, first.z};
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  double sum_z_squared = 0.0;
  for (const pcl::PointXYZRGB & point : *cloud) {
    result.bounds.min_x = std::min(result.bounds.min_x, static_cast<double>(point.x));
    result.bounds.max_x = std::max(result.bounds.max_x, static_cast<double>(point.x));
    result.bounds.min_y = std::min(result.bounds.min_y, static_cast<double>(point.y));
    result.bounds.max_y = std::max(result.bounds.max_y, static_cast<double>(point.y));
    result.bounds.min_z = std::min(result.bounds.min_z, static_cast<double>(point.z));
    result.bounds.max_z = std::max(result.bounds.max_z, static_cast<double>(point.z));
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
    sum_z_squared += static_cast<double>(point.z) * point.z;
  }
  const double count = static_cast<double>(cloud->size());
  result.centroid = {sum_x / count, sum_y / count, sum_z / count};
  result.z_mean = result.centroid[2];
  result.z_stddev = std::sqrt(std::max(0.0, sum_z_squared / count - result.z_mean * result.z_mean));
  result.oriented = calculate_oriented_bounds(cloud, result.centroid);
  result.estimated_spacing = estimate_spacing(cloud);
  const double area = result.bounds.size_x() * result.bounds.size_y();
  const double volume = area * result.bounds.size_z();
  if (area > kEpsilon) result.density_xy = count / area;
  if (volume > kEpsilon) result.density_xyz = count / volume;
  result.plane = fit_plane_pca(cloud);
  result.valid = true;
  return result;
}

HistogramResult build_height_histogram(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  int bin_count)
{
  HistogramResult result;
  if (!cloud || cloud->empty()) {
    result.error = QStringLiteral("点云为空，无法生成直方图。");
    return result;
  }
  if (!all_points_finite(cloud)) {
    result.error = QStringLiteral("点云包含 NaN 或 Inf 坐标。");
    return result;
  }
  if (bin_count < 1 || bin_count > 500) {
    result.error = QStringLiteral("直方图箱数必须在 1–500 之间。");
    return result;
  }
  auto limits = std::minmax_element(cloud->begin(), cloud->end(),
    [](const pcl::PointXYZRGB & first, const pcl::PointXYZRGB & second) {
      return first.z < second.z;
    });
  result.minimum = limits.first->z;
  result.maximum = limits.second->z;
  result.point_count = cloud->size();
  result.bins.resize(static_cast<std::size_t>(bin_count));
  const double span = result.maximum - result.minimum;
  const double width = span > kEpsilon ? span / bin_count : 1.0;
  for (int index = 0; index < bin_count; ++index) {
    HistogramBin & bin = result.bins[static_cast<std::size_t>(index)];
    bin.lower = result.minimum + index * width;
    bin.upper = span > kEpsilon ? result.minimum + (index + 1) * width : result.maximum;
  }
  for (const pcl::PointXYZRGB & point : *cloud) {
    int index = span > kEpsilon ?
      static_cast<int>((point.z - result.minimum) / width) : 0;
    index = std::clamp(index, 0, bin_count - 1);
    ++result.bins[static_cast<std::size_t>(index)].count;
  }
  result.valid = true;
  return result;
}

ProfileResult build_elevation_profile(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const std::vector<pcl::PointXYZ> & path,
  double corridor_width,
  double bin_size)
{
  ProfileResult result;
  result.corridor_width = corridor_width;
  result.bin_size = bin_size;
  if (!cloud || cloud->empty()) {
    result.error = QStringLiteral("点云为空，无法生成剖面。");
    return result;
  }
  if (!all_points_finite(cloud)) {
    result.error = QStringLiteral("点云包含 NaN 或 Inf 坐标。");
    return result;
  }
  if (path.size() < 2) {
    result.error = QStringLiteral("剖面路径至少需要 2 个点。");
    return result;
  }
  if (!std::isfinite(corridor_width) || corridor_width <= 0.0 ||
    !std::isfinite(bin_size) || bin_size <= 0.0)
  {
    result.error = QStringLiteral("剖面宽度和分箱长度必须大于 0。");
    return result;
  }
  for (const pcl::PointXYZ & point : path) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      result.error = QStringLiteral("剖面路径坐标必须是有限数值。");
      return result;
    }
  }
  std::vector<double> segment_starts(path.size() - 1, 0.0);
  std::vector<double> segment_lengths(path.size() - 1, 0.0);
  for (std::size_t index = 1; index < path.size(); ++index) {
    segment_starts[index - 1] = result.path_length;
    const double length = std::hypot(
      static_cast<double>(path[index].x) - path[index - 1].x,
      static_cast<double>(path[index].y) - path[index - 1].y);
    segment_lengths[index - 1] = length;
    result.path_length += length;
  }
  if (result.path_length <= kEpsilon) {
    result.error = QStringLiteral("剖面路径的水平长度必须大于 0。");
    return result;
  }
  const double requested_bins = std::ceil(result.path_length / bin_size);
  if (!std::isfinite(requested_bins) || requested_bins > 100000.0) {
    result.error = QStringLiteral("剖面分箱过密，请增大分箱长度。");
    return result;
  }
  const int bin_count = std::max(1, static_cast<int>(requested_bins));
  std::vector<std::vector<double>> elevations(static_cast<std::size_t>(bin_count));
  const double half_width = corridor_width * 0.5;
  for (const pcl::PointXYZRGB & point : *cloud) {
    double nearest_distance = std::numeric_limits<double>::infinity();
    double nearest_station = 0.0;
    for (std::size_t segment = 0; segment < segment_lengths.size(); ++segment) {
      const double length = segment_lengths[segment];
      if (length <= kEpsilon) continue;
      const double dx = static_cast<double>(path[segment + 1].x) - path[segment].x;
      const double dy = static_cast<double>(path[segment + 1].y) - path[segment].y;
      const double t = std::clamp(
        ((point.x - path[segment].x) * dx + (point.y - path[segment].y) * dy) /
          (length * length), 0.0, 1.0);
      const double projection_x = path[segment].x + t * dx;
      const double projection_y = path[segment].y + t * dy;
      const double distance = std::hypot(point.x - projection_x, point.y - projection_y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_station = segment_starts[segment] + t * length;
      }
    }
    if (nearest_distance <= half_width) {
      const int bin = std::clamp(
        static_cast<int>(nearest_station / bin_size), 0, bin_count - 1);
      elevations[static_cast<std::size_t>(bin)].push_back(point.z);
      ++result.selected_points;
    }
  }
  result.bins.resize(static_cast<std::size_t>(bin_count));
  for (int index = 0; index < bin_count; ++index) {
    ProfileBin & bin = result.bins[static_cast<std::size_t>(index)];
    bin.station_start = index * bin_size;
    bin.station_end = std::min(result.path_length, (index + 1) * bin_size);
    std::vector<double> & values = elevations[static_cast<std::size_t>(index)];
    bin.point_count = values.size();
    if (values.empty()) continue;
    std::sort(values.begin(), values.end());
    bin.z_min = values.front();
    bin.z_max = values.back();
    bin.z_mean = std::accumulate(values.begin(), values.end(), 0.0) /
      static_cast<double>(values.size());
    bin.z_median = quantile(values, 0.5);
  }
  result.valid = true;
  return result;
}

VolumeResult estimate_grid_volume(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const std::vector<pcl::PointXYZ> & boundary,
  double base_z,
  double cell_size)
{
  VolumeResult result;
  result.base_z = base_z;
  result.cell_size = cell_size;
  PolygonSelection selection;
  selection.vertices = boundary;
  if (!cloud || cloud->empty()) {
    result.error = QStringLiteral("点云为空，无法估算体积。");
    return result;
  }
  if (!all_points_finite(cloud)) {
    result.error = QStringLiteral("点云包含 NaN 或 Inf 坐标。");
    return result;
  }
  if (!valid_polygon_selection(selection, &result.error)) return result;
  if (!std::isfinite(base_z) || !std::isfinite(cell_size) || cell_size <= 0.0) {
    result.error = QStringLiteral("基准高程必须有限，格网尺寸必须大于 0。");
    return result;
  }
  double min_x = boundary.front().x;
  double max_x = min_x;
  double min_y = boundary.front().y;
  double max_y = min_y;
  for (const pcl::PointXYZ & point : boundary) {
    min_x = std::min(min_x, static_cast<double>(point.x));
    max_x = std::max(max_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_y = std::max(max_y, static_cast<double>(point.y));
  }
  const double requested_width = std::ceil((max_x - min_x) / cell_size);
  const double requested_height = std::ceil((max_y - min_y) / cell_size);
  if (!std::isfinite(requested_width) || !std::isfinite(requested_height) ||
    requested_width > 5000000.0 || requested_height > 5000000.0 ||
    requested_width * requested_height > 5000000.0)
  {
    result.error = QStringLiteral("体积格网超过 500 万格，请增大格网尺寸。");
    return result;
  }
  const int width = std::max(1, static_cast<int>(requested_width));
  const int height = std::max(1, static_cast<int>(requested_height));
  struct CellAccumulator { double sum_z = 0.0; std::size_t count = 0; };
  std::map<std::pair<int, int>, CellAccumulator> accumulators;
  for (const pcl::PointXYZRGB & point : *cloud) {
    if (!point_in_polygon_xy(point.x, point.y, boundary)) continue;
    int column = static_cast<int>(std::floor((point.x - min_x) / cell_size));
    int row = static_cast<int>(std::floor((point.y - min_y) / cell_size));
    column = std::clamp(column, 0, width - 1);
    row = std::clamp(row, 0, height - 1);
    CellAccumulator & cell = accumulators[{column, row}];
    cell.sum_z += point.z;
    ++cell.count;
    ++result.selected_points;
  }
  const double cell_area = cell_size * cell_size;
  result.boundary_area = std::abs(polygon_signed_area(boundary));
  for (int row = 0; row < height; ++row) {
    for (int column = 0; column < width; ++column) {
      const double center_x = min_x + (column + 0.5) * cell_size;
      const double center_y = min_y + (row + 0.5) * cell_size;
      if (point_in_polygon_xy(center_x, center_y, boundary)) ++result.possible_cells;
    }
  }
  for (const auto & item : accumulators) {
    const int column = item.first.first;
    const int row = item.first.second;
    const CellAccumulator & source = item.second;
    const double mean_z = source.sum_z / static_cast<double>(source.count);
    const double signed_volume = (mean_z - base_z) * cell_area;
    result.cells.push_back(VolumeCell{
      min_x + (column + 0.5) * cell_size,
      min_y + (row + 0.5) * cell_size,
      mean_z,
      source.count,
      signed_volume});
    if (signed_volume >= 0.0) result.volume_above += signed_volume;
    else result.volume_below += -signed_volume;
  }
  result.occupied_cells = result.cells.size();
  result.covered_area = result.occupied_cells * cell_area;
  result.net_volume = result.volume_above - result.volume_below;
  if (result.occupied_cells == 0) {
    result.error = QStringLiteral("边界内没有可用于体积估算的点。");
    return result;
  }
  result.valid = true;
  return result;
}

OutlierFilterResult filter_statistical_outliers(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  int mean_k,
  double standard_deviation_multiplier)
{
  OutlierFilterResult result;
  result.kept = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  result.removed = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (!cloud || cloud->size() < 3) {
    result.error = QStringLiteral("离群点分析至少需要 3 个点。");
    return result;
  }
  if (!all_points_finite(cloud)) {
    result.error = QStringLiteral("点云包含 NaN 或 Inf 坐标。");
    return result;
  }
  if (mean_k < 2 || mean_k >= static_cast<int>(cloud->size())) {
    result.error = QStringLiteral("邻居数必须在 2 和点数减 1 之间。");
    return result;
  }
  if (!std::isfinite(standard_deviation_multiplier) || standard_deviation_multiplier <= 0.0) {
    result.error = QStringLiteral("标准差倍数必须大于 0。");
    return result;
  }
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
  filter.setInputCloud(cloud);
  filter.setMeanK(mean_k);
  filter.setStddevMulThresh(standard_deviation_multiplier);
  filter.filter(*result.kept);
  filter.setNegative(true);
  filter.filter(*result.removed);
  finalize_cloud(result.kept);
  finalize_cloud(result.removed);
  result.valid = true;
  return result;
}

DominantPlaneResult extract_dominant_plane(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  double distance_threshold,
  int maximum_iterations)
{
  DominantPlaneResult result;
  result.inliers = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  result.remainder = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (!cloud || cloud->size() < 3) {
    result.error = QStringLiteral("主平面提取至少需要 3 个点。");
    return result;
  }
  if (!all_points_finite(cloud)) {
    result.error = QStringLiteral("点云包含 NaN 或 Inf 坐标。");
    return result;
  }
  if (!std::isfinite(distance_threshold) || distance_threshold <= 0.0 ||
    maximum_iterations < 1 || maximum_iterations > 100000)
  {
    result.error = QStringLiteral("距离阈值必须大于 0，迭代次数必须在 1–100000 之间。");
    return result;
  }
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(distance_threshold);
  segmentation.setMaxIterations(maximum_iterations);
  segmentation.setInputCloud(cloud);
  auto indices = pcl::make_shared<pcl::PointIndices>();
  auto coefficients = pcl::make_shared<pcl::ModelCoefficients>();
  segmentation.segment(*indices, *coefficients);
  if (indices->indices.size() < 3 || coefficients->values.size() < 4) {
    result.error = QStringLiteral("没有找到满足阈值的主平面。");
    return result;
  }
  const std::size_t minimum_dominant_points = std::max<std::size_t>(
    3, static_cast<std::size_t>(std::ceil(static_cast<double>(cloud->size()) * 0.05)));
  if (indices->indices.size() < minimum_dominant_points) {
    result.error = QStringLiteral("找到的共面点不足总点数的 5%，不能作为主平面。");
    return result;
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*result.inliers);
  extract.setNegative(true);
  extract.filter(*result.remainder);
  finalize_cloud(result.inliers);
  finalize_cloud(result.remainder);
  if (!fit_plane_pca(result.inliers).valid) {
    result.error = QStringLiteral("主平面内点近似共线，结果不稳定。");
    result.inliers->clear();
    result.remainder->clear();
    return result;
  }
  Eigen::Vector4d plane(
    coefficients->values[0], coefficients->values[1],
    coefficients->values[2], coefficients->values[3]);
  const double norm = plane.head<3>().norm();
  if (norm <= kEpsilon) {
    result.error = QStringLiteral("主平面系数无效。");
    return result;
  }
  plane /= norm;
  if (plane.z() < 0.0) plane = -plane;
  double squared_sum = 0.0;
  for (const pcl::PointXYZRGB & point : *result.inliers) {
    const double residual = plane.x() * point.x + plane.y() * point.y +
      plane.z() * point.z + plane.w();
    squared_sum += residual * residual;
  }
  result.valid = true;
  result.coefficients = {plane.x(), plane.y(), plane.z(), plane.w()};
  result.source_points = cloud->size();
  result.inlier_points = result.inliers->size();
  result.inlier_ratio = static_cast<double>(result.inlier_points) /
    static_cast<double>(result.source_points);
  result.rms = std::sqrt(squared_sum / static_cast<double>(result.inlier_points));
  return result;
}
