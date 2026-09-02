#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <QString>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct Bounds3d
{
  double min_x = std::numeric_limits<double>::quiet_NaN();
  double max_x = std::numeric_limits<double>::quiet_NaN();
  double min_y = std::numeric_limits<double>::quiet_NaN();
  double max_y = std::numeric_limits<double>::quiet_NaN();
  double min_z = std::numeric_limits<double>::quiet_NaN();
  double max_z = std::numeric_limits<double>::quiet_NaN();

  double size_x() const { return max_x - min_x; }
  double size_y() const { return max_y - min_y; }
  double size_z() const { return max_z - min_z; }
  double diagonal() const;
};

struct OrientedBounds
{
  double major_size = std::numeric_limits<double>::quiet_NaN();
  double minor_size = std::numeric_limits<double>::quiet_NaN();
  double height = std::numeric_limits<double>::quiet_NaN();
  double horizontal_diagonal = std::numeric_limits<double>::quiet_NaN();
  double diagonal_3d = std::numeric_limits<double>::quiet_NaN();
  double yaw_degrees = std::numeric_limits<double>::quiet_NaN();

  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;
  double major_axis_x = 1.0;
  double major_axis_y = 0.0;
  double minor_axis_x = 0.0;
  double minor_axis_y = 1.0;
  double major_min = 0.0;
  double major_max = 0.0;
  double minor_min = 0.0;
  double minor_max = 0.0;
  double z_min = 0.0;
  double z_max = 0.0;
  std::size_t points_used = 0;
};

struct CloudMetrics
{
  std::size_t header_points = 0;
  std::size_t finite_points = 0;
  std::size_t invalid_points = 0;
  std::size_t non_black_points = 0;
  std::size_t displayed_points = 0;
  bool has_rgb = false;
  bool has_rgba = false;
  bool has_intensity = false;
  bool display_downsampled = false;

  std::array<double, 3> centroid{0.0, 0.0, 0.0};
  std::array<double, 3> lowest_point{0.0, 0.0, 0.0};
  std::array<double, 3> highest_point{0.0, 0.0, 0.0};
  double estimated_spacing = std::numeric_limits<double>::quiet_NaN();
  Bounds3d raw_bounds;
  Bounds3d robust_axis_bounds;
  OrientedBounds oriented;
};

struct CloudLoadResult
{
  QString path;
  QString error;
  QString fields;
  QString encoding;
  std::uint64_t file_bytes = 0;
  CloudMetrics metrics;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_cloud;

  bool ok() const { return error.isEmpty() && cloud && !cloud->empty(); }
};

CloudLoadResult load_pcd_and_analyze(
  const QString & path,
  std::size_t maximum_display_points = 2500000);

QString cloud_result_to_json(const CloudLoadResult & result, bool indented = true);
