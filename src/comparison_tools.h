#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <QString>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct CloudComparisonOptions
{
  bool run_icp = true;
  bool centroid_prealign = true;
  double voxel_size = 0.05;
  int maximum_iterations = 60;
  double maximum_correspondence_distance = 0.5;
  double difference_threshold = 0.05;
  std::size_t maximum_display_points = 1000000;
};

struct DistanceStatistics
{
  std::size_t point_count = 0;
  std::size_t over_threshold_count = 0;
  double minimum = std::numeric_limits<double>::quiet_NaN();
  double mean = std::numeric_limits<double>::quiet_NaN();
  double rmse = std::numeric_limits<double>::quiet_NaN();
  double median = std::numeric_limits<double>::quiet_NaN();
  double p95 = std::numeric_limits<double>::quiet_NaN();
  double maximum = std::numeric_limits<double>::quiet_NaN();
  double over_threshold_ratio = 0.0;
};

struct CloudComparisonResult
{
  bool valid = false;
  QString error;
  bool icp_requested = false;
  bool converged = false;
  double fitness_score = std::numeric_limits<double>::quiet_NaN();
  double difference_threshold = 0.0;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  DistanceStatistics statistics;
  std::vector<double> distances;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr display_heatmap_cloud;
};

bool valid_comparison_options(
  const CloudComparisonOptions & options,
  QString * error = nullptr);

std::array<std::uint8_t, 3> difference_heat_color(
  double distance,
  double threshold);

CloudComparisonResult compare_point_clouds(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & reference,
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & moving,
  const CloudComparisonOptions & options);

bool write_comparison_distances_csv(
  const QString & path,
  const CloudComparisonResult & comparison,
  QString * error = nullptr);

bool write_comparison_summary_json(
  const QString & path,
  const CloudComparisonResult & comparison,
  const QString & reference_path,
  const QString & moving_path,
  QString * error = nullptr);
