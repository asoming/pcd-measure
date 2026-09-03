#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <vector>

#include <QString>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cloud_data.h"

struct BoxSelection
{
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
  bool inverted = false;
};

struct PolygonSelection
{
  std::vector<pcl::PointXYZ> vertices;
  double min_z = -std::numeric_limits<double>::infinity();
  double max_z = std::numeric_limits<double>::infinity();
  bool inverted = false;
};

struct PlaneFitResult
{
  bool valid = false;
  QString error;
  std::array<double, 3> centroid{0.0, 0.0, 0.0};
  std::array<double, 3> normal{0.0, 0.0, 1.0};
  double d = 0.0;
  double slope_degrees = 0.0;
  double azimuth_degrees = 0.0;
  double rms = 0.0;
  double mean_absolute_residual = 0.0;
  double maximum_absolute_residual = 0.0;
};

struct RegionAnalysisResult
{
  bool valid = false;
  QString error;
  std::size_t point_count = 0;
  Bounds3d bounds;
  OrientedBounds oriented;
  std::array<double, 3> centroid{0.0, 0.0, 0.0};
  double z_mean = 0.0;
  double z_stddev = 0.0;
  double estimated_spacing = std::numeric_limits<double>::quiet_NaN();
  double density_xy = std::numeric_limits<double>::quiet_NaN();
  double density_xyz = std::numeric_limits<double>::quiet_NaN();
  PlaneFitResult plane;
};

struct HistogramBin
{
  double lower = 0.0;
  double upper = 0.0;
  std::size_t count = 0;
};

struct HistogramResult
{
  bool valid = false;
  QString error;
  double minimum = 0.0;
  double maximum = 0.0;
  std::size_t point_count = 0;
  std::vector<HistogramBin> bins;
};

struct ProfileBin
{
  double station_start = 0.0;
  double station_end = 0.0;
  std::size_t point_count = 0;
  double z_min = std::numeric_limits<double>::quiet_NaN();
  double z_mean = std::numeric_limits<double>::quiet_NaN();
  double z_median = std::numeric_limits<double>::quiet_NaN();
  double z_max = std::numeric_limits<double>::quiet_NaN();
};

struct ProfileResult
{
  bool valid = false;
  QString error;
  double path_length = 0.0;
  double corridor_width = 0.0;
  double bin_size = 0.0;
  std::size_t selected_points = 0;
  std::vector<ProfileBin> bins;
};

struct VolumeCell
{
  double center_x = 0.0;
  double center_y = 0.0;
  double mean_z = 0.0;
  std::size_t point_count = 0;
  double signed_volume = 0.0;
};

struct VolumeResult
{
  bool valid = false;
  QString error;
  double base_z = 0.0;
  double cell_size = 0.0;
  std::size_t selected_points = 0;
  std::size_t occupied_cells = 0;
  std::size_t possible_cells = 0;
  double covered_area = 0.0;
  double boundary_area = 0.0;
  double volume_above = 0.0;
  double volume_below = 0.0;
  double net_volume = 0.0;
  std::vector<VolumeCell> cells;
};

struct OutlierFilterResult
{
  bool valid = false;
  QString error;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kept;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed;
};

struct DominantPlaneResult
{
  bool valid = false;
  QString error;
  std::array<double, 4> coefficients{0.0, 0.0, 1.0, 0.0};
  std::size_t source_points = 0;
  std::size_t inlier_points = 0;
  double inlier_ratio = 0.0;
  double rms = 0.0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr remainder;
};

bool valid_box_selection(const BoxSelection & selection, QString * error = nullptr);
bool valid_polygon_selection(const PolygonSelection & selection, QString * error = nullptr);
bool point_in_polygon_xy(double x, double y, const std::vector<pcl::PointXYZ> & polygon);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr select_box_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const BoxSelection & selection,
  QString * error = nullptr);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr select_polygon_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const PolygonSelection & selection,
  QString * error = nullptr);

PlaneFitResult fit_plane_pca(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

RegionAnalysisResult analyze_region(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

HistogramResult build_height_histogram(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  int bin_count);

ProfileResult build_elevation_profile(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const std::vector<pcl::PointXYZ> & path,
  double corridor_width,
  double bin_size);

VolumeResult estimate_grid_volume(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const std::vector<pcl::PointXYZ> & boundary,
  double base_z,
  double cell_size);

OutlierFilterResult filter_statistical_outliers(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  int mean_k,
  double standard_deviation_multiplier);

DominantPlaneResult extract_dominant_plane(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  double distance_threshold,
  int maximum_iterations = 1000);
