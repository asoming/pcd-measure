#pragma once

#include <QString>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "measurement.h"

struct CloudTransformParameters
{
  double translation_x = 0.0;
  double translation_y = 0.0;
  double translation_z = 0.0;
  double rotation_z_degrees = 0.0;
  double uniform_scale = 1.0;
};

bool valid_cloud_transform(
  const CloudTransformParameters & parameters,
  QString * error = nullptr);

double normalized_degrees(double degrees);

Eigen::Matrix4f cloud_transform_matrix(
  const CloudTransformParameters & parameters);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_point_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const Eigen::Matrix4f & transform);

MeasurementRecord transform_measurement_record(
  const MeasurementRecord & record,
  const Eigen::Matrix4f & transform);

bool write_transform_matrix_json(
  const QString & path,
  const Eigen::Matrix4f & transform,
  const CloudTransformParameters & parameters,
  QString * error = nullptr);
