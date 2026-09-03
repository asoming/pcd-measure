#pragma once

#include <array>
#include <vector>

#include <QJsonObject>
#include <QString>

#include <pcl/point_types.h>

enum class MeasurementKind
{
  Point,
  Segment,
  Polyline,
  Angle,
  Area,
  Orthogonal,
  PointToPlane,
  Circle
};

struct MeasurementRecord
{
  int id = 0;
  MeasurementKind kind = MeasurementKind::Segment;
  QString name;
  QString group = QStringLiteral("默认");
  QString note;
  QString color_hex = QStringLiteral("#FFB547");
  QString created_at;
  bool visible = true;
  bool valid = false;
  QString error;

  std::vector<pcl::PointXYZ> vertices;
  pcl::PointXYZ point_a;
  pcl::PointXYZ point_b;
  std::array<double, 3> center{0.0, 0.0, 0.0};
  std::array<double, 3> normal{0.0, 0.0, 1.0};

  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;
  double horizontal = 0.0;
  double distance_3d = 0.0;
  double angle_degrees = 0.0;
  double azimuth_degrees = 0.0;
  double slope_percent = 0.0;
  double perimeter_3d = 0.0;
  double perimeter_horizontal = 0.0;
  double area_planar = 0.0;
  double area_horizontal = 0.0;
  double signed_distance = 0.0;
  double radius = 0.0;
  double diameter = 0.0;
  double circumference = 0.0;
  double fit_rms = 0.0;
};

QString measurement_kind_key(MeasurementKind kind);
QString measurement_kind_label(MeasurementKind kind);
MeasurementKind measurement_kind_from_key(const QString & key);
int measurement_required_points(MeasurementKind kind);
int measurement_minimum_points(MeasurementKind kind);
bool measurement_requires_manual_finish(MeasurementKind kind);

MeasurementRecord calculate_measurement(
  MeasurementKind kind,
  const std::vector<pcl::PointXYZ> & vertices,
  int id = 0);

QJsonObject measurement_to_json(const MeasurementRecord & record);
bool measurement_from_json(const QJsonObject & object, MeasurementRecord & record);
