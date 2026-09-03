#include "measurement.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QDateTime>
#include <QJsonArray>
#include <QStringList>

#include <Eigen/Eigenvalues>

namespace
{

constexpr double kEpsilon = 1e-12;

Eigen::Vector3d to_vector(const pcl::PointXYZ & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

pcl::PointXYZ to_point(const Eigen::Vector3d & value)
{
  return pcl::PointXYZ(
    static_cast<float>(value.x()),
    static_cast<float>(value.y()),
    static_cast<float>(value.z()));
}

QJsonArray point_to_json(const pcl::PointXYZ & point)
{
  return QJsonArray{point.x, point.y, point.z};
}

bool json_to_point(const QJsonValue & value, pcl::PointXYZ & point)
{
  const QJsonArray array = value.toArray();
  if (array.size() != 3) {
    return false;
  }
  point.x = static_cast<float>(array.at(0).toDouble());
  point.y = static_cast<float>(array.at(1).toDouble());
  point.z = static_cast<float>(array.at(2).toDouble());
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

void calculate_endpoints(MeasurementRecord & record)
{
  if (record.vertices.empty()) {
    return;
  }
  record.point_a = record.vertices.front();
  record.point_b = record.vertices.back();
  record.dx = static_cast<double>(record.point_b.x) - record.point_a.x;
  record.dy = static_cast<double>(record.point_b.y) - record.point_a.y;
  record.dz = static_cast<double>(record.point_b.z) - record.point_a.z;
  const double endpoint_horizontal = std::hypot(record.dx, record.dy);
  record.azimuth_degrees = std::atan2(record.dy, record.dx) * 180.0 / std::acos(-1.0);
  if (record.azimuth_degrees < 0.0) {
    record.azimuth_degrees += 360.0;
  }
  if (endpoint_horizontal > kEpsilon) {
    record.slope_percent = 100.0 * record.dz / endpoint_horizontal;
  } else if (std::abs(record.dz) > kEpsilon) {
    record.slope_percent = std::copysign(
      std::numeric_limits<double>::infinity(), record.dz);
  } else {
    record.slope_percent = 0.0;
  }
}

void calculate_path(MeasurementRecord & record, bool closed)
{
  const std::size_t edge_count = closed ? record.vertices.size() :
    (record.vertices.empty() ? 0 : record.vertices.size() - 1);
  for (std::size_t i = 0; i < edge_count; ++i) {
    const pcl::PointXYZ & first = record.vertices[i];
    const pcl::PointXYZ & second = record.vertices[(i + 1) % record.vertices.size()];
    const double dx = static_cast<double>(second.x) - first.x;
    const double dy = static_cast<double>(second.y) - first.y;
    const double dz = static_cast<double>(second.z) - first.z;
    const double horizontal = std::hypot(dx, dy);
    record.perimeter_horizontal += horizontal;
    record.perimeter_3d += std::hypot(horizontal, dz);
  }
  record.horizontal = record.perimeter_horizontal;
  record.distance_3d = record.perimeter_3d;
}

double cross_2d(
  const Eigen::Vector2d & first,
  const Eigen::Vector2d & second,
  const Eigen::Vector2d & third)
{
  return (second.x() - first.x()) * (third.y() - first.y()) -
    (second.y() - first.y()) * (third.x() - first.x());
}

bool point_on_segment_2d(
  const Eigen::Vector2d & point,
  const Eigen::Vector2d & first,
  const Eigen::Vector2d & second)
{
  constexpr double tolerance = 1e-10;
  return std::abs(cross_2d(first, second, point)) <= tolerance &&
    point.x() >= std::min(first.x(), second.x()) - tolerance &&
    point.x() <= std::max(first.x(), second.x()) + tolerance &&
    point.y() >= std::min(first.y(), second.y()) - tolerance &&
    point.y() <= std::max(first.y(), second.y()) + tolerance;
}

bool segments_intersect_2d(
  const Eigen::Vector2d & a,
  const Eigen::Vector2d & b,
  const Eigen::Vector2d & c,
  const Eigen::Vector2d & d)
{
  const double first = cross_2d(a, b, c);
  const double second = cross_2d(a, b, d);
  const double third = cross_2d(c, d, a);
  const double fourth = cross_2d(c, d, b);
  if (((first > 0.0 && second < 0.0) || (first < 0.0 && second > 0.0)) &&
    ((third > 0.0 && fourth < 0.0) || (third < 0.0 && fourth > 0.0)))
  {
    return true;
  }
  return point_on_segment_2d(c, a, b) || point_on_segment_2d(d, a, b) ||
    point_on_segment_2d(a, c, d) || point_on_segment_2d(b, c, d);
}

bool calculate_angle(MeasurementRecord & record)
{
  const Eigen::Vector3d first = to_vector(record.vertices[0]) - to_vector(record.vertices[1]);
  const Eigen::Vector3d second = to_vector(record.vertices[2]) - to_vector(record.vertices[1]);
  const double denominator = first.norm() * second.norm();
  if (denominator <= kEpsilon) {
    record.error = QStringLiteral("角度测量的相邻点不能重合。");
    return false;
  }
  const double cosine = std::clamp(first.dot(second) / denominator, -1.0, 1.0);
  record.angle_degrees = std::acos(cosine) * 180.0 / std::acos(-1.0);
  calculate_path(record, false);
  return true;
}

bool calculate_area(MeasurementRecord & record)
{
  const std::size_t count = record.vertices.size();
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const pcl::PointXYZ & point : record.vertices) {
    centroid += to_vector(point);
  }
  centroid /= static_cast<double>(count);

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const pcl::PointXYZ & point : record.vertices) {
    const Eigen::Vector3d offset = to_vector(point) - centroid;
    covariance += offset * offset.transpose();
  }
  covariance /= static_cast<double>(count);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    record.error = QStringLiteral("无法拟合面积测量平面。");
    return false;
  }

  Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  Eigen::Vector3d axis_u = solver.eigenvectors().col(2).normalized();
  Eigen::Vector3d axis_v = normal.cross(axis_u).normalized();

  std::vector<Eigen::Vector2d> projected;
  projected.reserve(count);
  for (const pcl::PointXYZ & point : record.vertices) {
    const Eigen::Vector3d offset = to_vector(point) - centroid;
    projected.emplace_back(offset.dot(axis_u), offset.dot(axis_v));
  }
  for (std::size_t index = 0; index < count; ++index) {
    if ((projected[index] - projected[(index + 1) % count]).norm() <= kEpsilon) {
      record.error = QStringLiteral("面积边界存在相邻重复点。");
      return false;
    }
  }
  for (std::size_t first = 0; first < count; ++first) {
    const std::size_t first_next = (first + 1) % count;
    for (std::size_t second = first + 1; second < count; ++second) {
      const std::size_t second_next = (second + 1) % count;
      if (first == second || first_next == second || second_next == first) continue;
      if (segments_intersect_2d(
          projected[first], projected[first_next], projected[second], projected[second_next]))
      {
        record.error = QStringLiteral("面积边界存在自交。");
        return false;
      }
    }
  }

  double planar_twice_area = 0.0;
  double horizontal_twice_area = 0.0;
  double squared_residual_sum = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const Eigen::Vector3d current = to_vector(record.vertices[i]) - centroid;
    const Eigen::Vector3d next = to_vector(record.vertices[(i + 1) % count]) - centroid;
    planar_twice_area += current.dot(axis_u) * next.dot(axis_v) -
      next.dot(axis_u) * current.dot(axis_v);
    horizontal_twice_area += current.x() * next.y() - next.x() * current.y();
    const double residual = current.dot(normal);
    squared_residual_sum += residual * residual;
  }

  record.area_planar = std::abs(planar_twice_area) * 0.5;
  record.area_horizontal = std::abs(horizontal_twice_area) * 0.5;
  record.fit_rms = std::sqrt(squared_residual_sum / static_cast<double>(count));
  record.center = {centroid.x(), centroid.y(), centroid.z()};
  record.normal = {normal.x(), normal.y(), normal.z()};
  calculate_path(record, true);
  if (record.area_planar <= kEpsilon) {
    record.error = QStringLiteral("面积测量的点近似共线。");
    return false;
  }
  return true;
}

bool calculate_point_to_plane(MeasurementRecord & record)
{
  const Eigen::Vector3d point = to_vector(record.vertices[0]);
  const Eigen::Vector3d plane_a = to_vector(record.vertices[1]);
  const Eigen::Vector3d plane_b = to_vector(record.vertices[2]);
  const Eigen::Vector3d plane_c = to_vector(record.vertices[3]);
  Eigen::Vector3d normal = (plane_b - plane_a).cross(plane_c - plane_a);
  if (normal.norm() <= kEpsilon) {
    record.error = QStringLiteral("定义平面的三个点不能共线。");
    return false;
  }
  normal.normalize();
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  record.signed_distance = (point - plane_a).dot(normal);
  const Eigen::Vector3d projection = point - record.signed_distance * normal;
  record.point_a = record.vertices[0];
  record.point_b = to_point(projection);
  record.dx = projection.x() - point.x();
  record.dy = projection.y() - point.y();
  record.dz = projection.z() - point.z();
  record.horizontal = std::hypot(record.dx, record.dy);
  record.distance_3d = std::abs(record.signed_distance);
  record.center = {projection.x(), projection.y(), projection.z()};
  record.normal = {normal.x(), normal.y(), normal.z()};
  return true;
}

bool calculate_circle(MeasurementRecord & record)
{
  const Eigen::Vector3d first = to_vector(record.vertices[0]);
  const Eigen::Vector3d u = to_vector(record.vertices[1]) - first;
  const Eigen::Vector3d v = to_vector(record.vertices[2]) - first;
  const Eigen::Vector3d cross = u.cross(v);
  const double denominator = 2.0 * cross.squaredNorm();
  if (denominator <= kEpsilon) {
    record.error = QStringLiteral("圆测量的三个点不能共线。");
    return false;
  }
  const Eigen::Vector3d offset =
    (u.squaredNorm() * v.cross(cross) + v.squaredNorm() * cross.cross(u)) / denominator;
  const Eigen::Vector3d center = first + offset;
  Eigen::Vector3d normal = cross.normalized();
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  record.center = {center.x(), center.y(), center.z()};
  record.normal = {normal.x(), normal.y(), normal.z()};
  record.radius = offset.norm();
  record.diameter = 2.0 * record.radius;
  record.circumference = 2.0 * std::acos(-1.0) * record.radius;
  record.distance_3d = record.circumference;
  record.horizontal = record.diameter;
  return true;
}

}  // namespace

QString measurement_kind_key(MeasurementKind kind)
{
  switch (kind) {
    case MeasurementKind::Point: return QStringLiteral("point");
    case MeasurementKind::Segment: return QStringLiteral("segment");
    case MeasurementKind::Polyline: return QStringLiteral("polyline");
    case MeasurementKind::Angle: return QStringLiteral("angle");
    case MeasurementKind::Area: return QStringLiteral("area");
    case MeasurementKind::Orthogonal: return QStringLiteral("orthogonal");
    case MeasurementKind::PointToPlane: return QStringLiteral("point_to_plane");
    case MeasurementKind::Circle: return QStringLiteral("circle");
  }
  return QStringLiteral("segment");
}

QString measurement_kind_label(MeasurementKind kind)
{
  switch (kind) {
    case MeasurementKind::Point: return QStringLiteral("单点坐标");
    case MeasurementKind::Segment: return QStringLiteral("两点距离");
    case MeasurementKind::Polyline: return QStringLiteral("连续折线");
    case MeasurementKind::Angle: return QStringLiteral("三点角度");
    case MeasurementKind::Area: return QStringLiteral("多边形面积");
    case MeasurementKind::Orthogonal: return QStringLiteral("正交分解");
    case MeasurementKind::PointToPlane: return QStringLiteral("点到平面");
    case MeasurementKind::Circle: return QStringLiteral("圆与直径");
  }
  return QStringLiteral("两点距离");
}

MeasurementKind measurement_kind_from_key(const QString & key)
{
  const QString normalized = key.trimmed().toLower();
  if (normalized == QStringLiteral("point")) return MeasurementKind::Point;
  if (normalized == QStringLiteral("polyline")) return MeasurementKind::Polyline;
  if (normalized == QStringLiteral("angle")) return MeasurementKind::Angle;
  if (normalized == QStringLiteral("area")) return MeasurementKind::Area;
  if (normalized == QStringLiteral("orthogonal")) return MeasurementKind::Orthogonal;
  if (normalized == QStringLiteral("point_to_plane")) return MeasurementKind::PointToPlane;
  if (normalized == QStringLiteral("circle")) return MeasurementKind::Circle;
  return MeasurementKind::Segment;
}

int measurement_required_points(MeasurementKind kind)
{
  switch (kind) {
    case MeasurementKind::Point: return 1;
    case MeasurementKind::Segment: return 2;
    case MeasurementKind::Angle: return 3;
    case MeasurementKind::Orthogonal: return 2;
    case MeasurementKind::PointToPlane: return 4;
    case MeasurementKind::Circle: return 3;
    case MeasurementKind::Polyline:
    case MeasurementKind::Area:
      return 0;
  }
  return 0;
}

int measurement_minimum_points(MeasurementKind kind)
{
  if (kind == MeasurementKind::Polyline) return 2;
  if (kind == MeasurementKind::Area) return 3;
  return measurement_required_points(kind);
}

bool measurement_requires_manual_finish(MeasurementKind kind)
{
  return kind == MeasurementKind::Polyline || kind == MeasurementKind::Area;
}

MeasurementRecord calculate_measurement(
  MeasurementKind kind,
  const std::vector<pcl::PointXYZ> & vertices,
  int id)
{
  MeasurementRecord record;
  record.id = id;
  record.kind = kind;
  record.vertices = vertices;
  record.created_at = QDateTime::currentDateTime().toString(Qt::ISODate);
  record.name = id > 0 ?
    QStringLiteral("%1 #%2").arg(measurement_kind_label(kind)).arg(id) :
    measurement_kind_label(kind);

  const int required = measurement_required_points(kind);
  const int minimum = measurement_minimum_points(kind);
  if (required > 0 && static_cast<int>(vertices.size()) != required) {
    record.error = QStringLiteral("%1需要恰好 %2 个点。")
      .arg(measurement_kind_label(kind)).arg(required);
    return record;
  }
  if (static_cast<int>(vertices.size()) < minimum) {
    record.error = QStringLiteral("%1至少需要 %2 个点。")
      .arg(measurement_kind_label(kind)).arg(minimum);
    return record;
  }
  for (const pcl::PointXYZ & point : vertices) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      record.error = QStringLiteral("测量点坐标必须是有限数值。");
      return record;
    }
  }
  calculate_endpoints(record);

  switch (kind) {
    case MeasurementKind::Point:
      record.valid = true;
      break;
    case MeasurementKind::Segment:
    case MeasurementKind::Polyline:
    case MeasurementKind::Orthogonal:
      calculate_path(record, false);
      record.angle_degrees = std::atan2(
        std::abs(record.dz), std::hypot(record.dx, record.dy)) * 180.0 / std::acos(-1.0);
      record.valid = true;
      break;
    case MeasurementKind::Angle:
      record.valid = calculate_angle(record);
      break;
    case MeasurementKind::Area:
      record.valid = calculate_area(record);
      break;
    case MeasurementKind::PointToPlane:
      record.valid = calculate_point_to_plane(record);
      break;
    case MeasurementKind::Circle:
      record.valid = calculate_circle(record);
      break;
  }
  return record;
}

QJsonObject measurement_to_json(const MeasurementRecord & record)
{
  QJsonObject object;
  object.insert(QStringLiteral("id"), record.id);
  object.insert(QStringLiteral("kind"), measurement_kind_key(record.kind));
  object.insert(QStringLiteral("name"), record.name);
  object.insert(QStringLiteral("group"), record.group);
  object.insert(QStringLiteral("note"), record.note);
  object.insert(QStringLiteral("color"), record.color_hex);
  object.insert(QStringLiteral("visible"), record.visible);
  object.insert(QStringLiteral("created_at"), record.created_at);
  QJsonArray vertices;
  for (const pcl::PointXYZ & point : record.vertices) {
    vertices.append(point_to_json(point));
  }
  object.insert(QStringLiteral("vertices_m"), vertices);
  object.insert(QStringLiteral("delta_m"), QJsonArray{record.dx, record.dy, record.dz});
  object.insert(QStringLiteral("distance_3d_m"), record.distance_3d);
  object.insert(QStringLiteral("horizontal_distance_m"), record.horizontal);
  object.insert(QStringLiteral("angle_deg"), record.angle_degrees);
  object.insert(QStringLiteral("azimuth_deg"), record.azimuth_degrees);
  object.insert(QStringLiteral("slope_percent"),
    std::isfinite(record.slope_percent) ? record.slope_percent : QJsonValue());
  object.insert(QStringLiteral("perimeter_3d_m"), record.perimeter_3d);
  object.insert(QStringLiteral("perimeter_horizontal_m"), record.perimeter_horizontal);
  object.insert(QStringLiteral("area_planar_m2"), record.area_planar);
  object.insert(QStringLiteral("area_horizontal_m2"), record.area_horizontal);
  object.insert(QStringLiteral("signed_distance_m"), record.signed_distance);
  object.insert(QStringLiteral("center_m"),
    QJsonArray{record.center[0], record.center[1], record.center[2]});
  object.insert(QStringLiteral("normal"),
    QJsonArray{record.normal[0], record.normal[1], record.normal[2]});
  object.insert(QStringLiteral("radius_m"), record.radius);
  object.insert(QStringLiteral("diameter_m"), record.diameter);
  object.insert(QStringLiteral("circumference_m"), record.circumference);
  object.insert(QStringLiteral("fit_rms_m"), record.fit_rms);
  return object;
}

bool measurement_from_json(const QJsonObject & object, MeasurementRecord & record)
{
  std::vector<pcl::PointXYZ> vertices;
  for (const QJsonValue & value : object.value(QStringLiteral("vertices_m")).toArray()) {
    pcl::PointXYZ point;
    if (json_to_point(value, point)) {
      vertices.push_back(point);
    }
  }
  const int id = std::max(1, object.value(QStringLiteral("id")).toInt(1));
  const QString kind_key = object.value(QStringLiteral("kind"))
    .toString(QStringLiteral("segment")).trimmed().toLower();
  const QStringList known_kinds{QStringLiteral("point"), QStringLiteral("segment"),
    QStringLiteral("polyline"), QStringLiteral("angle"), QStringLiteral("area"),
    QStringLiteral("orthogonal"), QStringLiteral("point_to_plane"), QStringLiteral("circle")};
  if (!known_kinds.contains(kind_key)) {
    return false;
  }
  const MeasurementKind kind = measurement_kind_from_key(kind_key);
  MeasurementRecord parsed = calculate_measurement(kind, vertices, id);
  if (!parsed.valid) {
    return false;
  }
  parsed.name = object.value(QStringLiteral("name")).toString(parsed.name);
  parsed.group = object.value(QStringLiteral("group")).toString(QStringLiteral("默认"));
  parsed.note = object.value(QStringLiteral("note")).toString();
  parsed.color_hex = object.value(QStringLiteral("color")).toString(QStringLiteral("#FFB547"));
  parsed.visible = object.value(QStringLiteral("visible")).toBool(true);
  parsed.created_at = object.value(QStringLiteral("created_at")).toString(parsed.created_at);
  record = std::move(parsed);
  return true;
}
