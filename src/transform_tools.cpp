#include "transform_tools.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

#include <pcl/common/point_tests.h>

bool valid_cloud_transform(const CloudTransformParameters & parameters, QString * error)
{
  const std::array<double, 5> values{
    parameters.translation_x, parameters.translation_y, parameters.translation_z,
    parameters.rotation_z_degrees, parameters.uniform_scale};
  if (!std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })) {
    if (error) *error = QStringLiteral("变换参数必须是有限数值。");
    return false;
  }
  if (parameters.uniform_scale <= 1e-9 || parameters.uniform_scale > 1e6) {
    if (error) *error = QStringLiteral("统一尺度必须大于 0 且不超过 1,000,000。");
    return false;
  }
  return true;
}

double normalized_degrees(double degrees)
{
  if (!std::isfinite(degrees)) return std::numeric_limits<double>::quiet_NaN();
  double result = std::fmod(degrees, 360.0);
  if (result <= -180.0) result += 360.0;
  if (result > 180.0) result -= 360.0;
  return result;
}

Eigen::Matrix4f cloud_transform_matrix(const CloudTransformParameters & parameters)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  QString error;
  if (!valid_cloud_transform(parameters, &error)) {
    matrix.setConstant(std::numeric_limits<float>::quiet_NaN());
    return matrix;
  }
  const double angle = normalized_degrees(parameters.rotation_z_degrees) *
    std::acos(-1.0) / 180.0;
  const float cosine = static_cast<float>(std::cos(angle) * parameters.uniform_scale);
  const float sine = static_cast<float>(std::sin(angle) * parameters.uniform_scale);
  matrix(0, 0) = cosine;
  matrix(0, 1) = -sine;
  matrix(1, 0) = sine;
  matrix(1, 1) = cosine;
  matrix(2, 2) = static_cast<float>(parameters.uniform_scale);
  matrix(0, 3) = static_cast<float>(parameters.translation_x);
  matrix(1, 3) = static_cast<float>(parameters.translation_y);
  matrix(2, 3) = static_cast<float>(parameters.translation_z);
  return matrix;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_point_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud,
  const Eigen::Matrix4f & transform)
{
  auto result = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (!cloud || !transform.allFinite()) return result;
  result->reserve(cloud->size());
  for (const pcl::PointXYZRGB & source : *cloud) {
    if (!pcl::isFinite(source)) continue;
    const Eigen::Vector4f transformed = transform * Eigen::Vector4f(source.x, source.y, source.z, 1.0F);
    if (!transformed.allFinite()) continue;
    pcl::PointXYZRGB target = source;
    target.x = transformed.x();
    target.y = transformed.y();
    target.z = transformed.z();
    result->push_back(target);
  }
  result->width = static_cast<std::uint32_t>(result->size());
  result->height = 1;
  result->is_dense = true;
  return result;
}

MeasurementRecord transform_measurement_record(
  const MeasurementRecord & record,
  const Eigen::Matrix4f & transform)
{
  std::vector<pcl::PointXYZ> vertices;
  vertices.reserve(record.vertices.size());
  if (!transform.allFinite()) return MeasurementRecord{};
  for (const pcl::PointXYZ & source : record.vertices) {
    const Eigen::Vector4f transformed = transform * Eigen::Vector4f(source.x, source.y, source.z, 1.0F);
    vertices.emplace_back(transformed.x(), transformed.y(), transformed.z());
  }
  MeasurementRecord result = calculate_measurement(record.kind, vertices, record.id);
  result.name = record.name;
  result.group = record.group;
  result.note = record.note;
  result.color_hex = record.color_hex;
  result.created_at = record.created_at;
  result.visible = record.visible;
  return result;
}

bool write_transform_matrix_json(
  const QString & path,
  const Eigen::Matrix4f & transform,
  const CloudTransformParameters & parameters,
  QString * error)
{
  if (!transform.allFinite() || !valid_cloud_transform(parameters, error)) return false;
  QJsonArray rows;
  for (int row = 0; row < 4; ++row) {
    QJsonArray values;
    for (int column = 0; column < 4; ++column) values.append(transform(row, column));
    rows.append(values);
  }
  QJsonObject parameter_json{
    {QStringLiteral("translation_m"), QJsonArray{
      parameters.translation_x, parameters.translation_y, parameters.translation_z}},
    {QStringLiteral("rotation_z_degrees"), normalized_degrees(parameters.rotation_z_degrees)},
    {QStringLiteral("uniform_scale"), parameters.uniform_scale}};
  QJsonObject root{{QStringLiteral("format"), QStringLiteral("pcd-transform-matrix")},
    {QStringLiteral("matrix"), rows}, {QStringLiteral("parameters"), parameter_json}};
  QSaveFile file(path);
  const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
  if (!file.open(QIODevice::WriteOnly) || file.write(payload) != payload.size() || !file.commit())
  {
    if (error) *error = QStringLiteral("变换矩阵 JSON 写入失败：%1").arg(path);
    return false;
  }
  return true;
}
