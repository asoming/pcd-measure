#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <QJsonObject>
#include <QTest>

#include "measurement.h"

namespace
{

bool close_to(double actual, double expected, double tolerance = 1e-6)
{
  return std::abs(actual - expected) <= tolerance;
}

}  // namespace

class MeasurementTest : public QObject
{
  Q_OBJECT

private slots:
  void kindDefinitions()
  {
    QCOMPARE(measurement_required_points(MeasurementKind::Point), 1);
    QCOMPARE(measurement_required_points(MeasurementKind::PointToPlane), 4);
    QCOMPARE(measurement_minimum_points(MeasurementKind::Area), 3);
    QVERIFY(measurement_requires_manual_finish(MeasurementKind::Polyline));
    QVERIFY(measurement_requires_manual_finish(MeasurementKind::Area));
    QVERIFY(!measurement_requires_manual_finish(MeasurementKind::Circle));
    QCOMPARE(measurement_kind_from_key(QStringLiteral("circle")), MeasurementKind::Circle);
    QVERIFY(!calculate_measurement(
      MeasurementKind::Segment,
      {pcl::PointXYZ(), pcl::PointXYZ(1.0F, 0.0F, 0.0F),
       pcl::PointXYZ(2.0F, 0.0F, 0.0F)}).valid);
  }

  void pointAndLinearMeasurements()
  {
    const pcl::PointXYZ a(0.0F, 0.0F, 0.0F);
    const pcl::PointXYZ b(3.0F, 4.0F, 12.0F);

    const MeasurementRecord point = calculate_measurement(MeasurementKind::Point, {b}, 1);
    QVERIFY(point.valid);
    QCOMPARE(point.vertices.size(), std::size_t(1));
    QVERIFY(close_to(point.point_a.z, 12.0));

    const MeasurementRecord segment = calculate_measurement(MeasurementKind::Segment, {a, b}, 2);
    QVERIFY(segment.valid);
    QVERIFY(close_to(segment.horizontal, 5.0));
    QVERIFY(close_to(segment.distance_3d, 13.0));
    QVERIFY(close_to(segment.slope_percent, 240.0));

    const MeasurementRecord polyline = calculate_measurement(
      MeasurementKind::Polyline,
      {a, pcl::PointXYZ(3.0F, 0.0F, 0.0F), pcl::PointXYZ(3.0F, 4.0F, 0.0F)}, 3);
    QVERIFY(polyline.valid);
    QVERIFY(close_to(polyline.distance_3d, 7.0));
    QVERIFY(close_to(polyline.perimeter_horizontal, 7.0));

    const MeasurementRecord orthogonal = calculate_measurement(
      MeasurementKind::Orthogonal, {a, b}, 4);
    QVERIFY(orthogonal.valid);
    QVERIFY(close_to(orthogonal.dx, 3.0));
    QVERIFY(close_to(orthogonal.dy, 4.0));
    QVERIFY(close_to(orthogonal.dz, 12.0));
    QVERIFY(close_to(orthogonal.distance_3d, 13.0));

    const MeasurementRecord vertical = calculate_measurement(MeasurementKind::Segment,
      {pcl::PointXYZ(1.0F, 1.0F, 5.0F), pcl::PointXYZ(1.0F, 1.0F, -2.0F)}, 40);
    QVERIFY(vertical.valid);
    QVERIFY(close_to(vertical.horizontal, 0.0));
    QVERIFY(close_to(vertical.dz, -7.0));
    QVERIFY(std::isinf(vertical.slope_percent));
    QVERIFY(std::signbit(vertical.slope_percent));
    QVERIFY(close_to(vertical.angle_degrees, 90.0));

    const MeasurementRecord zero = calculate_measurement(MeasurementKind::Segment,
      {a, a}, 41);
    QVERIFY(zero.valid);
    QVERIFY(close_to(zero.distance_3d, 0.0));
    QVERIFY(close_to(zero.slope_percent, 0.0));

    const std::array<std::pair<pcl::PointXYZ, double>, 4> directions{{
      {pcl::PointXYZ(1.0F, 0.0F, 0.0F), 0.0},
      {pcl::PointXYZ(0.0F, 1.0F, 0.0F), 90.0},
      {pcl::PointXYZ(-1.0F, 0.0F, 0.0F), 180.0},
      {pcl::PointXYZ(0.0F, -1.0F, 0.0F), 270.0}}};
    for (const auto & direction : directions) {
      const MeasurementRecord bearing = calculate_measurement(
        MeasurementKind::Orthogonal, {a, direction.first}, 42);
      QVERIFY(close_to(bearing.azimuth_degrees, direction.second));
    }
  }

  void nonFiniteCoordinatesAreRejected()
  {
    pcl::PointXYZ invalid(0.0F, 0.0F, 0.0F);
    invalid.x = std::numeric_limits<float>::quiet_NaN();
    MeasurementRecord result = calculate_measurement(
      MeasurementKind::Segment, {pcl::PointXYZ(), invalid}, 90);
    QVERIFY(!result.valid);
    QVERIFY(result.error.contains(QStringLiteral("有限")));

    invalid.x = std::numeric_limits<float>::infinity();
    result = calculate_measurement(MeasurementKind::Point, {invalid}, 91);
    QVERIFY(!result.valid);
    QVERIFY(!result.error.isEmpty());
  }

  void angleMeasurement()
  {
    const MeasurementRecord record = calculate_measurement(
      MeasurementKind::Angle,
      {pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(0.0F, 0.0F, 0.0F),
       pcl::PointXYZ(0.0F, 1.0F, 0.0F)}, 5);
    QVERIFY(record.valid);
    QVERIFY(close_to(record.angle_degrees, 90.0));
    QVERIFY(close_to(record.perimeter_3d, 2.0));

    const MeasurementRecord invalid = calculate_measurement(
      MeasurementKind::Angle,
      {pcl::PointXYZ(), pcl::PointXYZ(), pcl::PointXYZ(1.0F, 0.0F, 0.0F)}, 6);
    QVERIFY(!invalid.valid);
    QVERIFY(!invalid.error.isEmpty());

    const MeasurementRecord straight = calculate_measurement(
      MeasurementKind::Angle,
      {pcl::PointXYZ(-1.0F, 0.0F, 0.0F), pcl::PointXYZ(),
       pcl::PointXYZ(1.0F, 0.0F, 0.0F)}, 60);
    QVERIFY(straight.valid);
    QVERIFY(close_to(straight.angle_degrees, 180.0));
    const MeasurementRecord zero_angle = calculate_measurement(
      MeasurementKind::Angle,
      {pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(),
       pcl::PointXYZ(2.0F, 0.0F, 0.0F)}, 61);
    QVERIFY(zero_angle.valid);
    QVERIFY(close_to(zero_angle.angle_degrees, 0.0));
  }

  void areaMeasurement()
  {
    const MeasurementRecord record = calculate_measurement(
      MeasurementKind::Area,
      {pcl::PointXYZ(0.0F, 0.0F, 2.0F), pcl::PointXYZ(2.0F, 0.0F, 2.0F),
       pcl::PointXYZ(2.0F, 3.0F, 2.0F), pcl::PointXYZ(0.0F, 3.0F, 2.0F)}, 7);
    QVERIFY(record.valid);
    QVERIFY(close_to(record.area_planar, 6.0));
    QVERIFY(close_to(record.area_horizontal, 6.0));
    QVERIFY(close_to(record.perimeter_3d, 10.0));
    QVERIFY(close_to(record.fit_rms, 0.0));
    QVERIFY(close_to(record.center[0], 1.0));
    QVERIFY(close_to(record.center[1], 1.5));

    const MeasurementRecord tilted = calculate_measurement(
      MeasurementKind::Area,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 2.0F),
       pcl::PointXYZ(2.0F, 3.0F, 2.0F), pcl::PointXYZ(0.0F, 3.0F, 0.0F)}, 70);
    QVERIFY(tilted.valid);
    QVERIFY(close_to(tilted.area_planar, 3.0 * std::sqrt(8.0)));
    QVERIFY(close_to(tilted.area_horizontal, 6.0));
    QVERIFY(close_to(tilted.fit_rms, 0.0));

    const MeasurementRecord self_intersecting = calculate_measurement(
      MeasurementKind::Area,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 2.0F, 0.0F),
       pcl::PointXYZ(0.0F, 2.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F)}, 71);
    QVERIFY(!self_intersecting.valid);
    QVERIFY(self_intersecting.error.contains(QStringLiteral("自交")));

    const MeasurementRecord repeated = calculate_measurement(
      MeasurementKind::Area,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F),
       pcl::PointXYZ(2.0F, 0.0F, 0.0F), pcl::PointXYZ(0.0F, 2.0F, 0.0F)}, 72);
    QVERIFY(!repeated.valid);
    QVERIFY(repeated.error.contains(QStringLiteral("重复")));

    const MeasurementRecord concave = calculate_measurement(
      MeasurementKind::Area,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F),
       pcl::PointXYZ(1.0F, 1.0F, 0.0F), pcl::PointXYZ(2.0F, 2.0F, 0.0F),
       pcl::PointXYZ(0.0F, 2.0F, 0.0F)}, 73);
    QVERIFY(concave.valid);
    QVERIFY(close_to(concave.area_planar, 3.0));
    std::vector<pcl::PointXYZ> reversed_vertices = concave.vertices;
    std::reverse(reversed_vertices.begin(), reversed_vertices.end());
    const MeasurementRecord reversed = calculate_measurement(
      MeasurementKind::Area, reversed_vertices, 74);
    QVERIFY(reversed.valid);
    QVERIFY(close_to(reversed.area_planar, concave.area_planar));
  }

  void pointToPlaneMeasurement()
  {
    const MeasurementRecord record = calculate_measurement(
      MeasurementKind::PointToPlane,
      {pcl::PointXYZ(0.25F, 0.25F, 2.0F), pcl::PointXYZ(0.0F, 0.0F, 0.0F),
       pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(0.0F, 1.0F, 0.0F)}, 8);
    QVERIFY(record.valid);
    QVERIFY(close_to(record.distance_3d, 2.0));
    QVERIFY(close_to(record.signed_distance, 2.0));
    QVERIFY(close_to(record.center[0], 0.25));
    QVERIFY(close_to(record.center[1], 0.25));
    QVERIFY(close_to(record.center[2], 0.0));
    QVERIFY(close_to(record.normal[2], 1.0));

    const MeasurementRecord below = calculate_measurement(
      MeasurementKind::PointToPlane,
      {pcl::PointXYZ(0.2F, 0.3F, -2.0F), pcl::PointXYZ(0.0F, 0.0F, 0.0F),
       pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(0.0F, 1.0F, 0.0F)}, 80);
    QVERIFY(below.valid);
    QVERIFY(close_to(below.signed_distance, -2.0));
    const MeasurementRecord vertical_plane = calculate_measurement(
      MeasurementKind::PointToPlane,
      {pcl::PointXYZ(2.0F, 0.2F, 0.3F), pcl::PointXYZ(0.0F, 0.0F, 0.0F),
       pcl::PointXYZ(0.0F, 1.0F, 0.0F), pcl::PointXYZ(0.0F, 0.0F, 1.0F)}, 81);
    QVERIFY(vertical_plane.valid);
    QVERIFY(close_to(vertical_plane.distance_3d, 2.0));
    const MeasurementRecord collinear_plane = calculate_measurement(
      MeasurementKind::PointToPlane,
      {pcl::PointXYZ(1.0F, 1.0F, 1.0F), pcl::PointXYZ(),
       pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F)}, 82);
    QVERIFY(!collinear_plane.valid);
  }

  void circleMeasurement()
  {
    const MeasurementRecord record = calculate_measurement(
      MeasurementKind::Circle,
      {pcl::PointXYZ(1.0F, 0.0F, 0.0F), pcl::PointXYZ(0.0F, 1.0F, 0.0F),
       pcl::PointXYZ(-1.0F, 0.0F, 0.0F)}, 9);
    QVERIFY(record.valid);
    QVERIFY(close_to(record.center[0], 0.0));
    QVERIFY(close_to(record.center[1], 0.0));
    QVERIFY(close_to(record.radius, 1.0));
    QVERIFY(close_to(record.diameter, 2.0));
    QVERIFY(close_to(record.circumference, 2.0 * std::acos(-1.0)));

    const MeasurementRecord vertical = calculate_measurement(
      MeasurementKind::Circle,
      {pcl::PointXYZ(1.0F, 3.0F, 3.0F), pcl::PointXYZ(1.0F, 2.0F, 4.0F),
       pcl::PointXYZ(1.0F, 1.0F, 3.0F)}, 10);
    QVERIFY(vertical.valid);
    QVERIFY(close_to(vertical.center[0], 1.0));
    QVERIFY(close_to(vertical.center[1], 2.0));
    QVERIFY(close_to(vertical.center[2], 3.0));
    QVERIFY(close_to(vertical.radius, 1.0));

    const MeasurementRecord near_collinear = calculate_measurement(
      MeasurementKind::Circle,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(1.0F, 0.0F, 0.0F),
       pcl::PointXYZ(2.0F, 0.0F, 0.0F)}, 11);
    QVERIFY(!near_collinear.valid);
  }

  void serializationRoundTrip()
  {
    MeasurementRecord source = calculate_measurement(
      MeasurementKind::Area,
      {pcl::PointXYZ(0.0F, 0.0F, 0.0F), pcl::PointXYZ(2.0F, 0.0F, 0.0F),
       pcl::PointXYZ(0.0F, 2.0F, 0.0F)}, 12);
    source.name = QStringLiteral("门厅面积");
    source.group = QStringLiteral("一层");
    source.note = QStringLiteral("复核,保留");
    source.color_hex = QStringLiteral("#12ABEF");
    source.visible = false;

    MeasurementRecord restored;
    QVERIFY(measurement_from_json(measurement_to_json(source), restored));
    QCOMPARE(restored.id, 12);
    QCOMPARE(restored.kind, MeasurementKind::Area);
    QCOMPARE(restored.name, source.name);
    QCOMPARE(restored.group, source.group);
    QCOMPARE(restored.note, source.note);
    QCOMPARE(restored.color_hex, source.color_hex);
    QCOMPARE(restored.visible, false);
    QVERIFY(close_to(restored.area_planar, 2.0));

    QJsonObject unknown = measurement_to_json(source);
    unknown.insert(QStringLiteral("kind"), QStringLiteral("future_unknown_kind"));
    QVERIFY(!measurement_from_json(unknown, restored));
  }
};

QTEST_APPLESS_MAIN(MeasurementTest)

#include "measurement_test.moc"
