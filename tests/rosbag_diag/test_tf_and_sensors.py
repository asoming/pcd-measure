from __future__ import annotations

import math
import unittest
from types import SimpleNamespace as Namespace

from rosbag_diag.sensor_analysis import SensorAnalyzer
from rosbag_diag.tf_analysis import TfAnalyzer


def stamp(nanoseconds: int) -> Namespace:
    return Namespace(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def transform(
    parent: str,
    child: str,
    nanoseconds: int,
    x: float,
    *,
    yaw_quaternion_z: float = 0.0,
    yaw_quaternion_w: float = 1.0,
) -> Namespace:
    return Namespace(
        header=Namespace(frame_id=parent, stamp=stamp(nanoseconds)),
        child_frame_id=child,
        transform=Namespace(
            translation=Namespace(x=x, y=0.0, z=0.0),
            rotation=Namespace(x=0.0, y=0.0, z=yaw_quaternion_z, w=yaw_quaternion_w),
        ),
    )


class TfAnalyzerTests(unittest.TestCase):
    def test_detects_jump_multiple_parent_cycle_and_disconnected_tree(self) -> None:
        analyzer = TfAnalyzer()
        for index in range(12):
            x = index * 0.01 if index < 11 else 3.0
            message = Namespace(transforms=[transform("map", "base", index * 100_000_000, x)])
            analyzer.consume("/tf", message, index * 100_000_000)
        analyzer.consume("/tf", Namespace(transforms=[transform("odom", "base", 2_000_000_000, 0.0)]), 2_000_000_000)
        analyzer.consume("/tf", Namespace(transforms=[transform("base", "map", 2_100_000_000, 0.0)]), 2_100_000_000)
        analyzer.consume("/tf_static", Namespace(transforms=[transform("isolated", "camera", 1, 0.0)]), 1)

        result, issues = analyzer.finish(
            translation_jump_m=0.5,
            rotation_jump_deg=30.0,
            maximum_speed_mps=10.0,
            gap_factor=3.0,
        )
        identifiers = {issue.issue_id for issue in issues}

        self.assertTrue(result["available"])
        self.assertIn("TF_MULTIPLE_PARENTS:base", identifiers)
        self.assertTrue(any(value.startswith("TF_TRANSLATION_JUMP") for value in identifiers))
        self.assertIn("TF_GRAPH_CYCLE", identifiers)
        self.assertIn("TF_DISCONNECTED_GRAPH", identifiers)

    def test_detects_changed_static_transform_and_invalid_transform(self) -> None:
        analyzer = TfAnalyzer()
        analyzer.consume("/tf_static", Namespace(transforms=[transform("base", "camera", 1, 0.0)]), 1)
        analyzer.consume("/tf_static", Namespace(transforms=[transform("base", "camera", 2, 0.1)]), 2)
        analyzer.consume(
            "/tf",
            Namespace(transforms=[transform("", "missing_parent", 3, 0.0)]),
            3,
        )

        result, issues = analyzer.finish(
            translation_jump_m=0.5,
            rotation_jump_deg=30.0,
            maximum_speed_mps=10.0,
            gap_factor=3.0,
        )

        self.assertEqual(result["invalid_transform_count"], 1)
        self.assertTrue(any(issue.issue_id.startswith("TF_STATIC_CHANGED") for issue in issues))

    def test_short_recording_jump_gap_time_reversal_and_bad_quaternion(self) -> None:
        analyzer = TfAnalyzer()
        analyzer.consume(
            "/tf", Namespace(transforms=[transform("map", "base", 100_000_000, 0.0)]),
            100_000_000,
        )
        analyzer.consume(
            "/tf",
            Namespace(transforms=[transform(
                "map", "base", 200_000_000, 2.0,
                yaw_quaternion_z=math.sin(math.pi / 4.0),
                yaw_quaternion_w=math.cos(math.pi / 4.0),
            )]),
            200_000_000,
        )
        for index in range(3, 12):
            timestamp = index * 100_000_000
            analyzer.consume(
                "/tf", Namespace(transforms=[transform("map", "laser", timestamp, index * 0.01)]),
                timestamp,
            )
        analyzer.consume(
            "/tf", Namespace(transforms=[transform("map", "laser", 3_000_000_000, 0.12)]),
            3_000_000_000,
        )
        analyzer.consume(
            "/tf", Namespace(transforms=[transform("map", "laser", 2_500_000_000, 0.13)]),
            2_500_000_000,
        )
        zero_rotation = transform("base", "broken", 1, 0.0, yaw_quaternion_w=0.0)
        analyzer.consume("/tf", Namespace(transforms=[zero_rotation]), 1)
        non_unit = transform("base", "camera", 1, 0.0, yaw_quaternion_w=2.0)
        analyzer.consume("/tf_static", Namespace(transforms=[non_unit]), 1)

        result, issues = analyzer.finish(
            translation_jump_m=0.5,
            rotation_jump_deg=30.0,
            maximum_speed_mps=10.0,
            gap_factor=3.0,
        )
        identifiers = {issue.issue_id for issue in issues}

        self.assertEqual(result["invalid_transform_count"], 1)
        self.assertTrue(any(value.startswith("TF_TRANSLATION_JUMP") for value in identifiers))
        self.assertTrue(any(value.startswith("TF_ROTATION_JUMP") for value in identifiers))
        self.assertTrue(any(value.startswith("TF_SPEED_SPIKE") for value in identifiers))
        self.assertTrue(any(value.startswith("TF_GAP") for value in identifiers))
        self.assertTrue(any(value.startswith("TF_TIME_BACKWARDS") for value in identifiers))
        self.assertTrue(any(value.startswith("TF_QUATERNION_NORM") for value in identifiers))


class SensorAnalyzerTests(unittest.TestCase):
    def test_imu_invalid_quaternion_and_saturation_are_reported(self) -> None:
        analyzer = SensorAnalyzer()
        normal = Namespace(
            linear_acceleration=Namespace(x=0.0, y=0.0, z=9.81),
            angular_velocity=Namespace(x=0.0, y=0.0, z=0.01),
            orientation=Namespace(x=0.0, y=0.0, z=0.0, w=2.0),
        )
        saturated = Namespace(
            linear_acceleration=Namespace(x=200.0, y=0.0, z=0.0),
            angular_velocity=Namespace(x=40.0, y=0.0, z=0.0),
            orientation=Namespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        invalid = Namespace(
            linear_acceleration=Namespace(x=math.nan, y=0.0, z=0.0),
            angular_velocity=Namespace(x=0.0, y=0.0, z=0.0),
            orientation=Namespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        for _ in range(20):
            analyzer.consume("/imu", "sensor_msgs/msg/Imu", normal)
        analyzer.consume("/imu", "sensor_msgs/msg/Imu", saturated)
        analyzer.consume("/imu", "sensor_msgs/msg/Imu", invalid)

        _, issues = analyzer.finish()
        identifiers = {issue.issue_id for issue in issues}

        self.assertIn("SENSOR_INVALID:/imu", identifiers)
        self.assertIn("SENSOR_SATURATION:/imu", identifiers)
        self.assertIn("IMU_QUATERNION_NORM:/imu", identifiers)

    def test_image_malformed_frozen_and_hardware_diagnostic_error_are_reported(self) -> None:
        analyzer = SensorAnalyzer()
        image = Namespace(width=10, height=10, step=30, data=b"short")
        for _ in range(15):
            analyzer.consume("/camera/image", "sensor_msgs/msg/Image", image)
        diagnostic = Namespace(status=[Namespace(level=2, name="camera", message="device offline")])
        analyzer.consume("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", diagnostic)

        _, issues = analyzer.finish()
        identifiers = {issue.issue_id for issue in issues}

        self.assertIn("SENSOR_EMPTY_OR_MALFORMED:/camera/image", identifiers)
        self.assertIn("SENSOR_FROZEN:/camera/image", identifiers)
        self.assertTrue(any(value.startswith("HARDWARE_DIAGNOSTIC:camera") for value in identifiers))

    def test_lidar_livox_cloud_odometry_gnss_and_battery_faults(self) -> None:
        analyzer = SensorAnalyzer()
        scan = Namespace(ranges=[1.0, 2.0, math.inf, math.inf, math.inf], range_min=0.1, range_max=10.0)
        for _ in range(5):
            analyzer.consume("/scan", "sensor_msgs/msg/LaserScan", scan)
        analyzer.consume(
            "/points", "sensor_msgs/msg/PointCloud2",
            Namespace(width=10, height=1, point_step=16, row_step=160, data=b"short"),
        )
        analyzer.consume(
            "/livox/lidar", "livox_ros_driver2/msg/CustomMsg",
            Namespace(
                point_num=3,
                points=[Namespace(x=1.0, y=0.0, z=0.0, reflectivity=20)],
                timebase=100,
            ),
        )
        analyzer.consume(
            "/odom", "nav_msgs/msg/Odometry",
            Namespace(pose=Namespace(pose=Namespace(position=Namespace(x=0.0, y=0.0, z=0.0)))),
        )
        analyzer.consume(
            "/odom", "nav_msgs/msg/Odometry",
            Namespace(pose=Namespace(pose=Namespace(position=Namespace(x=3.0, y=0.0, z=0.0)))),
        )
        analyzer.consume(
            "/fix", "sensor_msgs/msg/NavSatFix",
            Namespace(
                latitude=0.0, longitude=0.0, altitude=0.0,
                status=Namespace(status=-1), position_covariance=[1.0] * 9,
            ),
        )
        analyzer.consume(
            "/battery", "sensor_msgs/msg/BatteryState",
            Namespace(voltage=20.0, current=2.0, percentage=0.05, temperature=85.0),
        )
        analyzer.consume(
            "/joint_states", "sensor_msgs/msg/JointState",
            Namespace(name=["left", "right"], position=[0.0], velocity=[0.0, 0.0]),
        )
        analyzer.consume(
            "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray",
            Namespace(status=[Namespace(level=1, name="motor", message="temperature rising")]),
        )

        sensors, issues = analyzer.finish()
        identifiers = {issue.issue_id for issue in issues}
        roles = {sensor["role"] for sensor in sensors}

        self.assertTrue({
            "激光扫描", "点云", "Livox 雷达", "里程计", "GNSS", "电池",
            "关节状态", "硬件诊断",
        } <= roles)
        self.assertIn("LIDAR_INVALID_RANGES:/scan", identifiers)
        self.assertIn("SENSOR_EMPTY_OR_MALFORMED:/points", identifiers)
        self.assertIn("SENSOR_EMPTY_OR_MALFORMED:/livox/lidar", identifiers)
        self.assertIn("ODOMETRY_POSITION_JUMP:/odom", identifiers)
        self.assertIn("GNSS_NO_FIX:/fix", identifiers)
        self.assertIn("BATTERY_LOW:/battery", identifiers)
        self.assertIn("BATTERY_TEMPERATURE_HIGH:/battery", identifiers)
        self.assertIn("SENSOR_EMPTY_OR_MALFORMED:/joint_states", identifiers)
        hardware = next(issue for issue in issues if issue.issue_id.startswith("HARDWARE_DIAGNOSTIC:motor"))
        self.assertEqual(hardware.severity, "warning")


if __name__ == "__main__":
    unittest.main()
