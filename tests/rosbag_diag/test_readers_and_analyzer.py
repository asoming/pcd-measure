from __future__ import annotations

import json
import shutil
import sqlite3
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as Namespace
from unittest.mock import patch

from rosbag_diag.analyzers import DiagnosticOptions, diagnose_bag
from rosbag_diag.model import BagInformation, DecodedRecord, TopicDefinition, UnsupportedBagError
from rosbag_diag.readers import detect_bag_kind, open_bag
from rosbag_diag.reporting import build_html_report, report_json, write_html_report, write_json_report


def create_sqlite_bag(
    root: Path,
    topics: dict[str, tuple[str, list[int], str]],
    *,
    declared_count_delta: int = 0,
) -> Path:
    root.mkdir(parents=True)
    database_path = root / "test_0.db3"
    database = sqlite3.connect(database_path)
    database.executescript(
        """
        CREATE TABLE schema(schema_version INTEGER, ros_distro TEXT);
        CREATE TABLE metadata(id INTEGER PRIMARY KEY, metadata_version INTEGER, metadata TEXT);
        CREATE TABLE topics(
          id INTEGER PRIMARY KEY,
          name TEXT NOT NULL,
          type TEXT NOT NULL,
          serialization_format TEXT NOT NULL,
          offered_qos_profiles TEXT NOT NULL
        );
        CREATE TABLE messages(
          id INTEGER PRIMARY KEY,
          topic_id INTEGER NOT NULL,
          timestamp INTEGER NOT NULL,
          data BLOB NOT NULL
        );
        """
    )
    message_id = 1
    all_timestamps: list[int] = []
    metadata_topics: list[str] = []
    for topic_id, (name, (msgtype, timestamps, qos)) in enumerate(topics.items(), start=1):
        database.execute(
            "INSERT INTO topics VALUES (?, ?, ?, 'cdr', ?)",
            (topic_id, name, msgtype, qos),
        )
        for timestamp in timestamps:
            database.execute(
                "INSERT INTO messages VALUES (?, ?, ?, ?)",
                (message_id, topic_id, timestamp, b"\x00\x01"),
            )
            message_id += 1
            all_timestamps.append(timestamp)
        metadata_topics.append(
            "\n".join(
                [
                    "    - topic_metadata:",
                    f"        name: {name}",
                    f"        type: {msgtype}",
                    "        serialization_format: cdr",
                    f"        offered_qos_profiles: {json.dumps(qos)}",
                    f"      message_count: {len(timestamps)}",
                ]
            )
        )
    database.commit()
    database.close()
    start = min(all_timestamps) if all_timestamps else 0
    end = max(all_timestamps) if all_timestamps else start
    declared_count = len(all_timestamps) + declared_count_delta
    metadata = "\n".join(
        [
            "rosbag2_bagfile_information:",
            "  version: 5",
            "  storage_identifier: sqlite3",
            "  duration:",
            f"    nanoseconds: {max(0, end - start)}",
            "  starting_time:",
            f"    nanoseconds_since_epoch: {start}",
            f"  message_count: {declared_count}",
            "  topics_with_message_count:",
            *metadata_topics,
            "  compression_format: ''",
            "  compression_mode: ''",
            "  relative_file_paths:",
            "    - test_0.db3",
            "",
        ]
    )
    (root / "metadata.yaml").write_text(metadata, encoding="utf-8")
    return root


class ReaderTests(unittest.TestCase):
    def test_detects_directory_and_standalone_database(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = create_sqlite_bag(
                Path(temporary) / "bag",
                {"/status": ("example/msg/Status", [100, 200], "")},
            )

            self.assertEqual(detect_bag_kind(root)[0], "ros2-sqlite3")
            self.assertEqual(detect_bag_kind(root / "test_0.db3")[0], "ros2-sqlite3")
            self.assertEqual(open_bag(root).info.message_count, 2)
            standalone = Path(temporary) / "standalone.db3"
            (root / "test_0.db3").replace(standalone)
            self.assertEqual(open_bag(standalone).info.path, standalone.resolve())

    def test_rejects_missing_and_unrelated_paths(self) -> None:
        with self.assertRaises(UnsupportedBagError):
            detect_bag_kind("/definitely/missing/test.bag")
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "not-a-bag.txt"
            path.write_text("x", encoding="utf-8")
            with self.assertRaises(UnsupportedBagError):
                detect_bag_kind(path)

    def test_corrupt_database_has_stable_user_error(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "corrupt.db3"
            path.write_bytes(b"not a sqlite database")
            with self.assertRaisesRegex(UnsupportedBagError, "SQLite bag"):
                open_bag(path)

    def test_selected_split_database_expands_to_complete_bag(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = create_sqlite_bag(
                Path(temporary) / "split bag 测试",
                {"/status": ("example/msg/Status", [100, 200], "")},
            )
            second = root / "test_1.db3"
            shutil.copy2(root / "test_0.db3", second)
            metadata = (root / "metadata.yaml").read_text(encoding="utf-8")
            metadata = metadata.replace("message_count: 2", "message_count: 4", 1)
            metadata = metadata.replace(
                "    - test_0.db3\n", "    - test_0.db3\n    - test_1.db3\n"
            )
            (root / "metadata.yaml").write_text(metadata, encoding="utf-8")

            source = open_bag(second)

            self.assertEqual(len(source.info.files), 2)
            self.assertEqual(sum(1 for _ in source.iter_timestamps()), 4)

    def test_missing_split_is_rejected_before_analysis(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = create_sqlite_bag(
                Path(temporary) / "missing_split",
                {"/status": ("example/msg/Status", [100, 200], "")},
            )
            metadata = (root / "metadata.yaml").read_text(encoding="utf-8")
            metadata = metadata.replace(
                "    - test_0.db3\n", "    - test_0.db3\n    - absent_1.db3\n"
            )
            (root / "metadata.yaml").write_text(metadata, encoding="utf-8")
            with self.assertRaisesRegex(UnsupportedBagError, "分片不存在"):
                open_bag(root)


class DiagnosticJourneyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_regular_control_topic_passes_rate_jitter_and_gap_checks(self) -> None:
        start = 1_700_000_000_000_000_000
        timestamps = [start + index * 50_000_000 for index in range(100)]
        bag = create_sqlite_bag(
            self.root / "regular",
            {"/cmd_vel": ("geometry_msgs/msg/Twist", timestamps, "")},
        )

        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))

        topic = report.topics[0]
        self.assertAlmostEqual(topic["mean_hz"], 20.0)
        self.assertEqual(topic["estimated_drops"], 0)
        identifiers = {issue.issue_id for issue in report.issues}
        self.assertNotIn("TOPIC_RATE_LOW:/cmd_vel", identifiers)
        self.assertNotIn("TOPIC_JITTER:/cmd_vel", identifiers)
        self.assertNotIn("TOPIC_GAPS:/cmd_vel", identifiers)

    def test_low_rate_gap_and_time_reversal_are_reported(self) -> None:
        start = 1_700_000_000_000_000_000
        timestamps = [
            start,
            start + 100_000_000,
            start + 200_000_000,
            start + 1_200_000_000,
            start + 1_100_000_000,
            start + 1_300_000_000,
        ]
        bag = create_sqlite_bag(
            self.root / "broken",
            {"/cmd_vel": ("geometry_msgs/msg/Twist", timestamps, "")},
        )

        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))
        identifiers = {issue.issue_id for issue in report.issues}

        self.assertIn("TOPIC_RATE_LOW:/cmd_vel", identifiers)
        self.assertIn("TOPIC_GAPS:/cmd_vel", identifiers)
        self.assertIn("STORAGE_TIME_BACKWARDS:/cmd_vel", identifiers)

    def test_metadata_count_mismatch_has_reindex_advice(self) -> None:
        bag = create_sqlite_bag(
            self.root / "mismatch",
            {"/topic": ("example/msg/Data", [100, 200, 300], "")},
            declared_count_delta=5,
        )

        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))

        issue = next(issue for issue in report.issues if issue.issue_id.startswith("BAG_METADATA"))
        self.assertIn("reindex", issue.suggestion)

    def test_qos_conflict_and_static_tf_durability_are_reported(self) -> None:
        profiles = "RELIABILITY: RELIABLE RELIABILITY: BEST_EFFORT DURABILITY: VOLATILE"
        bag = create_sqlite_bag(
            self.root / "qos",
            {"/tf_static": ("tf2_msgs/msg/TFMessage", [100, 200], profiles)},
        )

        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))

        self.assertIn("QOS_RISK:/tf_static", {issue.issue_id for issue in report.issues})

    def test_single_recorded_qos_is_not_reported_as_mismatch(self) -> None:
        profiles = "RELIABILITY: RELIABLE DURABILITY: VOLATILE HISTORY: KEEP_LAST"
        bag = create_sqlite_bag(
            self.root / "single_qos",
            {"/camera": ("sensor_msgs/msg/Image", [100, 200, 300], profiles)},
        )

        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))

        self.assertEqual(report.topics[0]["qos"]["status"], "recorded")
        self.assertFalse(any(issue.issue_id.startswith("QOS_RISK") for issue in report.issues))

    def test_duplicate_timestamps_warn_without_misclassifying_status_topic(self) -> None:
        timestamps = [100] * 5 + [200, 300, 400, 500, 600]
        bag = create_sqlite_bag(
            self.root / "duplicates",
            {"/status": ("example/msg/Status", timestamps, "")},
        )

        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))
        identifiers = {issue.issue_id for issue in report.issues}

        self.assertIn("STORAGE_TIME_DUPLICATE:/status", identifiers)
        self.assertNotIn("TOPIC_RATE_LOW:/status", identifiers)

    def test_json_and_html_reports_are_complete_and_atomic(self) -> None:
        bag = create_sqlite_bag(
            self.root / "report",
            {"/topic": ("example/msg/Data", [100, 200, 300], "")},
        )
        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))
        json_path = self.root / "output" / "report.json"
        html_path = self.root / "output" / "report.html"

        write_json_report(report, json_path)
        write_html_report(report, html_path)

        payload = json.loads(json_path.read_text(encoding="utf-8"))
        self.assertEqual(payload["schema"], "rosbag-diagnostic-report")
        self.assertIn("话题时序与 QoS", html_path.read_text(encoding="utf-8"))
        self.assertNotIn("NaN", report_json(report))
        self.assertIn("<!doctype html>", build_html_report(report))

        directory_target = self.root / "existing_directory"
        directory_target.mkdir()
        with self.assertRaises(OSError):
            write_json_report(report, directory_target)
        self.assertEqual(list(self.root.glob(".existing_directory.*.tmp")), [])

    def test_progress_is_monotonic_and_html_escapes_bag_content(self) -> None:
        bag = create_sqlite_bag(
            self.root / "report <unsafe>",
            {"/status<b>": ("example/msg/Data", [100, 200, 300], "")},
        )
        progress: list[int] = []

        report = diagnose_bag(
            bag,
            DiagnosticOptions(deep_analysis=False),
            lambda percent, _message: progress.append(percent),
        )
        html = build_html_report(report)

        self.assertEqual(progress[0], 1)
        self.assertEqual(progress[-1], 100)
        self.assertEqual(progress, sorted(progress))
        self.assertNotIn("/status<b>", html)
        self.assertIn("/status&lt;b&gt;", html)

    def test_empty_bag_is_critical_and_has_recording_advice(self) -> None:
        bag = create_sqlite_bag(self.root / "empty", {})

        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))

        issue = next(issue for issue in report.issues if issue.issue_id == "BAG_EMPTY")
        self.assertEqual(issue.severity, "critical")
        self.assertIn("录制", issue.suggestion)

    def test_header_missing_backward_and_lag_are_reported(self) -> None:
        start = 1_700_000_000_000_000_000
        storage_times = [start + index * 10_000_000 for index in range(20)]
        records: list[DecodedRecord] = []
        for index, storage_time in enumerate(storage_times):
            header_time = storage_time - 250_000_000
            if index in {3, 4}:
                header_time = 0
            elif index == 12:
                header_time -= 500_000_000
            stamp = Namespace(
                sec=header_time // 1_000_000_000,
                nanosec=header_time % 1_000_000_000,
            )
            message = Namespace(
                header=Namespace(stamp=stamp),
                linear_acceleration=Namespace(x=0.0, y=0.0, z=9.81),
                angular_velocity=Namespace(x=0.0, y=0.0, z=0.0),
                orientation=Namespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            records.append(DecodedRecord("/imu", "sensor_msgs/msg/Imu", storage_time, message))

        class FakeSource:
            decode_warnings: list[str] = []
            info = BagInformation(
                path=Path("/tmp/fake-header-bag"), ros_version=2, storage="sqlite3",
                reader="test reader",
                topics={"/imu": TopicDefinition(
                    "/imu", "sensor_msgs/msg/Imu", "cdr", "", len(storage_times)
                )},
                message_count=len(storage_times), start_time_ns=storage_times[0],
                end_time_ns=storage_times[-1], duration_ns=storage_times[-1] - storage_times[0],
            )

            @staticmethod
            def iter_timestamps():
                yield from (("/imu", value) for value in storage_times)

            @staticmethod
            def iter_decoded_samples(_maximum_per_topic: int):
                yield from records

        with patch("rosbag_diag.analyzers.open_bag", return_value=FakeSource()):
            report = diagnose_bag("ignored", DiagnosticOptions())

        identifiers = {issue.issue_id for issue in report.issues}
        self.assertIn("HEADER_STAMP_MISSING:/imu", identifiers)
        self.assertIn("HEADER_STAMP_BACKWARDS:/imu", identifiers)
        self.assertIn("HEADER_STORAGE_LAG:/imu", identifiers)
        self.assertEqual(report.topics[0]["time_basis"], "header")
        self.assertEqual(report.summary["deep_analysis_coverage_percent"], 100.0)

    def test_simulated_clock_epoch_is_not_reported_as_transport_latency(self) -> None:
        storage_start = 1_700_000_000_000_000_000
        storage_times = [storage_start + index * 10_000_000 for index in range(20)]
        records: list[DecodedRecord] = []
        for index, storage_time in enumerate(storage_times):
            header_time = 10_000_000_000 + index * 10_000_000
            message = Namespace(
                header=Namespace(stamp=Namespace(
                    sec=header_time // 1_000_000_000,
                    nanosec=header_time % 1_000_000_000,
                )),
                linear_acceleration=Namespace(x=index * 0.001, y=0.0, z=9.81),
                angular_velocity=Namespace(x=0.0, y=0.0, z=0.0),
                orientation=Namespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            records.append(DecodedRecord("/imu", "sensor_msgs/msg/Imu", storage_time, message))

        class FakeSource:
            decode_warnings: list[str] = []
            info = BagInformation(
                path=Path("/tmp/fake-sim-bag"), ros_version=2, storage="rosbag1",
                reader="test reader",
                topics={
                    "/clock": TopicDefinition(
                        "/clock", "rosgraph_msgs/msg/Clock", "cdr", "", len(storage_times)
                    ),
                    "/imu": TopicDefinition(
                        "/imu", "sensor_msgs/msg/Imu", "cdr", "", len(storage_times)
                    ),
                },
                message_count=len(storage_times) * 2,
                start_time_ns=storage_times[0],
                end_time_ns=storage_times[-1],
                duration_ns=storage_times[-1] - storage_times[0],
            )

            @staticmethod
            def iter_timestamps():
                for value in storage_times:
                    yield "/clock", value
                    yield "/imu", value

            @staticmethod
            def iter_decoded_samples(_maximum_per_topic: int):
                yield from records

        with patch("rosbag_diag.analyzers.open_bag", return_value=FakeSource()):
            report = diagnose_bag("ignored", DiagnosticOptions())

        identifiers = {issue.issue_id for issue in report.issues}
        clock_issue = next(
            issue for issue in report.issues
            if issue.issue_id == "HEADER_STORAGE_CLOCK_DOMAIN"
        )
        self.assertNotIn("HEADER_STORAGE_LAG:/imu", identifiers)
        self.assertEqual(clock_issue.severity, "notice")
        self.assertIn("/clock", clock_issue.evidence)
        self.assertEqual(
            next(topic for topic in report.topics if topic["name"] == "/imu")
            ["header_timing"]["storage_clock_relation"],
            "separate_epoch",
        )
        self.assertGreater(report.summary["score"], 0)
        self.assertEqual(report.summary["critical_count"], 0)

    def test_event_static_and_tf_topics_avoid_periodic_stream_false_positives(self) -> None:
        start = 1_700_000_000_000_000_000
        event_times = [start + index * 100_000_000 for index in range(10)]
        event_times.extend([start + 10_000_000_000, start + 25_000_000_000])
        tf_times = [
            start + offset * 1_000_000
            for offset in (0, 10, 20, 30, 500, 510, 520, 1000, 1010, 1020, 1500, 1510)
        ]
        bag = create_sqlite_bag(
            self.root / "event_topics",
            {
                "/plan": ("nav_msgs/msg/Path", event_times, ""),
                "/map": ("nav_msgs/msg/OccupancyGrid", [start], ""),
                "/tf": ("tf2_msgs/msg/TFMessage", tf_times, ""),
            },
        )

        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))
        identifiers = {issue.issue_id for issue in report.issues}

        self.assertNotIn("TOPIC_GAPS:/plan", identifiers)
        self.assertNotIn("TOPIC_JITTER:/plan", identifiers)
        self.assertNotIn("TOPIC_SINGLE_MESSAGE:/map", identifiers)
        self.assertNotIn("TOPIC_GAPS:/tf", identifiers)
        self.assertNotIn("TOPIC_JITTER:/tf", identifiers)

    def test_frequency_baseline_allows_small_timestamp_rounding_error(self) -> None:
        start = 1_700_000_000_000_000_000
        timestamps = [start + index * 200_001_000 for index in range(101)]
        bag = create_sqlite_bag(
            self.root / "scan_rounding",
            {"/scan": ("sensor_msgs/msg/LaserScan", timestamps, "")},
        )

        report = diagnose_bag(bag, DiagnosticOptions(deep_analysis=False))

        self.assertNotIn(
            "TOPIC_RATE_LOW:/scan",
            {issue.issue_id for issue in report.issues},
        )

    def test_invalid_thresholds_are_rejected(self) -> None:
        bag = create_sqlite_bag(
            self.root / "thresholds",
            {"/topic": ("example/msg/Data", [100, 200], "")},
        )
        with self.assertRaisesRegex(ValueError, "gap_factor"):
            diagnose_bag(bag, DiagnosticOptions(gap_factor=1.0))


if __name__ == "__main__":
    unittest.main()
