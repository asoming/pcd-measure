from __future__ import annotations

import tempfile
import shutil
import unittest
from pathlib import Path

try:
    from rosbags.rosbag1 import Writer as Ros1Writer
    from rosbags.rosbag2 import (
        CompressionFormat,
        CompressionMode,
        StoragePlugin,
        Writer as Ros2Writer,
    )
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    Ros1Writer = None

from rosbag_diag.analyzers import DiagnosticOptions, diagnose_bag
from rosbag_diag.model import UnsupportedBagError
from rosbag_diag.readers import detect_bag_kind


@unittest.skipIf(Ros1Writer is None, "optional rosbags package is not installed")
class OptionalRosbagsReaderTests(unittest.TestCase):
    def test_ros1_bag_is_read_and_deserialized(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bag_path = Path(temporary) / "sample.bag"
            typestore = get_typestore(Stores.ROS1_NOETIC)
            message_class = typestore.types["std_msgs/msg/String"]
            with Ros1Writer(bag_path) as writer:
                connection = writer.add_connection(
                    "/status", "std_msgs/msg/String", typestore=typestore
                )
                for index in range(20):
                    message = message_class(data=f"sample-{index}")
                    writer.write(
                        connection,
                        1_700_000_000_000_000_000 + index * 100_000_000,
                        typestore.serialize_ros1(message, "std_msgs/msg/String"),
                    )

            report = diagnose_bag(bag_path, DiagnosticOptions())

            self.assertEqual(detect_bag_kind(bag_path)[0], "ros1")
            self.assertEqual(report.bag["ros_version"], 1)
            self.assertEqual(report.bag["message_count"], 20)
            self.assertEqual(report.topics[0]["type"], "std_msgs/msg/String")
            self.assertFalse(any(
                issue.issue_id.startswith("HEADER_STAMP_MISSING")
                for issue in report.issues
            ))

    def test_ros2_mcap_is_read_and_deserialized(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bag_directory = Path(temporary) / "mcap_bag"
            typestore = get_typestore(Stores.ROS2_HUMBLE)
            message_class = typestore.types["std_msgs/msg/String"]
            with Ros2Writer(
                bag_directory,
                version=9,
                storage_plugin=StoragePlugin.MCAP,
            ) as writer:
                connection = writer.add_connection(
                    "/status", "std_msgs/msg/String", typestore=typestore
                )
                for index in range(10):
                    message = message_class(data=f"sample-{index}")
                    writer.write(
                        connection,
                        1_700_000_000_000_000_000 + index * 50_000_000,
                        typestore.serialize_cdr(message, "std_msgs/msg/String"),
                    )

            report = diagnose_bag(bag_directory, DiagnosticOptions())

            self.assertEqual(detect_bag_kind(bag_directory)[0], "ros2-mcap")
            self.assertEqual(report.bag["ros_version"], 2)
            self.assertEqual(report.bag["storage"], "mcap")
            self.assertEqual(report.bag["message_count"], 10)

            mcap_file = next(bag_directory.glob("*.mcap"))
            file_report = diagnose_bag(mcap_file, DiagnosticOptions())
            self.assertEqual(file_report.bag["message_count"], 10)

            import yaml

            second_mcap = bag_directory / "split_1.mcap"
            shutil.copy2(mcap_file, second_mcap)
            metadata_path = bag_directory / "metadata.yaml"
            metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
            bag_metadata = metadata["rosbag2_bagfile_information"]
            bag_metadata["relative_file_paths"].append(second_mcap.name)
            bag_metadata["message_count"] *= 2
            for topic in bag_metadata["topics_with_message_count"]:
                topic["message_count"] *= 2
            metadata_path.write_text(
                yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
            )
            split_report = diagnose_bag(bag_directory, DiagnosticOptions())
            self.assertEqual(split_report.bag["message_count"], 20)

    def test_compressed_ros2_sqlite_bag_is_read(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bag_directory = Path(temporary) / "compressed_bag"
            typestore = get_typestore(Stores.ROS2_HUMBLE)
            message_class = typestore.types["std_msgs/msg/String"]
            writer = Ros2Writer(
                bag_directory,
                version=9,
                storage_plugin=StoragePlugin.SQLITE3,
            )
            writer.set_compression(CompressionMode.FILE, CompressionFormat.ZSTD)
            with writer:
                connection = writer.add_connection(
                    "/status", "std_msgs/msg/String", typestore=typestore
                )
                for index in range(12):
                    message = message_class(data=f"compressed-{index}")
                    writer.write(
                        connection,
                        1_700_000_000_000_000_000 + index * 50_000_000,
                        typestore.serialize_cdr(message, "std_msgs/msg/String"),
                    )

            compressed_file = next(bag_directory.glob("*.db3.zstd"))
            self.assertEqual(detect_bag_kind(compressed_file)[0], "ros2-sqlite3")
            report = diagnose_bag(bag_directory, DiagnosticOptions())
            self.assertEqual(report.bag["message_count"], 12)
            self.assertEqual(report.bag["storage"], "sqlite3")
            self.assertEqual(report.bag["compression_mode"], "file")
            direct_report = diagnose_bag(compressed_file, DiagnosticOptions())
            self.assertEqual(direct_report.bag["message_count"], 12)

    def test_message_compressed_sqlite_and_corrupt_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag_directory = root / "message_compressed"
            typestore = get_typestore(Stores.ROS2_HUMBLE)
            message_class = typestore.types["std_msgs/msg/String"]
            writer = Ros2Writer(bag_directory, version=9)
            writer.set_compression(CompressionMode.MESSAGE, CompressionFormat.ZSTD)
            with writer:
                connection = writer.add_connection(
                    "/status", "std_msgs/msg/String", typestore=typestore
                )
                for index in range(4):
                    message = message_class(data=f"message-{index}")
                    writer.write(
                        connection,
                        1_700_000_000_000_000_000 + index * 10_000_000,
                        typestore.serialize_cdr(message, "std_msgs/msg/String"),
                    )
            report = diagnose_bag(bag_directory, DiagnosticOptions())
            self.assertEqual(report.bag["message_count"], 4)
            self.assertEqual(report.bag["compression_mode"], "message")

            for filename in ("broken.bag", "broken.mcap"):
                path = root / filename
                path.write_bytes(b"not a bag")
                with self.assertRaises(UnsupportedBagError):
                    diagnose_bag(path, DiagnosticOptions())


if __name__ == "__main__":
    unittest.main()
