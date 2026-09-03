"""ROS1/ROS2 bag reader adapters.

ROS2 SQLite timing diagnostics use only Python's standard library.  Payload
decoding uses the native ROS2 stack when available.  ROS1 and MCAP use the
optional ``rosbags`` package so the desktop application itself remains free of
compile-time ROS dependencies.
"""

from __future__ import annotations

import json
import os
import sqlite3
from abc import ABC, abstractmethod
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from .model import (
    BagInformation,
    DecodedRecord,
    MissingDependencyError,
    TopicDefinition,
    UnsupportedBagError,
)


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise MissingDependencyError(
            "读取 ROS2 metadata.yaml 需要 PyYAML；请运行 scripts/setup_rosbag_tools.sh。"
        ) from exc
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise UnsupportedBagError(f"无法读取 metadata.yaml：{exc}") from exc
    return payload if isinstance(payload, dict) else {}


def _stringify_qos(value: Any) -> str:
    if value in (None, ""):
        return ""
    if isinstance(value, str):
        return value
    try:
        return json.dumps(value, ensure_ascii=False, default=str)
    except (TypeError, ValueError):
        return str(value)


def _sqlite_connection(path: Path) -> sqlite3.Connection:
    try:
        connection = sqlite3.connect(f"{path.resolve().as_uri()}?mode=ro", uri=True)
    except sqlite3.Error as exc:
        raise UnsupportedBagError(f"无法只读打开 ROS2 SQLite：{path}：{exc}") from exc
    connection.execute("PRAGMA query_only = ON")
    return connection


class BagSource(ABC):
    """Format-independent source used by the analyzer."""

    info: BagInformation
    decode_warnings: list[str]

    @abstractmethod
    def iter_timestamps(self) -> Iterator[tuple[str, int]]:
        """Yield topic and storage timestamp in recorded order."""

    @abstractmethod
    def iter_decoded_samples(self, maximum_per_topic: int) -> Iterator[DecodedRecord]:
        """Yield a bounded sample of decoded messages."""


class _NativeRos2Decoder:
    """Lazy ROS2 CDR decoder with per-message-type failure isolation."""

    def __init__(self) -> None:
        self._classes: dict[str, type[Any] | None] = {}
        self._failure_reasons: dict[str, str] = {}
        try:
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
        except ImportError as exc:
            self._deserialize_message = None
            self._get_message = None
            self._failure_reasons["*"] = str(exc)
        else:
            self._deserialize_message = deserialize_message
            self._get_message = get_message

    @property
    def available(self) -> bool:
        return self._deserialize_message is not None and self._get_message is not None

    def decode(self, msgtype: str, data: bytes) -> Any | None:
        if not self.available:
            return None
        if msgtype not in self._classes:
            try:
                assert self._get_message is not None
                self._classes[msgtype] = self._get_message(msgtype)
            except (AttributeError, ImportError, ModuleNotFoundError, RuntimeError, ValueError) as exc:
                self._classes[msgtype] = None
                self._failure_reasons[msgtype] = str(exc)
        message_class = self._classes[msgtype]
        if message_class is None:
            return None
        try:
            assert self._deserialize_message is not None
            return self._deserialize_message(data, message_class)
        except (RuntimeError, TypeError, ValueError) as exc:
            self._failure_reasons.setdefault(msgtype, str(exc))
            return None

    def warnings(self) -> list[str]:
        if "*" in self._failure_reasons:
            return [f"ROS2 消息反序列化不可用：{self._failure_reasons['*']}"]
        return [
            f"消息类型 {msgtype} 无法反序列化：{reason}"
            for msgtype, reason in sorted(self._failure_reasons.items())
        ]


class SqliteBagSource(BagSource):
    """ROS2 SQLite reader with no non-standard requirement for timing data."""

    def __init__(self, selected_path: Path) -> None:
        self.decode_warnings = []
        self._selected_path = selected_path.resolve()
        self._metadata_path = self._find_metadata_path(self._selected_path)
        metadata = _load_yaml(self._metadata_path) if self._metadata_path else {}
        bag_metadata = metadata.get("rosbag2_bagfile_information", metadata)
        if not isinstance(bag_metadata, dict):
            bag_metadata = {}

        files = self._resolve_files(self._selected_path, bag_metadata)
        if not files:
            raise UnsupportedBagError("ROS2 bag 中没有找到可读取的 .db3 文件。")
        for file_path in files:
            if not file_path.is_file():
                raise UnsupportedBagError(f"metadata 引用的 bag 分片不存在：{file_path}")

        topics = self._metadata_topics(bag_metadata)
        database_topics, database_count, database_start, database_end = self._inspect_databases(files)
        for name, definition in database_topics.items():
            existing = topics.get(name)
            if existing is None:
                topics[name] = definition
            elif not existing.declared_count:
                topics[name] = TopicDefinition(
                    name=existing.name,
                    msgtype=existing.msgtype or definition.msgtype,
                    serialization_format=existing.serialization_format
                    or definition.serialization_format,
                    offered_qos_profiles=existing.offered_qos_profiles,
                    declared_count=definition.declared_count,
                )

        metadata_count = int(bag_metadata.get("message_count", 0) or 0)
        start_ns = _nested_int(bag_metadata, "starting_time", "nanoseconds_since_epoch")
        duration_ns = _nested_int(bag_metadata, "duration", "nanoseconds") or 0
        if start_ns is None:
            start_ns = database_start
        end_ns = start_ns + duration_ns if start_ns is not None and duration_ns else database_end
        if not duration_ns and start_ns is not None and end_ns is not None:
            duration_ns = max(0, end_ns - start_ns)

        root_path = (
            self._selected_path
            if self._selected_path.is_dir() or self._metadata_path is None
            else self._selected_path.parent
        )
        self.info = BagInformation(
            path=root_path,
            ros_version=2,
            storage=str(bag_metadata.get("storage_identifier", "sqlite3") or "sqlite3"),
            reader="sqlite3 + native ROS2 decoder",
            files=files,
            topics=topics,
            size_bytes=sum(path.stat().st_size for path in files),
            message_count=metadata_count or database_count,
            start_time_ns=start_ns,
            end_time_ns=end_ns,
            duration_ns=duration_ns,
            compression_format=str(bag_metadata.get("compression_format", "") or ""),
            compression_mode=str(bag_metadata.get("compression_mode", "") or ""),
        )
        if metadata_count and metadata_count != database_count:
            self.info.metadata_warnings.append(
                f"metadata 消息数 {metadata_count} 与数据库实际消息数 {database_count} 不一致。"
            )

    @staticmethod
    def _find_metadata_path(selected_path: Path) -> Path | None:
        if selected_path.is_dir() and (selected_path / "metadata.yaml").is_file():
            return selected_path / "metadata.yaml"
        if selected_path.is_file() and (selected_path.parent / "metadata.yaml").is_file():
            return selected_path.parent / "metadata.yaml"
        return None

    @staticmethod
    def _resolve_files(selected_path: Path, metadata: dict[str, Any]) -> list[Path]:
        base = selected_path if selected_path.is_dir() else selected_path.parent
        relative_paths = metadata.get("relative_file_paths", [])
        if isinstance(relative_paths, list):
            resolved = [base / str(item) for item in relative_paths if str(item).endswith(".db3")]
            if resolved:
                return resolved
        if selected_path.is_file() and selected_path.suffix.lower() == ".db3":
            return [selected_path]
        return sorted(base.glob("*.db3"))

    @staticmethod
    def _metadata_topics(metadata: dict[str, Any]) -> dict[str, TopicDefinition]:
        topics: dict[str, TopicDefinition] = {}
        raw_topics = metadata.get("topics_with_message_count", [])
        if not isinstance(raw_topics, list):
            return topics
        for entry in raw_topics:
            if not isinstance(entry, dict):
                continue
            topic = entry.get("topic_metadata", {})
            if not isinstance(topic, dict):
                continue
            name = str(topic.get("name", ""))
            if not name:
                continue
            topics[name] = TopicDefinition(
                name=name,
                msgtype=str(topic.get("type", "")),
                serialization_format=str(topic.get("serialization_format", "")),
                offered_qos_profiles=_stringify_qos(topic.get("offered_qos_profiles", "")),
                declared_count=int(entry.get("message_count", 0) or 0),
            )
        return topics

    @staticmethod
    def _inspect_databases(
        files: list[Path],
    ) -> tuple[dict[str, TopicDefinition], int, int | None, int | None]:
        topics: dict[str, TopicDefinition] = {}
        total_count = 0
        start_ns: int | None = None
        end_ns: int | None = None
        for file_path in files:
            with _sqlite_connection(file_path) as database:
                try:
                    rows = database.execute(
                        "SELECT id, name, type, serialization_format FROM topics"
                    ).fetchall()
                    counts = dict(
                        database.execute(
                            "SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id"
                        ).fetchall()
                    )
                    time_row = database.execute(
                        "SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages"
                    ).fetchone()
                except sqlite3.Error as exc:
                    raise UnsupportedBagError(
                        f"{file_path.name} 不是兼容的 ROS2 SQLite bag：{exc}"
                    ) from exc
                for topic_id, name, msgtype, serialization_format in rows:
                    previous = topics.get(name)
                    count = (previous.declared_count if previous else 0) + int(
                        counts.get(topic_id, 0)
                    )
                    topics[name] = TopicDefinition(
                        name=name,
                        msgtype=msgtype,
                        serialization_format=serialization_format or "",
                        declared_count=count,
                    )
                if time_row:
                    count, minimum, maximum = time_row
                    total_count += int(count or 0)
                    if minimum is not None:
                        start_ns = int(minimum) if start_ns is None else min(start_ns, int(minimum))
                    if maximum is not None:
                        end_ns = int(maximum) if end_ns is None else max(end_ns, int(maximum))
        return topics, total_count, start_ns, end_ns

    def iter_timestamps(self) -> Iterator[tuple[str, int]]:
        for file_path in self.info.files:
            with _sqlite_connection(file_path) as database:
                try:
                    cursor = database.execute(
                        "SELECT topics.name, messages.timestamp "
                        "FROM messages JOIN topics ON topics.id = messages.topic_id "
                        "ORDER BY messages.id"
                    )
                    for topic, timestamp_ns in cursor:
                        yield str(topic), int(timestamp_ns)
                except sqlite3.Error as exc:
                    raise UnsupportedBagError(
                        f"读取 {file_path.name} 的时间索引失败：{exc}"
                    ) from exc

    def iter_decoded_samples(self, maximum_per_topic: int) -> Iterator[DecodedRecord]:
        if maximum_per_topic <= 0:
            return
        decoder = _NativeRos2Decoder()
        if not decoder.available:
            self.decode_warnings.extend(decoder.warnings())
            return

        yielded: dict[str, int] = {}
        for file_path in self.info.files:
            with _sqlite_connection(file_path) as database:
                try:
                    topic_rows = database.execute(
                        "SELECT topics.id, topics.name, topics.type, COUNT(messages.id) "
                        "FROM topics LEFT JOIN messages ON messages.topic_id = topics.id "
                        "GROUP BY topics.id, topics.name, topics.type"
                    ).fetchall()
                    for topic_id, topic, msgtype, file_count in topic_rows:
                        topic = str(topic)
                        remaining = maximum_per_topic - yielded.get(topic, 0)
                        if remaining <= 0 or not file_count:
                            continue
                        stride = max(1, (int(file_count) + remaining - 1) // remaining)
                        cursor = database.execute(
                            "WITH ranked AS ("
                            " SELECT timestamp, data, ROW_NUMBER() OVER (ORDER BY id) - 1 AS sample_index"
                            " FROM messages WHERE topic_id = ?"
                            ") SELECT timestamp, data FROM ranked "
                            "WHERE sample_index % ? = 0 ORDER BY sample_index LIMIT ?",
                            (topic_id, stride, remaining),
                        )
                        for timestamp_ns, data in cursor:
                            message = decoder.decode(str(msgtype), bytes(data))
                            if message is None:
                                continue
                            yielded[topic] = yielded.get(topic, 0) + 1
                            yield DecodedRecord(
                                topic=topic,
                                msgtype=str(msgtype),
                                timestamp_ns=int(timestamp_ns),
                                message=message,
                            )
                except sqlite3.Error as exc:
                    self.decode_warnings.append(
                        f"读取 {file_path.name} 的消息载荷失败：{exc}"
                    )
        self.decode_warnings.extend(decoder.warnings())


class RosbagsSource(BagSource):
    """Reader adapter for ROS1 bag and ROS2 MCAP using ``rosbags``."""

    def __init__(self, selected_path: Path, ros_version: int) -> None:
        self.decode_warnings = []
        self._path = selected_path.resolve()
        self._ros_version = ros_version
        self._storage = "rosbag1" if ros_version == 1 else "mcap"
        self._compression_format = ""
        self._compression_mode = ""
        self._files = [self._path]
        if ros_version == 2 and self._path.is_dir():
            metadata_path = self._path / "metadata.yaml"
            if metadata_path.is_file():
                metadata = _load_yaml(metadata_path)
                bag_metadata = metadata.get("rosbag2_bagfile_information", metadata)
                if isinstance(bag_metadata, dict):
                    self._storage = str(
                        bag_metadata.get("storage_identifier", "mcap") or "mcap"
                    )
                    self._compression_format = str(
                        bag_metadata.get("compression_format", "") or ""
                    )
                    self._compression_mode = str(
                        bag_metadata.get("compression_mode", "") or ""
                    )
                    relative_paths = bag_metadata.get("relative_file_paths", [])
                    if isinstance(relative_paths, list):
                        resolved = [self._path / str(item) for item in relative_paths]
                        if resolved:
                            self._files = resolved
        self._any_reader, self._default_typestore = self._load_rosbags()
        self.info = self._inspect()

    def _load_rosbags(self) -> tuple[type[Any], Any]:
        try:
            from rosbags.highlevel import AnyReader
            from rosbags.typesys import Stores, get_typestore
        except ImportError as exc:
            raise MissingDependencyError(
                "ROS1/MCAP 深度解析需要 rosbags；请运行 scripts/setup_rosbag_tools.sh。"
            ) from exc
        store = Stores.ROS1_NOETIC if self._ros_version == 1 else Stores.ROS2_HUMBLE
        return AnyReader, get_typestore(store)

    def _reader(self) -> Any:
        return self._any_reader([self._path], default_typestore=self._default_typestore)

    @contextmanager
    def _opened_reader(self) -> Iterator[Any]:
        """Open AnyReader and close storage even when malformed input fails early."""

        reader = self._reader()
        try:
            reader.open()
        except Exception:
            # AnyReader only rolls back readers that opened completely. A
            # corrupt ROS1/MCAP file can fail after its file handle is opened,
            # so explicitly close that partially initialized child as well.
            for child in getattr(reader, "readers", []):
                storage = getattr(child, "storage", None)
                try:
                    if storage is not None:
                        storage.close()
                    elif getattr(child, "bio", None) is not None:
                        child.close()
                except Exception:
                    pass
            raise
        try:
            yield reader
        finally:
            reader.close()

    def _inspect(self) -> BagInformation:
        try:
            with self._opened_reader() as reader:
                topics: dict[str, TopicDefinition] = {}
                for connection in reader.connections:
                    qos = _stringify_qos(
                        getattr(getattr(connection, "ext", None), "offered_qos_profiles", "")
                    )
                    topics[connection.topic] = TopicDefinition(
                        name=connection.topic,
                        msgtype=connection.msgtype,
                        serialization_format="ros1" if self._ros_version == 1 else "cdr",
                        offered_qos_profiles=qos,
                        declared_count=int(getattr(connection, "msgcount", 0) or 0),
                    )
                start_ns = int(reader.start_time) if reader.message_count else None
                end_ns = int(reader.end_time) if reader.message_count else None
                return BagInformation(
                    path=self._path,
                    ros_version=self._ros_version,
                    storage=self._storage,
                    reader="rosbags AnyReader",
                    files=self._files,
                    topics=topics,
                    size_bytes=self._path.stat().st_size if self._path.is_file() else _directory_size(self._path),
                    message_count=int(reader.message_count),
                    start_time_ns=start_ns,
                    end_time_ns=end_ns,
                    duration_ns=max(0, (end_ns or 0) - (start_ns or 0)),
                    compression_format=self._compression_format,
                    compression_mode=self._compression_mode,
                )
        except Exception as exc:
            # Reader exceptions differ between rosbag1, rosbag2 and MCAP.
            # Convert all malformed-input failures at this adapter boundary to
            # one stable user-facing error while leaving BaseException alone.
            raise UnsupportedBagError(f"无法读取 bag：{exc}") from exc

    def iter_timestamps(self) -> Iterator[tuple[str, int]]:
        try:
            with self._opened_reader() as reader:
                for connection, timestamp_ns, _ in reader.messages():
                    yield connection.topic, int(timestamp_ns)
        except Exception as exc:
            raise UnsupportedBagError(f"读取 bag 时间索引失败：{exc}") from exc

    def iter_decoded_samples(self, maximum_per_topic: int) -> Iterator[DecodedRecord]:
        if maximum_per_topic <= 0:
            return
        seen: dict[str, int] = {}
        yielded: dict[str, int] = {}
        strides = {
            name: max(
                1,
                (definition.declared_count + maximum_per_topic - 1) // maximum_per_topic,
            )
            for name, definition in self.info.topics.items()
        }
        failed_types: dict[str, str] = {}
        try:
            with self._opened_reader() as reader:
                for connection, timestamp_ns, raw_data in reader.messages():
                    topic = connection.topic
                    index = seen.get(topic, 0)
                    seen[topic] = index + 1
                    if index % strides.get(topic, 1) != 0:
                        continue
                    if yielded.get(topic, 0) >= maximum_per_topic:
                        continue
                    try:
                        message = reader.deserialize(raw_data, connection.msgtype)
                    except Exception as exc:
                        failed_types.setdefault(connection.msgtype, str(exc))
                        continue
                    yielded[topic] = yielded.get(topic, 0) + 1
                    yield DecodedRecord(
                        topic=topic,
                        msgtype=connection.msgtype,
                        timestamp_ns=int(timestamp_ns),
                        message=message,
                    )
        except Exception as exc:
            self.decode_warnings.append(f"bag 消息载荷读取中断：{exc}")
        self.decode_warnings.extend(
            f"消息类型 {msgtype} 无法反序列化：{reason}"
            for msgtype, reason in sorted(failed_types.items())
        )


def _nested_int(mapping: dict[str, Any], key: str, nested_key: str) -> int | None:
    value = mapping.get(key, {})
    if not isinstance(value, dict) or value.get(nested_key) is None:
        return None
    try:
        return int(value[nested_key])
    except (TypeError, ValueError):
        return None


def _directory_size(path: Path) -> int:
    total = 0
    for child in path.rglob("*"):
        try:
            if child.is_file():
                total += child.stat().st_size
        except OSError:
            continue
    return total


def detect_bag_kind(path: str | os.PathLike[str]) -> tuple[str, Path]:
    """Return a stable bag kind and normalized selected path."""

    selected = Path(path).expanduser().resolve()
    if not selected.exists():
        raise UnsupportedBagError(f"文件或目录不存在：{selected}")
    suffix = selected.suffix.lower()
    lower_name = selected.name.lower()
    if selected.is_file() and suffix == ".bag":
        return "ros1", selected
    if selected.is_file() and (
        suffix == ".db3"
        or lower_name.endswith(".db3.zstd")
        or lower_name.endswith(".db3.lz4")
    ):
        return "ros2-sqlite3", selected
    if selected.is_file() and suffix == ".mcap":
        return "ros2-mcap", selected
    if selected.is_dir():
        metadata_path = selected / "metadata.yaml"
        if metadata_path.is_file():
            metadata = _load_yaml(metadata_path)
            bag_metadata = metadata.get("rosbag2_bagfile_information", metadata)
            storage = str(
                bag_metadata.get("storage_identifier", "")
                if isinstance(bag_metadata, dict)
                else ""
            ).lower()
            if (
                storage == "sqlite3"
                or list(selected.glob("*.db3"))
                or list(selected.glob("*.db3.zstd"))
                or list(selected.glob("*.db3.lz4"))
            ):
                return "ros2-sqlite3", selected
            if storage == "mcap" or list(selected.glob("*.mcap")):
                return "ros2-mcap", selected
    raise UnsupportedBagError(
        "请选择 ROS1 .bag、ROS2 bag 目录、.db3 或 .mcap 文件。"
    )


def open_bag(path: str | os.PathLike[str]) -> BagSource:
    """Open a bag using the smallest capable adapter."""

    kind, selected = detect_bag_kind(path)
    if kind == "ros1":
        return RosbagsSource(selected, ros_version=1)
    if kind == "ros2-mcap":
        # Passing the bag directory lets AnyReader honor metadata.yaml and read
        # every MCAP split as one recording. A directly selected .mcap remains
        # useful for recordings that have no metadata file.
        return RosbagsSource(selected, ros_version=2)

    metadata_path = (
        selected / "metadata.yaml"
        if selected.is_dir()
        else selected.parent / "metadata.yaml"
    )
    if metadata_path.is_file():
        metadata = _load_yaml(metadata_path)
        bag_metadata = metadata.get("rosbag2_bagfile_information", metadata)
        if isinstance(bag_metadata, dict):
            compression_mode = str(bag_metadata.get("compression_mode", "") or "").lower()
            compression_format = str(bag_metadata.get("compression_format", "") or "").lower()
            if compression_mode not in {"", "none"} or compression_format:
                try:
                    return RosbagsSource(metadata_path.parent, ros_version=2)
                except MissingDependencyError as exc:
                    raise MissingDependencyError(
                        "该 ROS2 bag 使用压缩存储，需先运行 scripts/setup_rosbag_tools.sh。"
                    ) from exc

    if selected.is_file() and selected.suffix.lower() in {".zstd", ".lz4"}:
        raise UnsupportedBagError("压缩的 ROS2 SQLite 分片必须与 metadata.yaml 一起按目录诊断。")

    source = SqliteBagSource(selected)
    if source.info.compression_mode or source.info.compression_format:
        # Compressed rosbag2 storage is delegated to rosbags, which handles the
        # compression stream before exposing records.
        try:
            return RosbagsSource(source.info.path, ros_version=2)
        except MissingDependencyError as exc:
            raise MissingDependencyError(
                "该 ROS2 bag 使用压缩存储，需先运行 scripts/setup_rosbag_tools.sh。"
            ) from exc
    return source
