#!/usr/bin/env python3
"""Record ROS 2 point clouds into an Odin-compatible OLX session.

The file layout follows the Apache-2.0 Odin ROS driver logger:
uint32 frame id, float64 seconds, uint32 count, then XYZRGBA (16 bytes/point).
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import shutil
import signal
import struct
import sys
import time
from datetime import datetime
from typing import Any


POINT_FIELD_INT8 = 1
POINT_FIELD_UINT8 = 2
POINT_FIELD_INT16 = 3
POINT_FIELD_UINT16 = 4
POINT_FIELD_INT32 = 5
POINT_FIELD_UINT32 = 6
POINT_FIELD_FLOAT32 = 7
POINT_FIELD_FLOAT64 = 8

_FIELD_SIZES = {
    POINT_FIELD_INT8: 1,
    POINT_FIELD_UINT8: 1,
    POINT_FIELD_INT16: 2,
    POINT_FIELD_UINT16: 2,
    POINT_FIELD_INT32: 4,
    POINT_FIELD_UINT32: 4,
    POINT_FIELD_FLOAT32: 4,
    POINT_FIELD_FLOAT64: 8,
}

_STRUCT_FORMATS = {
    POINT_FIELD_INT8: "b",
    POINT_FIELD_UINT8: "B",
    POINT_FIELD_INT16: "h",
    POINT_FIELD_UINT16: "H",
    POINT_FIELD_INT32: "i",
    POINT_FIELD_UINT32: "I",
    POINT_FIELD_FLOAT32: "f",
    POINT_FIELD_FLOAT64: "d",
}


def _timestamp_seconds(header: Any) -> float:
    stamp = header.stamp
    timestamp = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
    if not math.isfinite(timestamp) or timestamp < 0.0:
        raise ValueError("message timestamp must be finite and non-negative")
    return timestamp


def _validate_field(field: Any, point_step: int, name: str) -> int:
    datatype = int(field.datatype)
    size = _FIELD_SIZES.get(datatype)
    offset = int(field.offset)
    count = int(getattr(field, "count", 1))
    if size is None or offset < 0 or count != 1 or offset + size > point_step:
        raise ValueError(f"field {name!r} does not fit inside point_step")
    return datatype


def _contiguous_cloud_bytes(message: Any) -> memoryview:
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if width < 0 or height < 0 or point_step <= 0:
        raise ValueError("invalid PointCloud2 dimensions")
    expected_row = width * point_step
    if row_step < expected_row:
        raise ValueError("PointCloud2 row_step is smaller than width * point_step")
    raw = memoryview(message.data)
    required_bytes = row_step * height
    if len(raw) < required_bytes:
        raise ValueError(
            f"PointCloud2 data is truncated: expected {required_bytes} bytes, got {len(raw)}"
        )
    if height <= 1 or row_step == expected_row:
        return raw[: expected_row * height]
    compact = bytearray(expected_row * height)
    for row in range(height):
        source = row * row_step
        target = row * expected_row
        compact[target : target + expected_row] = raw[source : source + expected_row]
    return memoryview(compact)


def encode_cloud_frame(message: Any, frame_id: int) -> tuple[bytes, int]:
    """Encode one PointCloud2-like message; usable independently in tests."""
    fields = {str(field.name).lower(): field for field in message.fields}
    point_step = int(message.point_step)
    for name in ("x", "y", "z"):
        if name not in fields or _validate_field(fields[name], point_step, name) != POINT_FIELD_FLOAT32:
            raise ValueError("point cloud must contain float32 x/y/z fields")
    color_name = "rgba" if "rgba" in fields else ("rgb" if "rgb" in fields else "")
    intensity = fields.get("intensity")
    if color_name:
        color_datatype = _validate_field(fields[color_name], point_step, color_name)
        if _FIELD_SIZES[color_datatype] != 4:
            raise ValueError("rgb/rgba field must occupy four bytes")
    intensity_datatype = (
        _validate_field(intensity, point_step, "intensity") if intensity is not None else None
    )
    endian = ">" if bool(message.is_bigendian) else "<"
    width = int(message.width)
    height = int(message.height)
    if width < 0 or height < 0:
        raise ValueError("invalid PointCloud2 dimensions")
    point_count = width * height
    raw = _contiguous_cloud_bytes(message)

    try:
        import numpy as np

        names = ["x", "y", "z"]
        formats = [endian + "f4", endian + "f4", endian + "f4"]
        offsets = [int(fields[name].offset) for name in names]
        if color_name:
            names.append(color_name)
            formats.append(endian + "u4")
            offsets.append(int(fields[color_name].offset))
        elif intensity is not None:
            names.append("intensity")
            formats.append({
                POINT_FIELD_INT8: "i1",
                POINT_FIELD_UINT8: "u1",
                POINT_FIELD_INT16: endian + "i2",
                POINT_FIELD_UINT16: endian + "u2",
                POINT_FIELD_INT32: endian + "i4",
                POINT_FIELD_UINT32: endian + "u4",
                POINT_FIELD_FLOAT32: endian + "f4",
                POINT_FIELD_FLOAT64: endian + "f8",
            }[intensity_datatype])
            offsets.append(int(intensity.offset))
        dtype = np.dtype({
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": point_step,
        })
        source = np.frombuffer(raw, dtype=dtype, count=point_count)
        finite = np.isfinite(source["x"]) & np.isfinite(source["y"]) & np.isfinite(source["z"])
        source = source[finite]
        output = np.empty(len(source), dtype=np.dtype({
            "names": ["x", "y", "z", "r", "g", "b", "a"],
            "formats": ["<f4", "<f4", "<f4", "u1", "u1", "u1", "u1"],
            "offsets": [0, 4, 8, 12, 13, 14, 15],
            "itemsize": 16,
        }))
        output["x"] = source["x"]
        output["y"] = source["y"]
        output["z"] = source["z"]
        if color_name:
            packed = np.ascontiguousarray(source[color_name]).view(np.uint32)
            if bool(message.is_bigendian):
                packed = packed.byteswap()
            output["r"] = (packed >> 16) & 0xFF
            output["g"] = (packed >> 8) & 0xFF
            output["b"] = packed & 0xFF
            output["a"] = ((packed >> 24) & 0xFF) if color_name == "rgba" else 255
        elif intensity is not None:
            values = np.asarray(source["intensity"], dtype=np.float64)
            values = np.nan_to_num(values, nan=0.0, posinf=255.0, neginf=0.0)
            unit_scale_intensity = (
                intensity_datatype in (POINT_FIELD_FLOAT32, POINT_FIELD_FLOAT64)
                and len(values) > 0
                and values.min() >= 0.0
                and values.max() <= 1.0
            )
            if unit_scale_intensity:
                values = values * 255.0
            values = np.clip(values, 0, 255).astype(np.uint8)
            output["r"] = values
            output["g"] = values
            output["b"] = values
            output["a"] = 255
        else:
            output["r"] = 205
            output["g"] = 215
            output["b"] = 225
            output["a"] = 255
        payload = output.tobytes()
        valid_count = len(source)
    except ImportError:
        payload_buffer = bytearray()
        valid_count = 0
        xyz = [int(fields[name].offset) for name in ("x", "y", "z")]
        color_offset = int(fields[color_name].offset) if color_name else -1
        intensity_offset = int(intensity.offset) if intensity is not None else -1
        for index in range(point_count):
            base = index * point_step
            x, y, z = (struct.unpack_from(endian + "f", raw, base + offset)[0] for offset in xyz)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if color_offset >= 0:
                packed = struct.unpack_from(endian + "I", raw, base + color_offset)[0]
                r, g, b = (packed >> 16) & 255, (packed >> 8) & 255, packed & 255
                a = (packed >> 24) & 255 if color_name == "rgba" else 255
            elif intensity_offset >= 0:
                value = float(struct.unpack_from(
                    endian + _STRUCT_FORMATS[intensity_datatype], raw, base + intensity_offset
                )[0])
                if not math.isfinite(value):
                    value = 0.0
                if intensity_datatype in (POINT_FIELD_FLOAT32, POINT_FIELD_FLOAT64) and 0 <= value <= 1:
                    value *= 255.0
                gray = max(0, min(255, int(value)))
                r, g, b, a = gray, gray, gray, 255
            else:
                r, g, b, a = 205, 215, 225, 255
            payload_buffer.extend(struct.pack("<fffBBBB", x, y, z, r, g, b, a))
            valid_count += 1
        payload = bytes(payload_buffer)

    timestamp = _timestamp_seconds(message.header)
    return struct.pack("<IdI", int(frame_id), timestamp, valid_count) + payload, valid_count


def encode_pose_frame(message: Any, frame_id: int) -> bytes:
    pose = message.pose.pose
    values = (
        float(pose.position.x), float(pose.position.y), float(pose.position.z),
        float(pose.orientation.x), float(pose.orientation.y),
        float(pose.orientation.z), float(pose.orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose contains a non-finite position or quaternion")
    return struct.pack("<Id7f", int(frame_id), _timestamp_seconds(message.header), *values)


class SessionRecorder:
    def __init__(self, root: pathlib.Path, calibration: pathlib.Path | None = None) -> None:
        self.root = root
        self.image_directory = root / "image"
        self.cloud_path = root / f"MT{root.name}.olx"
        self.cloud_file = None
        self.pose_file = None
        self.image_file = None
        self._closed = False
        self.cloud_frames = 0
        self.pose_frames = 0
        self.image_frames = 0
        self.points = 0
        self.errors = 0
        self.started_at = time.monotonic()
        root.mkdir(parents=True, exist_ok=False)
        try:
            self.image_directory.mkdir()
            (root / "OdinRotate.bin").touch()
            (root / "OdinIMU.bin").touch()
            (self.image_directory / "info.txt").write_text(
                "device=OdinOne\npointcloud=xyzrgba\n"
                f"created_at={root.name}\nproducer=point-cloud-measure\n",
                encoding="utf-8",
            )
            target_calibration = self.image_directory / "cam_in_ex.txt"
            if calibration and calibration.is_file():
                shutil.copyfile(calibration, target_calibration)
            else:
                target_calibration.write_text("# calibration not supplied\n", encoding="utf-8")
            self.cloud_file = self.cloud_path.open("wb")
            self.pose_file = (root / "OdinPose.bin").open("wb")
            self.image_file = (root / "OdinImage.bin").open("wb")
        except Exception:
            self.close()
            shutil.rmtree(root, ignore_errors=True)
            raise

    def write_cloud(self, message: Any) -> None:
        frame, valid = encode_cloud_frame(message, self.cloud_frames)
        self.cloud_file.write(frame)
        self.cloud_frames += 1
        self.points += valid

    def write_pose(self, message: Any) -> None:
        self.pose_file.write(encode_pose_frame(message, self.pose_frames))
        self.pose_frames += 1

    def write_image(self, message: Any) -> None:
        payload = bytes(message.data)
        if not payload:
            raise ValueError("compressed image payload is empty")
        timestamp = _timestamp_seconds(message.header)
        self.image_file.write(struct.pack("<IdI", self.image_frames, timestamp, len(payload)))
        self.image_file.write(payload)
        image_format = str(getattr(message, "format", "")).lower()
        if "jpeg" in image_format or "jpg" in image_format or payload.startswith(b"\xff\xd8"):
            (self.image_directory / f"{self.image_frames:06d}.jpg").write_bytes(payload)
        elif "png" in image_format or payload.startswith(b"\x89PNG\r\n\x1a\n"):
            (self.image_directory / f"{self.image_frames:06d}.png").write_bytes(payload)
        self.image_frames += 1

    def status(self, final: bool = False) -> dict[str, Any]:
        return {
            "session": str(self.root),
            "olx": str(self.cloud_path),
            "cloud_frames": self.cloud_frames,
            "pose_frames": self.pose_frames,
            "image_frames": self.image_frames,
            "points": self.points,
            "errors": self.errors,
            "elapsed_seconds": round(time.monotonic() - self.started_at, 3),
            "final": final,
        }

    def flush(self) -> None:
        for output in (self.cloud_file, self.pose_file, self.image_file):
            if output is not None and not output.closed:
                output.flush()

    def close(self) -> None:
        if self._closed:
            return
        self.flush()
        for output in (self.cloud_file, self.pose_file, self.image_file):
            if output is not None and not output.closed:
                output.close()
        self._closed = True


def _safe_session_name(value: str) -> str:
    candidate = value.strip() or datetime.now().strftime("%Y%m%d_%H%M%S")
    if candidate in {".", ".."} or not re.fullmatch(r"[A-Za-z0-9._-]+", candidate):
        raise ValueError("session name may contain only letters, numbers, dot, underscore, and dash")
    return candidate


def _next_session_root(parent: pathlib.Path, requested_name: str) -> pathlib.Path:
    base_name = _safe_session_name(requested_name)
    first = parent / base_name
    if not first.exists():
        return first
    for suffix in range(1, 10_000):
        candidate = parent / f"{base_name}-{suffix:02d}"
        if not candidate.exists():
            return candidate
    raise FileExistsError(f"too many sessions named {base_name!r} in {parent}")


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record ROS 2 PointCloud2 data as an OLX session")
    parser.add_argument("--output-dir", required=True, help="parent directory for the new session")
    parser.add_argument("--session-name", default="", help="safe folder name; defaults to current time")
    parser.add_argument("--cloud-topic", default="/odin1/cloud_slam")
    parser.add_argument("--pose-topic", default="/odin1/odometry")
    parser.add_argument("--image-topic", default="/odin1/image/compressed")
    parser.add_argument("--no-pose", action="store_true")
    parser.add_argument("--no-image", action="store_true")
    parser.add_argument("--calibration", default="")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds; zero means manual")
    parser.add_argument("--max-frames", type=int, default=0, help="stop after N cloud frames; zero means manual")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(list(argv if argv is not None else sys.argv[1:]))
    if args.duration < 0 or args.max_frames < 0:
        raise SystemExit("duration and max-frames must not be negative")

    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage, PointCloud2
    except ImportError as error:
        print(f"ERROR ROS 2 Python dependencies are unavailable: {error}", file=sys.stderr, flush=True)
        return 2

    try:
        output_directory = pathlib.Path(args.output_dir).expanduser().resolve()
        output_directory.mkdir(parents=True, exist_ok=True)
        session_root = _next_session_root(output_directory, args.session_name)
        calibration = pathlib.Path(args.calibration).expanduser().resolve() if args.calibration else None
        recorder = SessionRecorder(session_root, calibration)
    except (OSError, ValueError) as error:
        print(f"ERROR cannot create OLX session: {error}", file=sys.stderr, flush=True)
        return 3

    print("SESSION " + str(session_root), flush=True)
    stop = False
    last_status = 0.0
    node = None
    runtime_error = False

    def request_stop(*_: Any) -> None:
        nonlocal stop
        stop = True

    def cloud_callback(message: Any) -> None:
        nonlocal stop
        try:
            recorder.write_cloud(message)
            if args.max_frames and recorder.cloud_frames >= args.max_frames:
                stop = True
        except Exception as error:  # keep the session recoverable after a malformed frame
            recorder.errors += 1
            print(f"ERROR cloud frame: {error}", file=sys.stderr, flush=True)

    def pose_callback(message: Any) -> None:
        try:
            recorder.write_pose(message)
        except Exception as error:
            recorder.errors += 1
            print(f"ERROR pose frame: {error}", file=sys.stderr, flush=True)

    def image_callback(message: Any) -> None:
        try:
            recorder.write_image(message)
        except Exception as error:
            recorder.errors += 1
            print(f"ERROR image frame: {error}", file=sys.stderr, flush=True)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        rclpy.init(args=None)
        node = rclpy.create_node("point_cloud_olx_recorder")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        node.create_subscription(PointCloud2, args.cloud_topic, cloud_callback, qos)
        if not args.no_pose and args.pose_topic:
            node.create_subscription(Odometry, args.pose_topic, pose_callback, qos)
        if not args.no_image and args.image_topic:
            node.create_subscription(CompressedImage, args.image_topic, image_callback, qos)
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.monotonic() - recorder.started_at
            if args.duration and elapsed >= args.duration:
                stop = True
            if elapsed - last_status >= 1.0:
                recorder.flush()
                print("STATUS " + json.dumps(recorder.status(), ensure_ascii=False), flush=True)
                last_status = elapsed
    except Exception as error:
        runtime_error = True
        recorder.errors += 1
        print(f"ERROR ROS 2 recorder failed: {error}", file=sys.stderr, flush=True)
    finally:
        recorder.close()
        print("STATUS " + json.dumps(recorder.status(final=True), ensure_ascii=False), flush=True)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if runtime_error:
        return 3
    return 0 if recorder.cloud_frames > 0 else 4


if __name__ == "__main__":
    raise SystemExit(main())
