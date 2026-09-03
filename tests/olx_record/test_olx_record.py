import builtins
import math
import pathlib
import struct
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

import olx_record


def header(seconds: int = 12, nanoseconds: int = 250_000_000):
    return SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds))


def cloud_message(points, *, big_endian=False):
    endian = ">" if big_endian else "<"
    payload = bytearray()
    for x, y, z, r, g, b in points:
        packed = (r << 16) | (g << 8) | b
        payload.extend(struct.pack(endian + "fffI", x, y, z, packed))
    fields = [
        SimpleNamespace(name="x", offset=0, datatype=7),
        SimpleNamespace(name="y", offset=4, datatype=7),
        SimpleNamespace(name="z", offset=8, datatype=7),
        SimpleNamespace(name="rgb", offset=12, datatype=7),
    ]
    return SimpleNamespace(
        header=header(), fields=fields, data=payload, width=len(points), height=1,
        point_step=16, row_step=16 * len(points), is_bigendian=big_endian,
    )


def field(name, offset, datatype, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def message_from_rows(rows, fields, *, point_step, row_padding=0, big_endian=False):
    height = len(rows)
    width = len(rows[0]) if rows else 0
    payload = bytearray()
    for row in rows:
        if len(row) != width:
            raise ValueError("test rows must have equal width")
        payload.extend(b"".join(row))
        payload.extend(b"\xEE" * row_padding)
    return SimpleNamespace(
        header=header(), fields=fields, data=payload, width=width, height=height,
        point_step=point_step, row_step=width * point_step + row_padding,
        is_bigendian=big_endian,
    )


class OlxRecordTest(unittest.TestCase):
    def test_cloud_frame_is_exact_xyzrgba_layout(self):
        message = cloud_message([
            (1.0, 2.0, 3.0, 10, 20, 30),
            (math.nan, 4.0, 5.0, 40, 50, 60),
            (-1.0, -2.0, -3.0, 70, 80, 90),
        ])
        encoded, count = olx_record.encode_cloud_frame(message, 9)
        self.assertEqual(count, 2)
        self.assertEqual(len(encoded), 16 + 2 * 16)
        self.assertEqual(struct.unpack_from("<IdI", encoded), (9, 12.25, 2))
        self.assertEqual(struct.unpack_from("<fffBBBB", encoded, 16),
                         (1.0, 2.0, 3.0, 10, 20, 30, 255))
        self.assertEqual(struct.unpack_from("<fffBBBB", encoded, 32),
                         (-1.0, -2.0, -3.0, 70, 80, 90, 255))

    def test_big_endian_input_is_normalized(self):
        encoded, count = olx_record.encode_cloud_frame(
            cloud_message([(1.5, -2.5, 3.5, 7, 8, 9)], big_endian=True), 2)
        self.assertEqual(count, 1)
        self.assertEqual(struct.unpack_from("<fffBBBB", encoded, 16),
                         (1.5, -2.5, 3.5, 7, 8, 9, 255))

    def test_row_padding_and_float_intensity_are_supported(self):
        rows = [
            [struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0),
             struct.pack("<ffff", 1.0, 0.0, 0.0, 0.25)],
            [struct.pack("<ffff", 0.0, 1.0, 0.0, 0.5),
             struct.pack("<ffff", 1.0, 1.0, 0.0, 1.0)],
        ]
        message = message_from_rows(
            rows,
            [field("x", 0, 7), field("y", 4, 7), field("z", 8, 7),
             field("intensity", 12, 7)],
            point_step=16,
            row_padding=8,
        )

        encoded, count = olx_record.encode_cloud_frame(message, 3)

        self.assertEqual(count, 4)
        gray = [struct.unpack_from("<fffBBBB", encoded, 16 + index * 16)[3]
                for index in range(4)]
        self.assertEqual(gray, [0, 63, 127, 255])

    def test_rgba_alpha_and_neutral_fallback(self):
        rgba_message = message_from_rows(
            [[struct.pack("<fffI", 1.0, 2.0, 3.0, 0x7F0A141E)]],
            [field("x", 0, 7), field("y", 4, 7), field("z", 8, 7),
             field("rgba", 12, 6)],
            point_step=16,
        )
        rgba, _ = olx_record.encode_cloud_frame(rgba_message, 0)
        neutral_message = message_from_rows(
            [[struct.pack("<fff", 4.0, 5.0, 6.0)]],
            [field("x", 0, 7), field("y", 4, 7), field("z", 8, 7)],
            point_step=12,
        )
        neutral, _ = olx_record.encode_cloud_frame(neutral_message, 1)

        self.assertEqual(struct.unpack_from("<fffBBBB", rgba, 16)[3:], (10, 20, 30, 127))
        self.assertEqual(struct.unpack_from("<fffBBBB", neutral, 16)[3:],
                         (205, 215, 225, 255))

    def test_zero_point_frame_and_malformed_layouts(self):
        empty = SimpleNamespace(
            header=header(), fields=[field("x", 0, 7), field("y", 4, 7), field("z", 8, 7)],
            data=b"", width=0, height=1, point_step=12, row_step=0, is_bigendian=False,
        )
        encoded, count = olx_record.encode_cloud_frame(empty, 8)
        self.assertEqual(count, 0)
        self.assertEqual(len(encoded), 16)

        truncated = cloud_message([(1.0, 2.0, 3.0, 1, 2, 3)])
        truncated.data = truncated.data[:-1]
        with self.assertRaisesRegex(ValueError, "truncated"):
            olx_record.encode_cloud_frame(truncated, 0)

        bad_row = cloud_message([(1.0, 2.0, 3.0, 1, 2, 3)])
        bad_row.row_step = 4
        with self.assertRaisesRegex(ValueError, "row_step"):
            olx_record.encode_cloud_frame(bad_row, 0)

        bad_field = cloud_message([(1.0, 2.0, 3.0, 1, 2, 3)])
        bad_field.fields[0].offset = 15
        with self.assertRaisesRegex(ValueError, "does not fit"):
            olx_record.encode_cloud_frame(bad_field, 0)

    def test_non_finite_timestamp_is_rejected(self):
        message = cloud_message([(1.0, 2.0, 3.0, 1, 2, 3)])
        message.header.stamp.sec = math.nan
        with self.assertRaisesRegex(ValueError, "timestamp"):
            olx_record.encode_cloud_frame(message, 0)

    def test_stdlib_fallback_encodes_intensity_without_numpy(self):
        message = message_from_rows(
            [[struct.pack("<ffff", 1.0, 2.0, 3.0, 0.5)]],
            [field("x", 0, 7), field("y", 4, 7), field("z", 8, 7),
             field("intensity", 12, 7)],
            point_step=16,
        )
        real_import = builtins.__import__

        def import_without_numpy(name, *args, **kwargs):
            if name == "numpy":
                raise ImportError("disabled for fallback test")
            return real_import(name, *args, **kwargs)

        with mock.patch("builtins.__import__", side_effect=import_without_numpy):
            encoded, count = olx_record.encode_cloud_frame(message, 6)

        self.assertEqual(count, 1)
        self.assertEqual(struct.unpack_from("<fffBBBB", encoded, 16)[3:],
                         (127, 127, 127, 255))

    def test_pose_frame_layout(self):
        pose = SimpleNamespace(
            header=header(20, 500_000_000),
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )),
        )
        encoded = olx_record.encode_pose_frame(pose, 4)
        self.assertEqual(len(encoded), 40)
        self.assertEqual(struct.unpack("<Id7f", encoded),
                         (4, 20.5, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0))

        pose.pose.pose.position.x = math.inf
        with self.assertRaisesRegex(ValueError, "non-finite"):
            olx_record.encode_pose_frame(pose, 5)

    def test_session_files_are_complete(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary) / "20260102_030405"
            recorder = olx_record.SessionRecorder(root)
            recorder.write_cloud(cloud_message([(0.0, 0.0, 0.0, 1, 2, 3)]))
            recorder.close()
            self.assertEqual(recorder.cloud_frames, 1)
            self.assertEqual((root / "MT20260102_030405.olx").stat().st_size, 32)
            self.assertTrue((root / "OdinPose.bin").is_file())
            self.assertTrue((root / "OdinImage.bin").is_file())
            self.assertTrue((root / "image" / "info.txt").is_file())
            recorder.close()

    def test_live_preview_is_bounded_and_keeps_xyzrgba(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent = pathlib.Path(temporary)
            root = parent / "preview-session"
            preview = parent / "runtime" / "live-preview.bin"
            points = [
                (float(index), float(index + 1), float(index + 2),
                 10 + index, 20 + index, 30 + index)
                for index in range(5)
            ]
            recorder = olx_record.SessionRecorder(
                root, preview_file=preview, preview_points=2, preview_interval=0.0
            )
            recorder.write_cloud(cloud_message(points))
            status = recorder.status()
            recorder.close()

            payload = preview.read_bytes()
            self.assertEqual(len(payload), 24 + 2 * 16)
            self.assertEqual(struct.unpack_from("<8sIdI", payload),
                             (b"PCPV0001", 0, 12.25, 2))
            self.assertEqual(struct.unpack_from("<fffBBBB", payload, 24),
                             (0.0, 1.0, 2.0, 10, 20, 30, 255))
            self.assertEqual(struct.unpack_from("<fffBBBB", payload, 40),
                             (3.0, 4.0, 5.0, 13, 23, 33, 255))
            self.assertEqual(status["preview_points"], 2)
            self.assertFalse(list(preview.parent.glob("*.tmp-*")))

    def test_images_are_indexed_and_empty_payload_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary) / "images"
            recorder = olx_record.SessionRecorder(root)
            jpeg = SimpleNamespace(header=header(), format="jpeg", data=b"\xff\xd8test")
            png = SimpleNamespace(header=header(13), format="png", data=b"\x89PNG\r\n\x1a\nrest")
            recorder.write_image(jpeg)
            recorder.write_image(png)
            with self.assertRaisesRegex(ValueError, "empty"):
                recorder.write_image(SimpleNamespace(header=header(), format="jpeg", data=b""))
            recorder.close()

            self.assertEqual(recorder.image_frames, 2)
            self.assertTrue((root / "image" / "000000.jpg").is_file())
            self.assertTrue((root / "image" / "000001.png").is_file())
            image_stream = (root / "OdinImage.bin").read_bytes()
            self.assertEqual(struct.unpack_from("<IdI", image_stream), (0, 12.25, 6))
            self.assertEqual(struct.unpack_from("<IdI", image_stream, 22), (1, 13.25, 12))

    def test_duplicate_session_name_gets_unique_suffix(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent = pathlib.Path(temporary)
            (parent / "capture").mkdir()
            (parent / "capture-01").mkdir()

            root = olx_record._next_session_root(parent, "capture")

            self.assertEqual(root.name, "capture-02")

    def test_session_name_validation(self):
        self.assertEqual(olx_record._safe_session_name("safe_01"), "safe_01")
        with self.assertRaises(ValueError):
            olx_record._safe_session_name("../escape")
        with self.assertRaises(ValueError):
            olx_record._safe_session_name("..")


if __name__ == "__main__":
    unittest.main()
