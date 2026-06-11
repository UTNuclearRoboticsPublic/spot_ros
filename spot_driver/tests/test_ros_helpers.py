"""Tests for the image conversions in ros_helpers.

Uses the real getImageMsg with minimal fake protos. Requires rclpy/bosdyn/
sensor_msgs (run inside a sourced ROS 2 environment); skips elsewhere.
"""
import types

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("bosdyn")
pytest.importorskip("sensor_msgs")
from bosdyn.api import image_pb2  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image  # noqa: E402
from spot_driver.ros_helpers import (  # noqa: E402
    getImageMsg, UnsupportedImageFormatError)


class _Logger:
    def error(self, *args, **kwargs):
        pass


class _LeaseManager:
    logger = _Logger()

    @staticmethod
    def robotToLocalTime(_timestamp):
        return types.SimpleNamespace(seconds=0, nanos=0)


def _image_response(fmt, pixel_format=None, rows=2, cols=2, data=b"\x01" * 12):
    """Minimal ImageResponse stand-in: format dispatch + intrinsics only."""
    intrinsics = types.SimpleNamespace(
        focal_length=types.SimpleNamespace(x=1.0, y=1.0),
        principal_point=types.SimpleNamespace(x=0.0, y=0.0))
    return types.SimpleNamespace(
        source=types.SimpleNamespace(
            name="cam", pinhole=types.SimpleNamespace(intrinsics=intrinsics)),
        shot=types.SimpleNamespace(
            acquisition_time=0,
            frame_name_image_sensor="cam_frame",
            transforms_snapshot=types.SimpleNamespace(
                child_to_parent_edge_map={}),
            image=types.SimpleNamespace(
                format=fmt, pixel_format=pixel_format,
                rows=rows, cols=cols, data=data)),
    )


def test_raw_rgb_u8_produces_valid_image():
    data = _image_response(image_pb2.Image.FORMAT_RAW,
                           image_pb2.Image.PIXEL_FORMAT_RGB_U8)
    img, info, _ = getImageMsg(data, _LeaseManager())
    assert isinstance(img, Image)
    assert img.encoding == "rgb8"
    assert img.step == 3 * 2
    assert len(img.data) == 12 and img.height == 2 and img.width == 2
    assert info.width == 2 and info.height == 2


def test_raw_depth_u16_produces_valid_image():
    data = _image_response(image_pb2.Image.FORMAT_RAW,
                           image_pb2.Image.PIXEL_FORMAT_DEPTH_U16, data=b"\x01" * 8)
    img, _info, _ = getImageMsg(data, _LeaseManager())
    assert img.encoding == "16UC1"
    assert img.step == 2 * 2


def test_jpeg_produces_compressed_image():
    jpeg_bytes = b"\xff\xd8\xff" + b"\x00" * 32
    data = _image_response(image_pb2.Image.FORMAT_JPEG, data=jpeg_bytes)
    img, info, _ = getImageMsg(data, _LeaseManager())
    assert isinstance(img, CompressedImage)
    assert img.format == "rgb8; jpeg compressed bgr8"
    assert bytes(img.data) == jpeg_bytes  # passed through unmodified
    assert info.width == 2 and info.height == 2  # intrinsics still populated


def test_raw_with_unhandled_pixel_format_is_rejected():
    data = _image_response(image_pb2.Image.FORMAT_RAW, 9999)
    with pytest.raises(UnsupportedImageFormatError, match="pixel_format"):
        getImageMsg(data, _LeaseManager())


def test_unknown_image_format_is_rejected():
    data = _image_response(9999, image_pb2.Image.PIXEL_FORMAT_RGB_U8)
    with pytest.raises(UnsupportedImageFormatError, match="unsupported image format"):
        getImageMsg(data, _LeaseManager())
