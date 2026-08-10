"""Tests for the spot_image_server FPS instrumentation.

Exercises the real SpotImageServer.record_fps logic through a lightweight
stand-in object, so no robot or rclpy spin is needed. Requires rclpy/bosdyn
importable (run inside a sourced ROS 2 environment); skips elsewhere.
"""
import threading
import types
from collections import defaultdict

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("bosdyn")
from spot_driver.image_server import SpotImageServer, new_fps_bucket  # noqa: E402


class _FakeLogger:
    def __init__(self):
        self.infos = []

    def info(self, msg, *args, **kwargs):
        self.infos.append(msg)


def _server_stub(window_sec, window_start):
    """Minimal stand-in carrying exactly the attributes record_fps touches."""
    stub = types.SimpleNamespace()
    stub.fps_lock = threading.Lock()
    stub.fps_send_times = {}
    stub.fps_stats = defaultdict(new_fps_bucket)
    stub.fps_window_sec = window_sec
    stub.fps_window_start = window_start
    logger = _FakeLogger()
    stub.get_logger = lambda: logger
    return stub, logger


def _response(name, nbytes):
    return types.SimpleNamespace(
        source=types.SimpleNamespace(name=name),
        shot=types.SimpleNamespace(
            image=types.SimpleNamespace(data=b"\x00" * nbytes)),
    )


def test_new_buckets_do_not_share_state():
    a = new_fps_bucket()
    b = new_fps_bucket()
    assert a == {"count": 0, "bytes": 0, "lat_sum": 0.0, "lat_max": 0.0}
    a["count"] += 1
    assert b["count"] == 0


def test_accumulates_within_window_without_logging(monkeypatch):
    stub, logger = _server_stub(window_sec=999.0, window_start=1000.0)
    now = [1000.0]
    monkeypatch.setattr("spot_driver.image_server.time.monotonic", lambda: now[0])

    stub.fps_send_times["cam"] = 1000.0
    now[0] = 1000.150  # 150 ms round-trip
    SpotImageServer.record_fps(stub, _response("cam", 1_000_000))

    assert logger.infos == []  # window not elapsed, no summary yet
    stats = stub.fps_stats["cam"]
    assert stats["count"] == 1
    assert stats["bytes"] == 1_000_000
    assert stats["lat_max"] == pytest.approx(0.150, abs=1e-6)
    assert "cam" not in stub.fps_send_times  # send time consumed


def test_window_flush_emits_one_summary_and_resets(monkeypatch):
    stub, logger = _server_stub(window_sec=5.0, window_start=0.0)
    now = [0.0]
    monkeypatch.setattr("spot_driver.image_server.time.monotonic", lambda: now[0])

    # 10 frames across a 5 s window: ~2 Hz, ~2 MB/s at 1 MB each
    for _ in range(10):
        stub.fps_send_times["cam"] = now[0]
        now[0] += 0.5
        SpotImageServer.record_fps(stub, _response("cam", 1_000_000))

    assert len(logger.infos) == 1
    summary = logger.infos[0]
    assert "cam" in summary and "MB/s total" in summary
    assert dict(stub.fps_stats) == {}  # stats cleared after the flush


def test_missing_send_time_skips_latency_but_counts_bytes(monkeypatch):
    stub, _logger = _server_stub(window_sec=999.0, window_start=42.0)
    monkeypatch.setattr("spot_driver.image_server.time.monotonic", lambda: 42.0)
    SpotImageServer.record_fps(stub, _response("cam", 512))
    stats = stub.fps_stats["cam"]
    assert stats["count"] == 1 and stats["bytes"] == 512
    assert stats["lat_sum"] == 0.0 and stats["lat_max"] == 0.0
