"""Tests for the adaptive dashboard image preview (encode helper + discovery)."""
from __future__ import annotations

import io

import pytest

from core.runtime.manual_operator_ui import (
    ManualOperatorUI,
    _encode_preview_frame,
    build_image_stream_info,
)
from core.runtime.stream_tracker import StreamMetrics, StreamStatus


def test_jpeg_passthrough_identical():
    jpeg = b"\xff\xd8fakejpegdata\xff\xd9"
    payload, mime = _encode_preview_frame((jpeg, "jpeg", 4, 2, 1))
    assert payload is jpeg
    assert mime == "image/jpeg"


def test_raw_rgb8_encodes_to_lossless_png():
    payload, mime = _encode_preview_frame((bytes(4 * 2 * 3), "rgb8", 4, 2, 1))
    assert mime == "image/png"
    assert payload[:4] == b"\x89PNG"
    from PIL import Image
    img = Image.open(io.BytesIO(payload))
    assert img.size == (4, 2)
    assert img.mode == "RGB"
    # PNG is lossless: pixel data survives the roundtrip exactly.
    assert list(img.getdata()) == [(0, 0, 0)] * (4 * 2)


def test_raw_mono8_encodes():
    payload, mime = _encode_preview_frame((bytes(4 * 2), "mono8", 4, 2, 1))
    assert mime == "image/png"
    assert payload[:4] == b"\x89PNG"


def test_bad_dims_and_unknown_encoding_raise():
    with pytest.raises(ValueError):
        _encode_preview_frame((b"\x00" * 10, "rgb8", 0, 0, 1))
    with pytest.raises(ValueError, match="unsupported"):
        _encode_preview_frame((b"\x00" * 10, "yuv422", 4, 2, 1))


def test_build_image_stream_info():
    meta = [
        {"name": "camera_head", "encoding": "raw", "transport": "zenoh"},
        {"name": "camera_old", "encoding": "jpeg", "transport": "ros2"},
    ]
    latest = {"camera_head": (b"\x00" * 24, "rgb8", 4, 2, 999)}
    snapshot = {
        "camera_head": (StreamStatus.HEALTHY, StreamMetrics(
            observed_rate_hz=29.7, last_timestamp_age_ms=12.0)),
        "camera_old": (StreamStatus.ABSENT, StreamMetrics()),
    }
    info = build_image_stream_info(meta, latest, snapshot)
    assert [s["name"] for s in info] == ["camera_head", "camera_old"]
    head = info[0]
    assert head["cached"] is True
    assert (head["width"], head["height"]) == (4, 2)
    assert head["status"] == "healthy"
    assert abs(head["rate"] - 29.7) < 1e-9
    old = info[1]
    assert old["cached"] is False
    assert old["status"] == "absent"


def _ui() -> ManualOperatorUI:
    return ManualOperatorUI(state_machine=None, control_router=None, stream_tracker=None)


def test_memoized_preview_encodes_once_per_frame():
    ui = _ui()
    frame = (bytes(4 * 2 * 3), "rgb8", 4, 2, 111)
    a = ui._memoized_preview("cam", frame)
    b = ui._memoized_preview("cam", frame)
    assert a is b  # second call served from the memo — no re-encode
    c = ui._memoized_preview("cam", (bytes(4 * 2 * 3), "rgb8", 4, 2, 222))
    assert c is not a
    assert c[1] == "image/png"
    assert c[0][:4] == b"\x89PNG"  # new ts → fresh encode


def test_memoized_preview_memoizes_encode_failure():
    ui = _ui()
    bad = (b"\x00" * 10, "yuv422", 4, 2, 333)
    assert ui._memoized_preview("cam", bad) is None
    # failure is memoized too — a second call does not raise or retry
    assert ui._memoized_preview("cam", bad) is None
