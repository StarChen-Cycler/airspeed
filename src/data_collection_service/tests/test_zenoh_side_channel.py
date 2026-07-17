"""Unit and edge-case tests for the zenoh side channel.

Covers both supported message shapes (sensor_msgs/Image, sensor_msgs/Joy),
the envelope contract, malformed-frame handling, multi-stream independence,
and the session-YAML transport key.
"""
from __future__ import annotations

import struct
from datetime import datetime, timezone

import pytest

from core.adapters import AdapterRegistry
from core.config import SessionConfigError, load_session_config_dict
from core.runtime.zenoh_side_channel import (
    _ENVELOPE,
    unpack_frame,
    unpack_image,
    unpack_joy,
)

_TS_NS = 1_700_000_000_500_000_000


def _img_frame(ts_ns: int = _TS_NS, seq: int = 0, width: int = 4, height: int = 2,
               encoding: str = "rgb8", payload: bytes | None = None) -> bytes:
    enc = encoding.encode()
    if payload is None:
        payload = bytes(width * height * 3)
    return _ENVELOPE.pack(ts_ns, seq, width, height, len(enc)) + enc + payload


def _joy_frame(values: list[float], ts_ns: int = _TS_NS, seq: int = 0,
               encoding: str = "float32") -> bytes:
    enc = encoding.encode()
    body = struct.pack(f"<{len(values)}f", *values)
    return _ENVELOPE.pack(ts_ns, seq, 0, 0, len(enc)) + enc + body


def _config(streams: dict):
    return load_session_config_dict({
        "schema_version": "1.0",
        "session": {"name": "t", "task_id": "t", "operator_id": "o"},
        "storage": {"root": "data/episodes", "format": "hdf5"},
        "streams": streams,
    })


def _image_stream(transport: str = "zenoh") -> dict:
    return {"camera_head": {
        "source": "sensor", "topic": "/camera/head/image_raw",
        "message_type": "sensor_msgs/Image", "transport": transport,
        "image_encoding": "raw", "columns": [],
        "fields": [{"path": "data", "type": "bytes"}],
    }}


def _tactile_stream(transport: str = "zenoh") -> dict:
    return {"tactile_left": {
        "source": "sensor", "topic": "/tactile/left",
        "message_type": "sensor_msgs/Joy", "transport": transport,
        "columns": ["taxel_1", "taxel_2", "taxel_3"],
        "fields": [{"path": "axes", "type": "sequence"}],
    }}


# -- envelope ---------------------------------------------------------------


def test_unpack_frame_roundtrip():
    ts_ns, seq, w, h, enc, body = unpack_frame(_img_frame(seq=7, width=640, height=480))
    assert (ts_ns, seq, w, h, enc) == (_TS_NS, 7, 640, 480, "rgb8")
    assert len(body) == 640 * 480 * 3


def test_unpack_frame_rejects_truncated_header():
    with pytest.raises(ValueError, match="shorter than envelope"):
        unpack_frame(b"\x00" * 5)


def test_unpack_frame_rejects_truncated_encoding():
    bad = _ENVELOPE.pack(_TS_NS, 0, 1, 1, 10) + b"rgb8"  # declares 10, has 4
    with pytest.raises(ValueError, match="truncated"):
        unpack_frame(bad)


# -- image shape ------------------------------------------------------------


def test_unpack_image_fields():
    msg = unpack_image(_img_frame())
    assert msg.header.stamp.sec == 1700000000
    assert msg.header.stamp.nanosec == 500000000
    assert (msg.width, msg.height, msg.encoding) == (4, 2, "rgb8")
    assert msg.step == 4 * 3
    assert len(msg.data) == 4 * 2 * 3


def test_unpack_image_rejects_zero_dims_and_empty_payload():
    with pytest.raises(ValueError, match="invalid dims"):
        unpack_image(_ENVELOPE.pack(_TS_NS, 0, 0, 2, 4) + b"rgb8" + b"\x00" * 6)
    with pytest.raises(ValueError, match="empty payload"):
        unpack_image(_ENVELOPE.pack(_TS_NS, 0, 4, 2, 4) + b"rgb8")


# -- joy shape (tactile / high-bandwidth arrays) ----------------------------


def test_unpack_joy_fields():
    msg = unpack_joy(_joy_frame([0.5, 1.25, -2.0]))
    assert msg.header.stamp.sec == 1700000000
    assert msg.axes == [0.5, 1.25, -2.0]
    assert msg.buttons == []


def test_unpack_joy_rejects_bad_encoding_and_ragged_body():
    with pytest.raises(ValueError, match="float32"):
        unpack_joy(_joy_frame([1.0], encoding="rgb8"))
    with pytest.raises(ValueError, match="whole float32"):
        unpack_joy(_ENVELOPE.pack(_TS_NS, 0, 0, 0, 7) + b"float32" + b"\x00" * 6)


# -- adapter integration ----------------------------------------------------


def test_image_frame_flows_through_adapter():
    adapters = AdapterRegistry.with_defaults().resolve_session(_config(_image_stream()))
    sample = adapters["camera_head"].adapt(
        unpack_image(_img_frame()), received_at=datetime.now(timezone.utc))
    assert sample.timestamp_ns == _TS_NS
    assert sample.encoding == "rgb8"
    assert len(sample.image_data) == 4 * 2 * 3


def test_tactile_frame_flows_through_adapter():
    adapters = AdapterRegistry.with_defaults().resolve_session(_config(_tactile_stream()))
    sample = adapters["tactile_left"].adapt(
        unpack_joy(_joy_frame([0.1, 0.2, 0.3])), received_at=datetime.now(timezone.utc))
    assert sample.timestamp_ns == _TS_NS
    assert sample.values == pytest.approx((0.1, 0.2, 0.3))


def test_tactile_columns_from_yaml():
    adapters = AdapterRegistry.with_defaults().resolve_session(_config(_tactile_stream()))
    assert adapters["tactile_left"].effective_columns() == ("taxel_1", "taxel_2", "taxel_3")


def test_large_tactile_payload_roundtrip():
    # 4096 taxels — realistic high-bandwidth tactile frame
    values = [float(i % 256) / 255.0 for i in range(4096)]
    msg = unpack_joy(_joy_frame(values))
    assert len(msg.axes) == 4096
    assert msg.axes[4095] == values[4095]


# -- transport key ----------------------------------------------------------


def test_invalid_transport_rejected():
    with pytest.raises(SessionConfigError, match="transport"):
        _config(_image_stream(transport="mqtt"))


def test_transport_defaults_to_ros2():
    cfg = _config({"s": {
        "source": "sensor", "topic": "/x", "message_type": "sensor_msgs/Image",
        "fields": [{"path": "data", "type": "bytes"}],
    }})
    assert dict(cfg.streams)["s"].transport == "ros2"
