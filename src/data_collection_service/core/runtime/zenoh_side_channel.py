"""Zenoh side-channel subscriber for high-bandwidth streams.

rclpy costs ~130 ms per ~1 MB message due to per-field payload conversion;
zenoh passes raw bytes at ~0.9 ms (validated 2026-07-17). Streams that declare
`transport: zenoh` in the session YAML therefore bypass ROS2: the producing
adaptor publishes struct-envelope frames on zenoh keys (the ROS topic path
without the leading slash), and this channel unpacks them into SimpleNamespace
objects shaped like the stream's declared ROS2 message type, then forwards them
to the collection node's normal message handler — adapter validation, stream
tracking, and HDF5 writing are unchanged.

Intended for image streams and any other high-bandwidth stream (e.g. tactile
arrays). Both supported message shapes carry a header, so the strict
ros_header contract holds end to end.

ENVELOPE CONTRACT — keep in sync with the producing adaptor
(e.g. _ZenohImageChannel in camera-stream-adaptor/camera_publisher.py):
  key:     ROS topic path without the leading slash (e.g. camera/head/image_raw)
  payload: <QQHHB (ts_ns, seq, width, height, enc_len) + encoding + body

Per message_type the fields mean:
  sensor_msgs/Image:  width/height = frame dims, encoding = pixel encoding
                      (e.g. "rgb8"), body = frame bytes
  sensor_msgs/Joy:    width/height = 0, encoding = "float32",
                      body = little-endian float32 array → axes
"""

from __future__ import annotations

import logging
import struct
from types import SimpleNamespace
from typing import Any, Callable

logger = logging.getLogger(__name__)

ZENOH_CONNECT_ENDPOINT = "tcp/127.0.0.1:7447"
_ENVELOPE = struct.Struct("<QQHHB")
_IMAGE_TYPE = "sensor_msgs/Image"
_JOY_TYPE = "sensor_msgs/Joy"
_SUPPORTED_TYPES = frozenset({_IMAGE_TYPE, _JOY_TYPE})


def unpack_frame(data: bytes) -> tuple[int, int, int, int, str, bytes]:
    """Unpack the shared envelope. Returns (ts_ns, seq, width, height, encoding, body)."""
    if len(data) < _ENVELOPE.size:
        raise ValueError("frame shorter than envelope header")
    ts_ns, seq, width, height, enc_len = _ENVELOPE.unpack(data[:_ENVELOPE.size])
    if len(data) < _ENVELOPE.size + enc_len:
        raise ValueError("frame truncated inside encoding field")
    encoding = data[_ENVELOPE.size:_ENVELOPE.size + enc_len].decode()
    if not encoding:
        raise ValueError("empty encoding")
    body = data[_ENVELOPE.size + enc_len:]
    return ts_ns, seq, width, height, encoding, body


def _header_ns(ts_ns: int) -> SimpleNamespace:
    sec, nanosec = divmod(ts_ns, 1_000_000_000)
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec), frame_id="")


def unpack_image(data: bytes) -> SimpleNamespace:
    """Envelope frame → sensor_msgs/Image-shaped namespace."""
    ts_ns, _seq, width, height, encoding, body = unpack_frame(data)
    if width <= 0 or height <= 0:
        raise ValueError(f"image frame with invalid dims {width}x{height}")
    if not body:
        raise ValueError("image frame with empty payload")
    return SimpleNamespace(
        header=_header_ns(ts_ns),
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=0,
        step=len(body) // height,
        data=body,
    )


def unpack_joy(data: bytes) -> SimpleNamespace:
    """Envelope frame → sensor_msgs/Joy-shaped namespace (axes from float32 body)."""
    ts_ns, _seq, _w, _h, encoding, body = unpack_frame(data)
    if encoding != "float32":
        raise ValueError(f"joy frame encoding must be 'float32'; got {encoding!r}")
    if not body or len(body) % 4 != 0:
        raise ValueError("joy payload is not a whole float32 array")
    n = len(body) // 4
    axes = list(struct.unpack(f"<{n}f", body))
    return SimpleNamespace(header=_header_ns(ts_ns), axes=axes, buttons=[])


_UNPACKERS = {_IMAGE_TYPE: unpack_image, _JOY_TYPE: unpack_joy}


class ZenohSideChannel:
    """Subscribes to high-bandwidth streams over zenoh.

    Lazily imports zenoh: when the package is missing, start() logs a warning
    and returns False — the streams show as absent instead of crashing the node.
    """

    def __init__(self, *, connect_endpoint: str = ZENOH_CONNECT_ENDPOINT) -> None:
        self._endpoint = connect_endpoint
        self._session = None

    def start(self, streams: list[tuple[str, str, str]],
              on_message: Callable[[str, Any], None]) -> bool:
        """Subscribe on_message(stream_name, msg) for each (name, key, message_type).

        Returns False for unsupported message types or a missing zenoh package.
        """
        bad = [name for name, _key, mt in streams if mt not in _SUPPORTED_TYPES]
        if bad:
            logger.error(f"zenoh side channel does not support message types of: {bad}")
            return False
        try:
            import zenoh
        except ImportError:
            logger.warning(
                "zenoh side channel unavailable (pip install eclipse-zenoh); "
                "%d stream(s) will show as absent", len(streams),
            )
            return False

        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{self._endpoint}"]')
        self._session = zenoh.open(conf)
        for stream_name, key, message_type in streams:
            self._session.declare_subscriber(
                key, self._make_handler(stream_name, message_type, on_message),
            )
            logger.info(f"  {stream_name} → zenoh:{key} ({self._endpoint})")
        return True

    def stop(self) -> None:
        if self._session is not None:
            self._session.close()
            self._session = None

    @staticmethod
    def _make_handler(stream_name: str, message_type: str,
                      on_message: Callable[[str, Any], None]):
        unpack = _UNPACKERS[message_type]

        def handler(sample) -> None:
            try:
                payload = sample.payload
                data = payload.to_bytes() if hasattr(payload, "to_bytes") else bytes(payload)
                msg = unpack(data)
            except Exception as exc:
                logger.error(f"{stream_name}: bad zenoh frame: {exc}")
                return
            on_message(stream_name, msg)
        return handler


__all__ = [
    "ZenohSideChannel",
    "unpack_frame",
    "unpack_image",
    "unpack_joy",
    "ZENOH_CONNECT_ENDPOINT",
]
