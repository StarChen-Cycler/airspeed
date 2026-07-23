#!/usr/bin/env python3
"""
Camera Stream ROS2 Publisher — RealSense cameras → Image + CameraInfo topics.

from __future__ import annotations

Publishes JPEG-encoded Image + CameraInfo per camera matching the
AIRSPEED sensor_interface convention. Streams at native camera rate
via one background reader thread per camera — no artificial rate cap.

Stream types (color, depth, infra) are enabled per-camera in config/camera.yaml.
Unavailable stream types configured for a camera are warned in the console.

Usage:
  python3 camera_publisher.py
  python3 camera_publisher.py --config-dir config
"""


import os
import sys
_lerobot_src = os.environ.get("LEROBOT_SRC", "")
if _lerobot_src and _lerobot_src not in sys.path:
    sys.path.insert(0, _lerobot_src)

import argparse
import json
import struct
import threading
import time
from pathlib import Path
from typing import List, Tuple, Dict, Any, Optional

import yaml

_HAS_REALSENSE = False
_HAS_CV2 = False

try:
    from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
    _HAS_REALSENSE = True
except Exception:
    RealSenseCamera = None
    RealSenseCameraConfig = None

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    pass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_CAMERA_INFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Stream types that require pyrealsense2 direct access (lerobot wrapper only gives color)
_DIRECT_API_STREAMS = {"depth", "infra_left", "infra_right"}

# ---------------------------------------------------------------------------
# Image zenoh side channel
#
# rclpy costs ~130 ms per 1 MB frame (per-field payload conversion); zenoh
# passes raw bytes at ~0.9 ms (validated 2026-07-17, see analysis memo).
# All image frames (rgb8 raw or JPEG) therefore travel over zenoh; only
# CameraInfo uses ROS2 topics. ENVELOPE CONTRACT — keep in sync with the
# collector's core/runtime/zenoh_image_channel.py:
#   key:     ROS topic path without the leading slash (e.g. camera/head/image_raw)
#   payload: <QQHHB (ts_ns, seq, width, height, enc_len) + encoding + frame
# ---------------------------------------------------------------------------
ZENOH_LISTEN_ENDPOINT = "tcp/0.0.0.0:7447"
_ENVELOPE = struct.Struct("<QQHHB")


class _ZenohImageChannel:
    """Zenoh publisher for raw image frames (lazy zenoh import)."""

    def __init__(self, listen_endpoint: str = ZENOH_LISTEN_ENDPOINT) -> None:
        try:
            import zenoh
        except ImportError as exc:
            raise RuntimeError(
                "raw image side channel requires eclipse-zenoh "
                "(pip install eclipse-zenoh) — or run with --jpeg / --no-side-channel"
            ) from exc
        conf = zenoh.Config()
        conf.insert_json5("listen/endpoints", f'["{listen_endpoint}"]')
        self._session = zenoh.open(conf)
        self._pubs: Dict[str, object] = {}
        self._seq = 0
        # Serializes sends from the per-camera reader threads.
        self._lock = threading.Lock()

    def _publisher(self, key: str):
        if key not in self._pubs:
            self._pubs[key] = self._session.declare_publisher(key)
        return self._pubs[key]

    def send(self, key: str, *, ts_ns: int, width: int, height: int,
             encoding: str, payload: bytes) -> None:
        enc = encoding.encode()
        with self._lock:
            header = _ENVELOPE.pack(ts_ns, self._seq, width, height, len(enc))
            self._seq += 1
            self._publisher(key).put(header + enc + payload)

    def close(self) -> None:
        self._session.close()


def _load_config(path: Path) -> Dict[str, Any]:
    with open(path) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Camera discovery + connection
# ---------------------------------------------------------------------------

def _discover_and_connect(cfg: Dict) -> List[Tuple[str, str, Dict, object]]:
    """Discover RealSense cameras and connect those enabled in config.

    Returns list of (camera_key, topic, stream_configs, camera_object).
    Warns when a configured camera is not found or a stream type is unavailable.
    """
    if not _HAS_REALSENSE:
        print("      RealSenseCamera not available, skipping cameras")
        return []

    cameras_cfg = cfg.get("cameras", {})
    if not cameras_cfg:
        print("      No cameras configured")
        return []

    # Discover
    try:
        found = RealSenseCamera.find_cameras()
    except Exception as e:
        print(f"      Camera discovery failed: {e}")
        return []

    max_cameras = cfg.get("max_cameras", len(found))
    n_found = len(found)
    print(f"      Found {n_found} RealSense camera(s) (max {max_cameras})")
    for i, info in enumerate(found[:max_cameras]):
        print(f"        [{i}] {info.get('name', 'Unknown')}  SN={info.get('id', 'unknown')}")

    # Match discovered cameras to config entries by SN (not by index)
    connected: List[Tuple[str, str, Dict, object]] = []
    used_keys: set[str] = set()

    for info in found[:max_cameras]:
        sn = info.get("id", "")
        cam_key: str | None = None
        entry: Dict | None = None

        # 1) Exact SN match
        for ck, ce in cameras_cfg.items():
            if ck in used_keys:
                continue
            cfg_sn = ce.get("serial", "auto")
            if cfg_sn != "auto" and cfg_sn == sn:
                cam_key = ck
                entry = ce
                break

        # 2) Fallback to first unused "auto" entry
        if cam_key is None:
            for ck, ce in cameras_cfg.items():
                if ck in used_keys:
                    continue
                if ce.get("serial", "auto") == "auto":
                    cam_key = ck
                    entry = ce
                    break

        if cam_key is None:
            print(f"      [SN={sn}] no matching config entry — skipping")
            continue

        used_keys.add(cam_key)
        topic = entry.get("topic", f"/camera/{cam_key}")
        frame_id = entry.get("frame_id", f"camera_{cam_key}_optical_frame")

        try:
            # Honor the color stream's fps/width/height from camera.yaml —
            # RealSenseCameraConfig requires all three or none; without them
            # the camera silently falls back to its native profile (e.g. D405
            # streams 848x480 instead of the configured 640x480).
            color_cfg = entry.get("streams", {}).get("color", {})
            w = color_cfg.get("width")
            h = color_cfg.get("height")
            fps = color_cfg.get("fps")
            if w and h and fps:
                cam_cfg = RealSenseCameraConfig(
                    serial_number_or_name=sn, fps=fps, width=w, height=h,
                )
            else:
                cam_cfg = RealSenseCameraConfig(serial_number_or_name=sn)
            cam = RealSenseCamera(cam_cfg)
            cam.connect()
            connected.append((cam_key, topic, entry, cam))
            print(f"        {cam_key} (SN={sn}) → {topic}  connected")
        except Exception as e:
            print(f"        {cam_key} (SN={sn}) → {topic}  FAILED: {e}")
            continue

        # Warn about unavailable stream types
        streams_cfg = entry.get("streams", {})
        for stype, scfg in streams_cfg.items():
            if scfg.get("enabled", False) and stype in _DIRECT_API_STREAMS:
                print(f"          [WARN] {stype}: requires pyrealsense2 direct access, not yet supported")

    # Warn about config entries that couldn't be matched
    for ck in cameras_cfg:
        if ck not in used_keys:
            print(f"      [WARN] {ck}: configured but not connected")

    return connected


def _build_camera_info(frame_id: str, intrinsics: Dict) -> CameraInfo:
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.height = intrinsics.get("height", 480)
    msg.width = intrinsics.get("width", 640)
    msg.distortion_model = intrinsics.get("distortion_model", "plumb_bob")
    msg.d = intrinsics.get("distortion_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])
    fx, fy = intrinsics.get("fx", 615.0), intrinsics.get("fy", 615.0)
    cx, cy = intrinsics.get("cx", 320.0), intrinsics.get("cy", 240.0)
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.binning_x = 0
    msg.binning_y = 0
    return msg


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class CameraPublisherNode(Node):
    """Publishes Image + CameraInfo per camera at the hardware's native rate."""

    def __init__(self, cameras: List[Tuple[str, str, Dict, object]], cfg: Dict,
                 *, force_jpeg: bool = False, side_channel: bool = True) -> None:
        super().__init__("camera_publisher")
        self._intrinsics = cfg.get("intrinsics", {})

        # Build stream entries: one per (camera, enabled stream type)
        self._streams: List[Dict] = []
        for cam_key, topic, entry, cam_obj in cameras:
            frame_id = entry.get("frame_id", f"camera_{cam_key}_optical_frame")
            streams_cfg = entry.get("streams", {})

            for stype, scfg in streams_cfg.items():
                if not scfg.get("enabled", False):
                    continue
                if stype in _DIRECT_API_STREAMS:
                    continue  # warned during discovery

                topic_suffix = {"color": "image_raw", "depth": "depth/image_raw",
                                "infra_left": "infra_left/image_raw",
                                "infra_right": "infra_right/image_raw"}.get(stype, f"{stype}/image_raw")

                img_pub = self.create_publisher(Image, f"{topic}/{topic_suffix}", _QOS)
                info_pub = self.create_publisher(CameraInfo, f"{topic}/camera_info", _CAMERA_INFO_QOS)
                self._streams.append({
                    "cam_key": cam_key,
                    "topic": topic,
                    "frame_id": frame_id,
                    "cam": cam_obj,
                    "img_pub": img_pub,
                    "info_pub": info_pub,
                    "stype": stype,
                    "key": f"{topic}/{topic_suffix}".lstrip("/"),
                    "encoding": "jpeg" if force_jpeg else scfg.get("encoding", "jpeg"),
                    "jpeg_q": scfg.get("jpeg_quality", 90),
                })
                self.get_logger().info(f"Camera: {topic}/{topic_suffix} ({stype})")

        # Publish CameraInfo once at startup (latched)
        for s in self._streams:
            cam_key = s["cam_key"]
            intrinsics = self._intrinsics.get(cam_key, {})
            if intrinsics:
                ci = _build_camera_info(s["frame_id"], intrinsics)
                s["info_pub"].publish(ci)

        self._running = False
        self._threads: List[threading.Thread] = []
        self._counts: Dict[str, int] = {}
        self._last_print = time.monotonic()
        self._log_lock = threading.Lock()

        # Image frames travel over the zenoh side channel (rclpy is too slow for
        # ~1 MB raw messages); JPEG frames are small but still use zenoh to match
        # the session config transport. CameraInfo stays on ROS2 topics.
        self._channel: Optional[_ZenohImageChannel] = None
        if side_channel and self._streams:
            self._channel = _ZenohImageChannel()
            self.get_logger().info(
                f"Zenoh side channel on {ZENOH_LISTEN_ENDPOINT} "
                f"for {len(self._streams)} stream(s)"
            )
        self.get_logger().info(f"Camera Publisher: native rate, {len(self._streams)} stream(s)")

    def start(self) -> None:
        self._running = True
        # One reader thread per camera. RealSenseCamera.read() blocks until the
        # camera's NEXT fresh frame, so a single shared loop couples every
        # camera to the slowest phase alignment — a near-miss on one camera
        # slips ALL cameras by a full frame period (the observed 30↔15 Hz
        # lockstep drop). Independent threads let each camera publish at its
        # own native rate.
        per_camera: Dict[str, List[Dict]] = {}
        for s in self._streams:
            per_camera.setdefault(s["cam_key"], []).append(s)
        self._threads = [
            threading.Thread(
                target=self._stream_loop, args=(streams,),
                daemon=True, name=f"camera-stream-{cam_key}",
            )
            for cam_key, streams in per_camera.items()
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._running = False
        for t in self._threads:
            t.join(timeout=2.0)
        if self._channel is not None:
            self._channel.close()

    def _stream_loop(self, streams: List[Dict]) -> None:
        """Run in a per-camera background thread at that camera's native rate.

        No create_timer() cap — the camera delivers frames at its hardware rate.
        Single color stream: ~30 Hz. Multi-stream (depth+IR+color): ~15 Hz (USB 3.2 limit).
        """
        while self._running:
            if not _HAS_CV2:
                time.sleep(0.1)
                continue

            for s in streams:
                try:
                    frame = s["cam"].read()
                    if frame is None or frame.size == 0:
                        continue

                    # Stamp at frame return (≈ exposure time) — before color
                    # convert and JPEG encode, whose variable latency must not
                    # leak into the canonical timestamp.
                    stamp = self.get_clock().now().to_msg()

                    if s["encoding"] == "jpeg":
                        # JPEG path: compress at source (small messages, stored as-is).
                        # cv2.imencode links against libjpeg-turbo (OpenCV on this box
                        # reports libjpeg-turbo 3.0.3) and was the fastest installed
                        # encoder in a 2026-07-23 benchmark: ~0.7 ms/frame p50, p99
                        # < 1 ms for 640x480 RGB at q90. PIL.save was ~7 ms, and
                        # torchvision.io.encode_jpeg was >30 ms.
                        if len(frame.shape) == 3 and frame.shape[2] == 3:
                            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        else:
                            frame_bgr = frame

                        ret, jpeg_buf = cv2.imencode(
                            ".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, s["jpeg_q"]]
                        )
                        if not ret:
                            continue
                        payload = jpeg_buf.tobytes()
                        step = len(payload)
                    else:
                        # Raw path: original pixels, no compression. The lerobot
                        # wrapper delivers RGB; mono frames pass through as-is.
                        payload = frame.tobytes()
                        step = frame.shape[1] * (frame.shape[2] if frame.ndim == 3 else 1)

                    if self._channel is not None:
                        # Zenoh side channel for image frames — rclpy cannot
                        # sustain ~1 MB raw messages; see _ZenohImageChannel docs.
                        self._channel.send(
                            s["key"],
                            ts_ns=stamp.sec * 10**9 + stamp.nanosec,
                            width=frame.shape[1], height=frame.shape[0],
                            encoding=s["encoding"], payload=payload,
                        )
                    else:
                        msg = Image()
                        msg.header.stamp = stamp
                        msg.header.frame_id = s["frame_id"]
                        msg.height = frame.shape[0]
                        msg.width = frame.shape[1]
                        msg.encoding = s["encoding"]
                        msg.is_bigendian = 0
                        msg.step = step
                        msg.data = payload
                        s["img_pub"].publish(msg)

                    key = s["cam_key"]
                    self._counts[key] = self._counts.get(key, 0) + 1

                except Exception as exc:
                    self.get_logger().error(
                        f"Camera {s['cam_key']}: {exc}", throttle_duration_sec=5.0
                    )

            self._maybe_log()

    def _maybe_log(self) -> None:
        # Called from every per-camera thread; serialize so the aggregate
        # rate print and counter reset happen once per window.
        with self._log_lock:
            now = time.monotonic()
            if now - self._last_print >= 10.0:
                total = sum(self._counts.values())
                rate = total / (now - self._last_print)
                parts = ", ".join(f"{k}={self._counts.get(k,0)}" for k in sorted(self._counts))
                self.get_logger().info(f"Frames: {rate:.1f} Hz ({total} total) | {parts}")
                self._counts.clear()
                self._last_print = now


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Camera Stream ROS2 Publisher")
    parser.add_argument("--config-dir", default="config", help="Config directory")
    parser.add_argument(
        "--jpeg", action="store_true",
        help="Compress frames to JPEG at the source (default: off — publish raw "
             "pixels per the config encoding; the HDF5 then stores original data)",
    )
    parser.add_argument(
        "--no-side-channel", action="store_true",
        help="Disable the zenoh side channel and publish raw frames on ROS2 "
             "topics (slow: ~130 ms/frame; for A/B debugging only)",
    )
    args = parser.parse_args()

    config_dir = Path(args.config_dir)
    if not config_dir.is_absolute():
        config_dir = Path(__file__).resolve().parent / config_dir
    cfg = _load_config(config_dir / "camera.yaml")

    if not _HAS_REALSENSE:
        print("ERROR: RealSenseCamera not available. Add lerobot to PYTHONPATH.")
        sys.exit(1)

    print("=" * 50)
    print("  Camera Stream ROS2 Publisher (native rate)")
    print("=" * 50)
    print()

    print("[1/2] Discovering cameras...")
    cameras = _discover_and_connect(cfg)
    if not cameras:
        print("      No cameras could be connected. Exiting.")
        sys.exit(1)

    print("\n[2/2] Starting ROS2 publisher...")
    rclpy.init(args=sys.argv)
    node = CameraPublisherNode(cameras, cfg, force_jpeg=args.jpeg,
                               side_channel=not args.no_side_channel)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()
