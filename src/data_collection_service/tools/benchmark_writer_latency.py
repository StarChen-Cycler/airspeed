#!/usr/bin/env python3
"""Benchmark single-threaded writer latency under simulated full load.

Replays 15 streams at the rates declared in the session YAML (VR/IK/arm/cam)
for a fixed duration, measures per-append wall-clock latency, and reports
mean/p99 write latency plus total frames written. Used by T7 to decide whether
the threaded writer twin is necessary.
"""
from __future__ import annotations

import argparse
import io
import json
import statistics
import sys
import time as _time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent.parent.parent
SRC = ROOT / "src" / "data_collection_service"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.config import load_session_config
from core.adapters import AdapterRegistry
from core.storage import AirsHdf5Writer


def _make_jpeg(width: int, height: int) -> bytes:
    img = Image.new("RGB", (width, height), color=(1, 2, 3))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


JPEG_BY_DIM = {
    (640, 480): _make_jpeg(640, 480),
    (848, 480): _make_jpeg(848, 480),
}


def _make_message(stream_name: str, stream, *, timestamp: datetime):
    """Build a minimal ROS-shaped message matching the stream contract."""
    msg_type = stream.message_type
    if msg_type == "geometry_msgs/PoseStamped":
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(
                    sec=int(timestamp.timestamp()),
                    nanosec=int((timestamp.timestamp() % 1) * 1e9),
                ),
                frame_id=stream.frame_id or "world",
            ),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
                orientation=SimpleNamespace(x=0.0, y=0.1, z=0.2, w=0.9),
            ),
        )
    if msg_type == "sensor_msgs/Joy":
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(
                    sec=int(timestamp.timestamp()),
                    nanosec=int((timestamp.timestamp() % 1) * 1e9),
                ),
                frame_id="",
            ),
            axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            buttons=[],
        )
    if msg_type == "sensor_msgs/JointState":
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(
                    sec=int(timestamp.timestamp()),
                    nanosec=int((timestamp.timestamp() % 1) * 1e9),
                ),
                frame_id="",
            ),
            name=[],
            position=[0.0] * 8,
            velocity=[],
            effort=[],
        )
    if msg_type == "sensor_msgs/Image":
        w, h = 640, 480
        if "wrist" in stream_name:
            w, h = 848, 480
        jpeg = JPEG_BY_DIM[(w, h)]
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(
                    sec=int(timestamp.timestamp()),
                    nanosec=int((timestamp.timestamp() % 1) * 1e9),
                ),
                frame_id=stream.frame_id or "camera",
            ),
            height=h, width=w, encoding="rgb8", is_bigendian=0, step=w * 3,
            data=bytes(w * h * 3),
        )
    raise ValueError(f"unsupported message type: {msg_type}")


def _append(writer: AirsHdf5Writer, adapter, name: str, msg, received_at: datetime) -> float:
    t0 = _time.perf_counter_ns()
    sample = adapter.adapt(msg, received_at=received_at)
    if sample.image_data is not None:
        writer.append_image(name, sample.image_data, sample.timestamp_ns,
                            width=sample.width, height=sample.height)
    elif sample.values is not None:
        writer.append_vector(name, sample.values, sample.timestamp_ns)
    return (_time.perf_counter_ns() - t0) / 1e6  # ms


def run_benchmark(duration_s: float, output_dir: Path, config_path: Path) -> dict:
    config = load_session_config(config_path)
    registry = AdapterRegistry.with_defaults()
    adapters = registry.resolve_session(config)

    output_dir.mkdir(parents=True, exist_ok=True)
    writer = AirsHdf5Writer(
        output_dir,
        description=config.session.name,
        robot_type="_".join(sorted({entry.role for _, entry in config.session.devices})),
        series_number=config.session.operator_id,
    )
    writer.open_episode("episode-benchmark")
    for adapter in adapters.values():
        adapter.register_with(writer)

    # Fallback to YAML-typical rates if sample_rate is not declared.
    yaml_rates = {
        "vr_head_pose": 62.0, "vr_left_pose": 62.0, "vr_right_pose": 62.0,
        "vr_left_buttons": 62.0, "vr_right_buttons": 62.0,
        "ik_left_joint_commands": 44.0, "ik_right_joint_commands": 44.0,
        "ik_left_target_pose": 44.0, "ik_right_target_pose": 44.0,
        "arm_left_joint_state": 30.0, "arm_right_joint_state": 30.0,
        "camera_head": 14.6, "camera_left_wrist": 14.6, "camera_right_wrist": 14.6,
    }
    stream_rates = {}
    for name, stream in config.streams:
        rate = float(getattr(stream, "sample_rate", 0.0) or 0.0)
        stream_rates[name] = rate if rate > 0 else yaml_rates.get(name, 10.0)

    latencies_ms: list[float] = []
    frame_counts: dict[str, int] = {name: 0 for name, _ in config.streams}

    start_mono = _time.perf_counter()
    start_dt = datetime.now(timezone.utc)
    elapsed = 0.0
    while elapsed < duration_s:
        for name, stream in config.streams:
            rate = stream_rates.get(name, 10.0)
            expected = int(elapsed * rate)
            while frame_counts[name] <= expected and elapsed < duration_s:
                ts = start_dt + timedelta(seconds=frame_counts[name] / rate)
                msg = _make_message(name, stream, timestamp=ts)
                lat = _append(writer, adapters[name], name, msg, received_at=ts)
                latencies_ms.append(lat)
                frame_counts[name] += 1
        elapsed = _time.perf_counter() - start_mono

    writer.close_episode(task_completed=True, termination_reason="goal_reached")

    total_frames = sum(frame_counts.values())
    mean_lat = statistics.mean(latencies_ms) if latencies_ms else 0.0
    p99_lat = sorted(latencies_ms)[int(len(latencies_ms) * 0.99)] if latencies_ms else 0.0
    max_lat = max(latencies_ms) if latencies_ms else 0.0

    return {
        "duration_s": duration_s,
        "total_frames": total_frames,
        "frames_per_stream": frame_counts,
        "mean_latency_ms": mean_lat,
        "p99_latency_ms": p99_lat,
        "max_latency_ms": max_lat,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Benchmark writer latency under full load")
    parser.add_argument("--duration", type=float, default=10.0, help="Benchmark duration in seconds")
    parser.add_argument("--output-dir", default="/tmp/writer-benchmark", help="Output directory")
    parser.add_argument("--config", default="src/data_collection_service/config/session_vr_ik_robot_button_control.yaml")
    args = parser.parse_args()

    result = run_benchmark(args.duration, Path(args.output_dir), Path(args.config))
    print(json.dumps(result, indent=2, default=int))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
