"""Tests for AirsHdf5Writer: columns, sample_rate, task attrs, camera dims."""
from __future__ import annotations

import io
import json
from pathlib import Path
from datetime import datetime, timezone

import h5py
import numpy as np
import pytest

from core.storage import AirsHdf5Writer, AirsHdf5WriterError
from core.storage.airs_hdf5_writer import _jpeg_dimensions, _measured_rate


def _make_valid_jpeg(width: int = 2, height: int = 3) -> bytes:
    from PIL import Image
    import io
    img = Image.new("RGB", (width, height), color=(1, 2, 3))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    return buf.getvalue()


def test_vector_columns_attr_written(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_vector_stream("s", dims=2, columns=("a", "b"))
    writer.append_vector("s", np.array([1.0, 2.0]), 1)
    path = writer.close_episode(task_completed=True, termination_reason="goal_reached")
    with h5py.File(path, "r") as f:
        assert json.loads(f["s"].attrs["columns"]) == ["a", "b"]


def test_vector_dimension_mismatch_raises(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_vector_stream("s", dims=2, columns=("a", "b"))
    with pytest.raises(AirsHdf5WriterError, match="dimension mismatch"):
        writer.append_vector("s", np.array([1.0]), 1)


def test_missing_columns_raises_at_close(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_vector_stream("s", dims=1)
    writer.append_vector("s", np.array([1.0]), 1)
    with pytest.raises(AirsHdf5WriterError, match="no column names"):
        writer.close_episode(task_completed=True, termination_reason="goal_reached")


def test_per_stream_sample_rate(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_vector_stream("s", dims=1, columns=("a",))
    for i in range(11):
        writer.append_vector("s", [float(i)], i * 100_000_000)
    path = writer.close_episode(task_completed=True, termination_reason="goal_reached")
    with h5py.File(path, "r") as f:
        assert abs(f["s"].attrs["sample_rate"] - 10.0) < 1e-9
        assert "sample_rate" not in f.attrs


def test_task_root_attrs(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.set_task("pick-up-cube", {
        "task_name": "pick-up-cube",
        "task_prompt": "Pick up the cube",
        "task_id": "abc123def456",
        "task_structure": "atomic_single",
        "deformable_objects": False,
    })
    writer.open_episode("ep")
    writer.register_vector_stream("s", dims=1, columns=("a",))
    writer.append_vector("s", [1.0], 1)
    path = writer.close_episode(task_completed=True, termination_reason="goal_reached")
    with h5py.File(path, "r") as f:
        assert f.attrs["task_prompt"] == "Pick up the cube"
        assert f.attrs["task_id"] == "abc123def456"
        assert f.attrs["task_structure"] == "atomic_single"
        assert bool(f.attrs["deformable_objects"]) is False


def test_outcome_attrs_no_legacy_success(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_vector_stream("s", dims=1, columns=("a",))
    writer.append_vector("s", [1.0], 1)
    path = writer.close_episode(task_completed=False, termination_reason="operator_abort")
    with h5py.File(path, "r") as f:
        assert bool(f.attrs["task_completed"]) is False
        assert bool(f.attrs["recording_valid"]) is True
        assert f.attrs["termination_reason"] == "operator_abort"
        assert "success" not in f.attrs


def test_stamp_recording_invalid(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_vector_stream("s", dims=1, columns=("a",))
    writer.append_vector("s", [1.0], 1)
    path = writer.close_episode(task_completed=True, termination_reason="goal_reached")
    AirsHdf5Writer.stamp_recording_invalid(path)
    trashed = Path(AirsHdf5Writer.move_to_trash(path))
    with h5py.File(trashed, "r") as f:
        assert bool(f.attrs["task_completed"]) is True
        assert bool(f.attrs["recording_valid"]) is False
        assert f.attrs["termination_reason"] == "recording_invalid"


def test_camera_dims_from_metadata(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_image_stream("cam", width=0, height=0, channels=3)
    writer.append_image("cam", _make_valid_jpeg(848, 480), 1, width=848, height=480)
    path = writer.close_episode(task_completed=True, termination_reason="goal_reached")
    with h5py.File(path, "r") as f:
        assert f["cam"].attrs["width"] == 848
        assert f["cam"].attrs["height"] == 480


def test_camera_dims_jpeg_decode_fallback(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_image_stream("cam", width=0, height=0, channels=3)
    data = _make_valid_jpeg(2, 3)
    writer.append_image("cam", data, 1, width=0, height=0)
    writer.append_image("cam", data, 2, width=0, height=0)
    path = writer.close_episode(task_completed=True, termination_reason="goal_reached")
    with h5py.File(path, "r") as f:
        assert f["cam"].attrs["width"] == 2
        assert f["cam"].attrs["height"] == 3


def test_measured_rate_edge_cases():
    assert _measured_rate(0, 1, 2) == 0.0
    assert _measured_rate(1, 1, 2) == 0.0
    assert _measured_rate(2, 0, 1_000_000_000) == 1.0
    assert _measured_rate(2, 0, 0) == 0.0


def test_jpeg_dimensions_parse():
    data = _make_valid_jpeg(7, 5)
    assert _jpeg_dimensions(data) == (7, 5)


def test_zero_frame_streams_are_pruned_at_close(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_vector_stream("live", dims=1, columns=("a",))
    writer.register_vector_stream("dead_vec", dims=0, columns=("b",))
    writer.register_image_stream("dead_img", width=0, height=0, channels=3)
    writer.append_vector("live", [1.0], 1)
    path = writer.close_episode(task_completed=True, termination_reason="goal_reached")
    with h5py.File(path, "r") as f:
        assert "live" in f
        assert f["live"]["data"].shape == (1, 1)
        assert json.loads(f["live"].attrs["columns"]) == ["a"]
        # Zero-frame streams leave no groups behind — no zero-length datasets
        assert "dead_vec" not in f
        assert "dead_img" not in f
        assert f.attrs["frames"] == 1


def test_raw_image_stream_stores_bytes_verbatim_with_message_encoding(tmp_path):
    writer = AirsHdf5Writer(tmp_path)
    writer.open_episode("ep")
    writer.register_image_stream("cam", width=0, height=0, channels=3, encoding="raw")
    raw = bytes(i % 256 for i in range(3 * 2 * 3))
    writer.append_image("cam", raw, 1, width=3, height=2, encoding="rgb8")
    path = writer.close_episode(task_completed=True, termination_reason="goal_reached")
    with h5py.File(path, "r") as f:
        assert f["cam"].attrs["encoding"] == "rgb8"
        assert bytes(f["cam"]["data"][0]) == raw
        assert f["cam"].attrs["width"] == 3
        assert f["cam"].attrs["height"] == 2
