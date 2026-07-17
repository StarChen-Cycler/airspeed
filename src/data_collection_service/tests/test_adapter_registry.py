"""Tests for adapter registry and effective column derivation."""
from __future__ import annotations

from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from core.adapters import AdapterError, AdapterRegistry
from core.config import load_session_config_dict


def _make_config(*, drop_pose_columns: bool = False, drop_button_columns: bool = False, joy_buttons: bool = False):
    streams = {
        "vr_head_pose": {
            "source": "teleop",
            "topic": "/vr/head_pose",
            "message_type": "geometry_msgs/PoseStamped",
            "columns": (["head_qw", "head_qx", "head_qy", "head_qz", "head_px", "head_py", "head_pz"]
                        if not drop_pose_columns else None),
            "fields": [
                {"path": "pose.position.x", "type": "float64"},
                {"path": "pose.position.y", "type": "float64"},
                {"path": "pose.position.z", "type": "float64"},
                {"path": "pose.orientation.x", "type": "float64"},
                {"path": "pose.orientation.y", "type": "float64"},
                {"path": "pose.orientation.z", "type": "float64"},
                {"path": "pose.orientation.w", "type": "float64"},
            ],
        },
        "vr_left_buttons": {
            "source": "teleop",
            "topic": "/vr/left_buttons",
            "message_type": "sensor_msgs/Joy" if joy_buttons else "std_msgs/Float32MultiArray",
            "columns": (["vr_l_trigger", "vr_l_grip"] if not drop_button_columns else None),
            "time_domain": "ros_header" if joy_buttons else "ros_receive",
            "fields": [{"path": "axes" if joy_buttons else "data", "type": "sequence"}],
        },
        "camera_left_wrist": {
            "source": "sensor",
            "topic": "/cam",
            "message_type": "sensor_msgs/Image",
            "image_encoding": "jpeg",
            "fields": [{"path": "data", "type": "bytes"}],
        },
    }
    return load_session_config_dict({
        "schema_version": "1.0",
        "session": {"name": "test", "task_id": "t", "operator_id": "o"},
        "storage": {"root": "data/episodes", "format": "hdf5"},
        "streams": streams,
    })


def test_effective_columns_from_yaml_override():
    cfg = _make_config()
    adapters = AdapterRegistry.with_defaults().resolve_session(cfg)
    assert adapters["vr_left_buttons"].effective_columns() == ("vr_l_trigger", "vr_l_grip")


def test_effective_columns_auto_derived_scalar_fields():
    cfg = _make_config(drop_pose_columns=True)
    adapters = AdapterRegistry.with_defaults().resolve_session(cfg)
    cols = adapters["vr_head_pose"].effective_columns()
    assert cols == (
        "pose.orientation.w", "pose.orientation.x", "pose.orientation.y",
        "pose.orientation.z", "pose.position.x", "pose.position.y", "pose.position.z",
    )


def test_image_stream_has_empty_columns():
    cfg = _make_config()
    adapters = AdapterRegistry.with_defaults().resolve_session(cfg)
    assert adapters["camera_left_wrist"].effective_columns() == ()


def test_sequence_stream_without_columns_fails():
    cfg = _make_config(drop_button_columns=True)
    with pytest.raises(AdapterError, match="streams without column names"):
        AdapterRegistry.with_defaults().resolve_session(cfg)


def test_joy_buttons_binding_reads_axes_with_header_time():
    cfg = _make_config(joy_buttons=True)
    adapters = AdapterRegistry.with_defaults().resolve_session(cfg)
    adapter = adapters["vr_left_buttons"]
    assert adapter.effective_columns() == ("vr_l_trigger", "vr_l_grip")

    msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1700000000, nanosec=500000000),
            frame_id="",
        ),
        axes=[1.0, 0.0, 0.0, 0.0, 0.0, 0.5],
        buttons=[],
    )
    sample = adapter.adapt(msg, received_at=datetime.now(timezone.utc))
    assert sample.timestamp_ns == 1700000000_500000000
    assert sample.values == (1.0, 0.0, 0.0, 0.0, 0.0, 0.5)
