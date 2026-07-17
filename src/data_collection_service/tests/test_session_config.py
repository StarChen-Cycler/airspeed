"""Tests for session config parsing, including the new `columns:` key."""
from __future__ import annotations

import pytest

from core.config import SessionConfigError, load_session_config_dict


def test_columns_parsed_and_validated():
    cfg = load_session_config_dict({
        "schema_version": "1.0",
        "session": {"name": "test", "task_id": "t", "operator_id": "o"},
        "storage": {"root": "data/episodes", "format": "hdf5"},
        "streams": {
            "s": {
                "source": "robot",
                "topic": "/s",
                "message_type": "sensor_msgs/JointState",
                "columns": ["a", "b", "c"],
                "fields": [{"path": "position", "type": "sequence"}],
            },
        },
    })
    stream = dict(cfg.streams)["s"]
    assert stream.columns == ("a", "b", "c")


def test_columns_must_be_unique():
    with pytest.raises(SessionConfigError, match="unique"):
        load_session_config_dict({
            "schema_version": "1.0",
            "session": {"name": "test", "task_id": "t", "operator_id": "o"},
            "storage": {"root": "data/episodes", "format": "hdf5"},
            "streams": {
                "s": {
                    "source": "robot",
                    "topic": "/s",
                    "message_type": "sensor_msgs/JointState",
                    "columns": ["a", "a"],
                    "fields": [{"path": "position", "type": "sequence"}],
                },
            },
        })


def test_columns_must_be_non_empty_strings():
    with pytest.raises(SessionConfigError, match="non-empty"):
        load_session_config_dict({
            "schema_version": "1.0",
            "session": {"name": "test", "task_id": "t", "operator_id": "o"},
            "storage": {"root": "data/episodes", "format": "hdf5"},
            "streams": {
                "s": {
                    "source": "robot",
                    "topic": "/s",
                    "message_type": "sensor_msgs/JointState",
                    "columns": [""],
                    "fields": [{"path": "position", "type": "sequence"}],
                },
            },
        })


def test_abort_binding_allowed():
    cfg = load_session_config_dict({
        "schema_version": "1.0",
        "session": {
            "name": "test", "task_id": "t", "operator_id": "o",
            "recording_control": {
                "mode": "device_binding",
                "bindings": {
                    "toggle": {"stream_name": "b", "button_index": 5},
                    "abort": {"stream_name": "b", "button_index": 4},
                    "delete": {"stream_name": "b", "button_index": 4},
                },
            },
        },
        "storage": {"root": "data/episodes", "format": "hdf5"},
        "streams": {
            "b": {
                "source": "teleop",
                "topic": "/b",
                "message_type": "sensor_msgs/Joy",
                "columns": ["x"],
                "fields": [{"path": "axes", "type": "sequence"}],
            },
        },
    })
    actions = {a for a, _ in cfg.session.recording_control.bindings}
    assert actions == {"toggle", "abort", "delete"}
