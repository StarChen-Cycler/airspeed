"""Tests for convert_h5_to_lerobot.py consuming new-vintage attrs."""
from __future__ import annotations

import io
import json
from pathlib import Path

import h5py
import numpy as np
import pytest

from tools.convert_h5_to_lerobot import convert_to_lerobot


def _make_jpeg(width: int, height: int) -> bytes:
    from PIL import Image
    img = Image.new("RGB", (width, height), color=(1, 2, 3))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    return buf.getvalue()


def _make_episode(h5_path: Path, *,
                  columns: dict[str, list[str]],
                  sample_rates: dict[str, float],
                  task_prompt: str = "Pick up the cube",
                  task_completed: bool = True,
                  recording_valid: bool = True,
                  add_legacy_dim_n: bool = False,
                  omit_columns: bool = False,
                  omit_task_prompt: bool = False,
                  ) -> None:
    with h5py.File(h5_path, "w") as f:
        for name, cols in columns.items():
            grp = f.create_group(name)
            grp.attrs["type"] = "vector"
            if not omit_columns:
                write_cols = ["dim_0"] if add_legacy_dim_n else cols
                grp.attrs["columns"] = json.dumps(write_cols)
            grp.attrs["sample_rate"] = sample_rates.get(name, 0.0)
            data = np.zeros((5, len(cols)), dtype=np.float32)
            grp.create_dataset("data", data=data)
            grp.create_dataset("timestamps", data=np.arange(5, dtype=np.uint64))
        # one camera
        cam = f.create_group("camera_head")
        cam.attrs["type"] = "image"
        cam.attrs["width"] = 4
        cam.attrs["height"] = 4
        cam.attrs["channels"] = 3
        cam.attrs["encoding"] = "jpeg"
        cam.attrs["sample_rate"] = sample_rates.get("camera_head", 0.0)
        jpeg = _make_jpeg(4, 4)
        dt = h5py.vlen_dtype(np.dtype("uint8"))
        dset = cam.create_dataset("data", shape=(5,), dtype=dt)
        for i in range(5):
            dset[i] = np.frombuffer(jpeg, dtype=np.uint8)
        cam.create_dataset("timestamps", data=np.arange(5, dtype=np.uint64))

        f.attrs["description"] = "test"
        f.attrs["robot_type"] = "openarm"
        f.attrs["series_number"] = "op"
        f.attrs["frames"] = 5
        f.attrs["task_completed"] = task_completed
        f.attrs["recording_valid"] = recording_valid
        f.attrs["termination_reason"] = "goal_reached"
        if not omit_task_prompt:
            f.attrs["task_prompt"] = task_prompt


def test_converter_uses_semantic_columns(tmp_path):
    h5 = tmp_path / "ep.h5"
    _make_episode(h5, columns={
        "vr_head_pose": ["head_qw", "head_qx", "head_qy", "head_qz", "head_px", "head_py", "head_pz"],
        "arm_left_joint_state": ["L_joint_state_1", "L_gripper_state"],
    }, sample_rates={"vr_head_pose": 60.0, "arm_left_joint_state": 20.0})
    out = tmp_path / "out"
    convert_to_lerobot(h5, out, fps=60)
    with open(out / "meta" / "info.json") as f:
        info = json.load(f)
    assert info["total_episodes"] == 1
    # feature names should include the authored names
    names = info["features"]["observation.state"]["names"]
    assert "L_joint_state_1" in names
    assert "head_qw" in names


def test_converter_errors_on_missing_columns(tmp_path):
    h5 = tmp_path / "ep.h5"
    _make_episode(h5, columns={"vr_head_pose": ["a", "b"]}, sample_rates={"vr_head_pose": 60.0},
                  omit_columns=True)
    with pytest.raises(ValueError, match="missing 'columns' attr"):
        convert_to_lerobot(h5, tmp_path / "out", fps=60)


def test_converter_errors_on_legacy_dim_n(tmp_path):
    h5 = tmp_path / "ep.h5"
    _make_episode(h5, columns={"vr_head_pose": ["a", "b"]}, sample_rates={"vr_head_pose": 60.0},
                  add_legacy_dim_n=True)
    with pytest.raises(ValueError, match="legacy dim_N"):
        convert_to_lerobot(h5, tmp_path / "out", fps=60)


def test_converter_errors_on_missing_task_prompt(tmp_path):
    h5 = tmp_path / "ep.h5"
    _make_episode(h5, columns={"vr_head_pose": ["a", "b"]}, sample_rates={"vr_head_pose": 60.0},
                  omit_task_prompt=True)
    with pytest.raises(ValueError, match="missing root attr 'task_prompt'"):
        convert_to_lerobot(h5, tmp_path / "out", fps=60)


def test_converter_warns_on_rate_deviation(tmp_path, capsys):
    h5 = tmp_path / "ep.h5"
    _make_episode(h5, columns={"vr_head_pose": ["a", "b"]}, sample_rates={"vr_head_pose": 30.0})
    convert_to_lerobot(h5, tmp_path / "out", fps=60)
    captured = capsys.readouterr()
    assert "deviates" in captured.err


def test_converter_keeps_failure_demo_by_default(tmp_path):
    h5 = tmp_path / "ep.h5"
    _make_episode(h5, columns={"vr_head_pose": ["a", "b"]}, sample_rates={"vr_head_pose": 60.0},
                  task_completed=False)
    out = tmp_path / "out"
    convert_to_lerobot(h5, out, fps=60)
    with open(out / "meta" / "info.json") as f:
        info = json.load(f)
    assert info["total_episodes"] == 1


def test_converter_excludes_failure_demos_when_requested(tmp_path):
    h5 = tmp_path / "ep.h5"
    _make_episode(h5, columns={"vr_head_pose": ["a", "b"]}, sample_rates={"vr_head_pose": 60.0},
                  task_completed=False)
    with pytest.raises(ValueError, match="task_completed=False"):
        convert_to_lerobot(h5, tmp_path / "out", fps=60,
                           exclude_failure_demos=True)


def test_converter_rejects_recording_invalid(tmp_path):
    h5 = tmp_path / "ep.h5"
    _make_episode(h5, columns={"vr_head_pose": ["a", "b"]}, sample_rates={"vr_head_pose": 60.0},
                  recording_valid=False)
    with pytest.raises(ValueError, match="recording_valid=False"):
        convert_to_lerobot(h5, tmp_path / "out", fps=60)
