#!/usr/bin/env python3
"""End-to-end validation of new-vintage H5 metadata + LeRobot converter.

Generates three synthetic episodes (completed, failed, trashed) with the
new-vintage attribute schema, dumps their attrs, runs the LeRobot converter,
and loads the resulting dataset. Also verifies the converter rejects legacy
and invalid episodes.

This script substitutes for real hardware takes in offline CI; the VR button
channel mapping (ch0-5) still needs physical verification on the headset.
"""
from __future__ import annotations

import argparse
import io
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import h5py
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent.parent.parent
SRC = ROOT / "src" / "data_collection_service"
TOOLS = SRC / "tools"

# Stream layout matching session_vr_ik_robot_button_control.yaml
STREAMS: dict[str, dict[str, object]] = {
    "vr_head_pose": {
        "type": "vector",
        "columns": ["head_qw", "head_qx", "head_qy", "head_qz", "head_px", "head_py", "head_pz"],
        "sample_rate": 62.0,
        "width": None,
        "height": None,
    },
    "vr_left_pose": {
        "type": "vector",
        "columns": ["vr_l_qw", "vr_l_qx", "vr_l_qy", "vr_l_qz", "vr_l_px", "vr_l_py", "vr_l_pz"],
        "sample_rate": 62.0,
    },
    "vr_right_pose": {
        "type": "vector",
        "columns": ["vr_r_qw", "vr_r_qx", "vr_r_qy", "vr_r_qz", "vr_r_px", "vr_r_py", "vr_r_pz"],
        "sample_rate": 62.0,
    },
    "vr_left_buttons": {
        "type": "vector",
        "columns": ["vr_l_trigger", "vr_l_grip", "vr_l_button_2", "vr_l_button_3", "vr_l_button_4", "vr_l_button_5"],
        "sample_rate": 62.0,
    },
    "vr_right_buttons": {
        "type": "vector",
        "columns": ["vr_r_trigger", "vr_r_grip", "vr_r_button_2", "vr_r_button_3", "vr_r_button_4", "vr_r_button_5"],
        "sample_rate": 62.0,
    },
    "ik_left_joint_commands": {
        "type": "vector",
        "columns": ["L_joint_cmd_1", "L_joint_cmd_2", "L_joint_cmd_3", "L_joint_cmd_4", "L_joint_cmd_5", "L_joint_cmd_6", "L_joint_cmd_7", "L_gripper_cmd"],
        "sample_rate": 44.0,
    },
    "ik_right_joint_commands": {
        "type": "vector",
        "columns": ["R_joint_cmd_1", "R_joint_cmd_2", "R_joint_cmd_3", "R_joint_cmd_4", "R_joint_cmd_5", "R_joint_cmd_6", "R_joint_cmd_7", "R_gripper_cmd"],
        "sample_rate": 44.0,
    },
    "ik_left_target_pose": {
        "type": "vector",
        "columns": ["ik_l_qw", "ik_l_qx", "ik_l_qy", "ik_l_qz", "ik_l_px", "ik_l_py", "ik_l_pz"],
        "sample_rate": 44.0,
    },
    "ik_right_target_pose": {
        "type": "vector",
        "columns": ["ik_r_qw", "ik_r_qx", "ik_r_qy", "ik_r_qz", "ik_r_px", "ik_r_py", "ik_r_pz"],
        "sample_rate": 44.0,
    },
    "arm_left_joint_state": {
        "type": "vector",
        "columns": ["L_joint_state_1", "L_joint_state_2", "L_joint_state_3", "L_joint_state_4", "L_joint_state_5", "L_joint_state_6", "L_joint_state_7", "L_gripper_state"],
        "sample_rate": 30.0,
    },
    "arm_right_joint_state": {
        "type": "vector",
        "columns": ["R_joint_state_1", "R_joint_state_2", "R_joint_state_3", "R_joint_state_4", "R_joint_state_5", "R_joint_state_6", "R_joint_state_7", "R_gripper_state"],
        "sample_rate": 30.0,
    },
    "camera_head": {
        "type": "image",
        "columns": None,
        "sample_rate": 14.6,
        "width": 640,
        "height": 480,
    },
    "camera_left_wrist": {
        "type": "image",
        "columns": None,
        "sample_rate": 14.6,
        "width": 848,
        "height": 480,
    },
    "camera_right_wrist": {
        "type": "image",
        "columns": None,
        "sample_rate": 14.6,
        "width": 848,
        "height": 480,
    },
}

CAMERA_DIMS = {
    "camera_head": (640, 480),
    "camera_left_wrist": (848, 480),
    "camera_right_wrist": (848, 480),
}


def _make_jpeg(width: int, height: int) -> bytes:
    img = Image.new("RGB", (width, height), color=(1, 2, 3))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    return buf.getvalue()


_JPEGS = {name: _make_jpeg(w, h) for name, (w, h) in CAMERA_DIMS.items()}


def _write_vector_stream(grp: h5py.Group, meta: dict, frames: int) -> None:
    cols = meta["columns"]
    grp.attrs["type"] = "vector"
    grp.attrs["columns"] = json.dumps(cols)
    grp.attrs["sample_rate"] = float(meta["sample_rate"])
    data = np.zeros((frames, len(cols)), dtype=np.float32)
    grp.create_dataset("data", data=data)
    grp.create_dataset("timestamps", data=np.arange(frames, dtype=np.uint64) * 1_000_000_000 // int(meta["sample_rate"]))


def _write_image_stream(grp: h5py.Group, meta: dict, frames: int) -> None:
    name = grp.name.split("/")[-1]
    w, h = CAMERA_DIMS[name]
    grp.attrs["type"] = "image"
    grp.attrs["width"] = w
    grp.attrs["height"] = h
    grp.attrs["channels"] = 3
    grp.attrs["encoding"] = "jpeg"
    grp.attrs["sample_rate"] = float(meta["sample_rate"])
    dt = h5py.vlen_dtype(np.dtype("uint8"))
    dset = grp.create_dataset("data", shape=(frames,), dtype=dt)
    jpeg = _JPEGS[name]
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    for i in range(frames):
        dset[i] = arr
    grp.create_dataset("timestamps", data=np.arange(frames, dtype=np.uint64) * 1_000_000_000 // int(meta["sample_rate"]))


def make_episode(path: Path, *, variant: str, frames: int = 64, fps: int = 60) -> Path:
    """Write a synthetic new-vintage episode."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as f:
        for name, meta in STREAMS.items():
            grp = f.create_group(name)
            if meta["type"] == "vector":
                _write_vector_stream(grp, meta, frames)
            else:
                _write_image_stream(grp, meta, frames)

        f.attrs["description"] = "validate_h5_metadata_pipeline"
        f.attrs["robot_type"] = "openarm"
        f.attrs["series_number"] = "validation_operator"
        f.attrs["frames"] = frames
        f.attrs["task_prompt"] = "Pick up the red cube and place it in the bin"
        f.attrs["task_id"] = "pick-red-cube-v1"
        f.attrs["task_structure"] = "rigid_cube_on_table"
        f.attrs["deformable_objects"] = False

        if variant == "completed":
            f.attrs["task_completed"] = True
            f.attrs["recording_valid"] = True
            f.attrs["termination_reason"] = "goal_reached"
        elif variant == "failed":
            f.attrs["task_completed"] = False
            f.attrs["recording_valid"] = True
            f.attrs["termination_reason"] = "operator_abort"
        elif variant == "trashed":
            f.attrs["task_completed"] = False
            f.attrs["recording_valid"] = False
            f.attrs["termination_reason"] = "recording_invalid"
        else:
            raise ValueError(variant)
    return path


def make_legacy_episode(path: Path, *, frames: int = 64) -> Path:
    """Write a legacy episode missing task_prompt and using dim_N columns."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as f:
        grp = f.create_group("vr_head_pose")
        grp.attrs["type"] = "vector"
        grp.attrs["columns"] = json.dumps(["dim_0", "dim_1"])
        grp.attrs["sample_rate"] = 60.0
        grp.create_dataset("data", data=np.zeros((frames, 2), dtype=np.float32))
        grp.create_dataset("timestamps", data=np.arange(frames, dtype=np.uint64))
        f.attrs["description"] = "legacy"
        f.attrs["robot_type"] = "openarm"
        f.attrs["series_number"] = "legacy"
        f.attrs["frames"] = frames
        f.attrs["success"] = True
        f.attrs["termination_reason"] = "goal_reached"
    return path


def dump_attrs(path: Path) -> dict:
    with h5py.File(path, "r") as f:
        attrs = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in f.attrs.items()}
        streams = {}
        for name in f.keys():
            grp = f[name]
            streams[name] = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in grp.attrs.items()}
    return {"path": str(path), "attrs": attrs, "streams": streams}


def run_converter(h5_path: Path, output_dir: Path, fps: int = 60) -> subprocess.CompletedProcess:
    cmd = [
        sys.executable,
        str(TOOLS / "convert_h5_to_lerobot.py"),
        "--hdf5", str(h5_path.resolve()),
        "--output", str(output_dir.resolve()),
        "--fps", str(fps),
    ]
    return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)


def load_lerobot_summary(path: Path) -> dict:
    """Read the dataset meta files without requiring a live LeRobotDataset."""
    import pandas as pd
    info = json.loads((path / "meta" / "info.json").read_text())
    tasks = pd.read_parquet(path / "meta" / "tasks.parquet")
    return {
        "info": info,
        "task_text": str(tasks.index[0]) if not tasks.empty else "",
        "num_episodes": info.get("total_episodes", 0),
        "num_frames": info.get("total_frames", 0),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate H5 metadata pipeline end-to-end")
    parser.add_argument("--output-dir", default=None, help="Directory for generated artifacts")
    parser.add_argument("--fps", type=int, default=60)
    args = parser.parse_args()

    base = Path(args.output_dir) if args.output_dir else Path(tempfile.mkdtemp(prefix="t9-validation-"))
    base = base.resolve()
    episodes_dir = base / "episodes"
    datasets_dir = base / "lerobot_datasets"
    dumps_dir = base / "attr_dumps"
    for d in (episodes_dir, datasets_dir, dumps_dir):
        d.mkdir(parents=True, exist_ok=True)

    print(f"Artifacts: {base}\n")

    variants = ["completed", "failed", "trashed"]
    episode_paths = {}
    for variant in variants:
        path = episodes_dir / f"episode-{variant}.h5"
        make_episode(path, variant=variant, fps=args.fps)
        episode_paths[variant] = path
        dump = dump_attrs(path)
        dump_path = dumps_dir / f"episode-{variant}-attrs.json"
        dump_path.write_text(json.dumps(dump, indent=2, default=str))
        print(f"Generated {variant}: {path}")
        print(f"  Attrs dump: {dump_path}")

    # Convert completed and failed; trashed should be rejected by recording_valid=False guardrail.
    convert_results = {}
    for variant in ("completed", "failed"):
        out = datasets_dir / variant
        if out.exists():
            shutil.rmtree(out)
        proc = run_converter(episode_paths[variant], out, fps=args.fps)
        convert_results[variant] = proc
        status = "PASS" if proc.returncode == 0 else "FAIL"
        print(f"\nConverter {variant}: {status}")
        if proc.returncode != 0:
            print(proc.stderr[-500:])
        else:
            summary = load_lerobot_summary(out)
            print(f"  LeRobot episodes: {summary['num_episodes']}, frames: {summary['num_frames']}")
            print(f"  Task prompt: {summary['task_text']}")

    # Trashed episode must be rejected.
    trashed_out = datasets_dir / "trashed"
    if trashed_out.exists():
        shutil.rmtree(trashed_out)
    proc = run_converter(episode_paths["trashed"], trashed_out, fps=args.fps)
    convert_results["trashed"] = proc
    if proc.returncode == 0:
        print("\nERROR: trashed episode was accepted; expected rejection")
        return 1
    if "recording_valid=False" not in proc.stderr:
        print("\nERROR: trashed rejection did not name recording_valid=False")
        return 1
    print("\nConverter trashed: REJECTED (recording_valid=False) — expected")

    # Legacy episode must be rejected.
    legacy_path = episodes_dir / "episode-legacy.h5"
    make_legacy_episode(legacy_path)
    legacy_dump = dump_attrs(legacy_path)
    (dumps_dir / "episode-legacy-attrs.json").write_text(json.dumps(legacy_dump, indent=2, default=str))
    legacy_out = datasets_dir / "legacy"
    if legacy_out.exists():
        shutil.rmtree(legacy_out)
    proc = run_converter(legacy_path, legacy_out, fps=args.fps)
    convert_results["legacy"] = proc
    if proc.returncode == 0:
        print("\nERROR: legacy episode was accepted; expected rejection")
        return 1
    if "missing root attr 'task_prompt'" not in proc.stderr and "legacy dim_N" not in proc.stderr:
        print("\nERROR: legacy rejection did not name expected attr error")
        return 1
    print("\nConverter legacy: REJECTED (missing task_prompt / dim_N) — expected")

    print("\n=== Validation PASSED ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
