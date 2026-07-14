#!/usr/bin/env python3
"""Convert AIRS HDF5 episode to LeRobot Dataset v3 format.

Maps AIRS streams to LeRobot feature namespaces:
  Vector streams → observation.state.*  +  action.*
  Image streams  → observation.images.*  (MP4 video)

Handles multi-rate async streams: VR@60Hz (1119 frames) is the canonical
timeline. Slower streams are nearest-timestamp matched.

Output: LeRobotDataset v3 with meta/ + data/ + videos/
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import h5py, numpy as np


def _nearest_idx(timestamps: np.ndarray, query_ts: int) -> int:
    """Find index of nearest timestamp to query_ts."""
    idx = np.searchsorted(timestamps, query_ts, side="left")
    idx = max(0, min(idx, len(timestamps) - 1))
    if idx > 0:
        dist_left = abs(int(timestamps[idx - 1]) - query_ts)
        dist_curr = abs(int(timestamps[idx]) - query_ts)
        if dist_left < dist_curr:
            idx -= 1
    return idx


def convert_to_lerobot(h5_path: Path, output_path: Path,
                       fps: int = 60, robot_type: str = "openarm",
                       repo_id: str = "airspeed_episode",
                       vcodec: str = "h264",
                       *, exclude_failure_demos: bool = False) -> str:
    """Convert AIRS HDF5 to LeRobot v3 dataset. Returns dataset path."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    with h5py.File(h5_path, "r") as f:
        # Fail-fast: new-vintage required attrs
        if "task_prompt" not in f.attrs:
            raise ValueError(
                f"{h5_path}: missing root attr 'task_prompt'; "
                "old episodes are retired and must not be converted"
            )
        if "recording_valid" in f.attrs and not bool(f.attrs["recording_valid"]):
            raise ValueError(
                f"{h5_path}: recording_valid=False; episode was deleted/invalidated"
            )
        if exclude_failure_demos and "task_completed" in f.attrs and not bool(f.attrs["task_completed"]):
            raise ValueError(
                f"{h5_path}: task_completed=False and --exclude-failure-demos is set"
            )

        # Classify streams from HDF5 attrs only.
        vr_vectors = []    # 60Hz teleop streams
        ik_vectors = []    # 50Hz IK command streams
        arm_vectors = []   # 20Hz arm state streams
        image_streams = [] # camera streams
        for name in f.keys():
            grp = f[name]
            gtype = str(grp.attrs.get("type", ""))
            if gtype == "image":
                image_streams.append(name)
            elif gtype == "vector":
                if name.startswith("vr_"):
                    vr_vectors.append(name)
                elif name.startswith("ik_"):
                    ik_vectors.append(name)
                elif name.startswith("arm_"):
                    arm_vectors.append(name)

        if not vr_vectors:
            raise ValueError(f"{h5_path}: no vr_* vector streams found")

        # Use VR head pose as canonical timeline
        canonical = "vr_head_pose"
        if canonical not in f:
            canonical = vr_vectors[0]
        canon_ts = f[canonical]["timestamps"][:]
        n_frames = len(canon_ts)
        print(f"  Canonical timeline: {canonical} ({n_frames} frames @ ~{fps}Hz)")

        # Pre-load all stream data for fast access
        stream_cache = {}
        for name in f.keys():
            grp = f[name]
            stream_cache[name] = {
                "type": str(grp.attrs["type"]),
                "data": grp["data"][:],
                "ts": grp["timestamps"][:],
                "sample_rate": float(grp.attrs.get("sample_rate", 0.0)),
                "columns": json.loads(grp.attrs.get("columns", "[]")),
            }

        # Validate and collect semantic column names
        all_vectors = vr_vectors + ik_vectors + arm_vectors
        seen: set[str] = set()
        duplicates: set[str] = set()
        state_names: list[str] = []
        for name in all_vectors:
            cols = stream_cache[name]["columns"]
            if not cols:
                raise ValueError(
                    f"{h5_path}: stream {name!r} missing 'columns' attr; "
                    "old episodes are retired"
                )
            for c in cols:
                if c.startswith("dim_"):
                    raise ValueError(
                        f"{h5_path}: stream {name!r} uses legacy dim_N naming ({c}); "
                        "old episodes are retired"
                    )
                if c in seen:
                    duplicates.add(c)
                seen.add(c)
                state_names.append(c)
        if duplicates:
            raise ValueError(
                f"{h5_path}: duplicate feature names across streams: {sorted(duplicates)}"
            )

        total_state_dim = len(state_names)

        # Warn on per-stream rate deviations from target fps
        for name in all_vectors + image_streams:
            rate = stream_cache[name]["sample_rate"]
            if rate <= 0.0:
                continue
            deviation = abs(rate - fps) / fps
            if deviation > 0.20:
                print(
                    f"WARNING: {h5_path}: stream {name!r} sample_rate={rate:.2f}Hz "
                    f"deviates {deviation*100:.0f}% from target {fps}Hz",
                    file=sys.stderr,
                )

        # Detect actual image dimensions
        image_dims = {}
        for cam in image_streams:
            grp = f[cam]
            w = int(grp.attrs.get("width", 0))
            h = int(grp.attrs.get("height", 0))
            if w <= 0 or h <= 0:
                import cv2
                jpeg = bytes(stream_cache[cam]["data"][0])
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                h, w = img.shape[:2]
            image_dims[cam] = (h, w)

        # Build feature schema
        features = {
            "observation.state": {
                "dtype": "float32", "shape": (total_state_dim,),
                "names": state_names,
            },
            "action": {
                "dtype": "float32", "shape": (total_state_dim,),
                "names": state_names,
            },
        }
        for cam in image_streams:
            h, w = image_dims[cam]
            features[f"observation.images.{cam}"] = {
                "dtype": "video", "shape": (h, w, 3),
                "names": ["height", "width", "rgb"],
            }

        print(f"  Features: state dim={total_state_dim}, "
              f"cameras={len(image_streams)}")

        # Create dataset
        dataset = LeRobotDataset.create(
            repo_id=repo_id, root=output_path, fps=fps,
            robot_type=robot_type, features=features,
            vcodec=vcodec,
        )

        task_prompt = str(f.attrs.get("task_prompt", ""))

        # Convert frames
        for i in range(n_frames):
            query_ts = int(canon_ts[i])
            frame = {}

            # Collect vector state
            state_parts = []
            for name in all_vectors:
                sc = stream_cache[name]
                idx = _nearest_idx(sc["ts"], query_ts)
                vals = sc["data"][idx]
                if vals.ndim == 0:
                    state_parts.append(np.array([float(vals)], dtype=np.float32))
                else:
                    state_parts.append(np.array(vals, dtype=np.float32))
            state = np.concatenate(state_parts).astype(np.float32)
            frame["observation.state"] = state
            frame["action"] = state  # teleop: action = observed state
            frame["task"] = task_prompt

            # Decode images
            for cam in image_streams:
                sc = stream_cache[cam]
                idx = _nearest_idx(sc["ts"], query_ts)
                jpeg_bytes = bytes(sc["data"][idx])
                import cv2
                img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
                frame[f"observation.images.{cam}"] = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            dataset.add_frame(frame)

            if (i + 1) % 200 == 0:
                print(f"    frame {i + 1}/{n_frames}")

        dataset.save_episode()
        dataset.finalize()
        print(f"  Saved: {n_frames} frames")

    return str(output_path)


def validate_lerobot(dataset_path: Path) -> dict:
    """Basic validation of LeRobot dataset output."""
    report = {"errors": [], "warnings": []}

    meta = dataset_path / "meta"
    if not (meta / "info.json").exists():
        report["errors"].append("meta/info.json missing")
    if not (meta / "stats.json").exists():
        report["errors"].append("meta/stats.json missing")

    # Check info.json content
    with open(meta / "info.json") as f:
        info = json.load(f)
    report["info"] = {"fps": info.get("fps"), "robot_type": info.get("robot_type"),
                      "total_episodes": info.get("total_episodes", "?")}

    # Check stats.json content
    with open(meta / "stats.json") as f:
        stats = json.load(f)
    stat_keys = list(stats.keys())
    report["stats_keys"] = len(stat_keys)

    # Check data parquet exists
    data_dir = dataset_path / "data"
    parquets = sorted(data_dir.glob("**/*.parquet"))
    report["data_parquet_count"] = len(parquets)

    # Check videos
    videos_dir = dataset_path / "videos"
    mp4s = sorted(videos_dir.glob("**/*.mp4"))
    report["video_count"] = len(mp4s)
    for mp4 in mp4s:
        size_mb = mp4.stat().st_size / 1024 / 1024
        report[f"video_{mp4.parent.name}"] = f"{size_mb:.1f} MB"

    # Check episodes metadata
    ep_dir = meta / "episodes"
    ep_parquets = sorted(ep_dir.glob("**/*.parquet")) if ep_dir.exists() else []
    report["episode_meta_count"] = len(ep_parquets)

    report["valid"] = len(report["errors"]) == 0
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert AIRS HDF5 to LeRobot v3")
    parser.add_argument("--hdf5", required=True, help="Path to AIRS .h5 file")
    parser.add_argument("--output", default="convert/test_artifacts/lerobot_dataset")
    parser.add_argument("--fps", type=int, default=60, help="Canonical frame rate")
    parser.add_argument("--robot-type", default="openarm")
    parser.add_argument("--vcodec", default="h264",
                        help="Video codec: h264, libsvtav1, hevc")
    parser.add_argument("--exclude-failure-demos", action="store_true",
                        help="Skip episodes where task_completed=False")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent.parent.parent
    h5 = project_root / args.hdf5
    out = project_root / args.output

    if ".trash" in h5.parts:
        print(f"Skipping .trash episode: {h5}")
        sys.exit(0)

    if args.validate_only:
        report = validate_lerobot(out)
        print(json.dumps(report, indent=2))
        sys.exit(0 if report["valid"] else 1)

    path = convert_to_lerobot(h5, out, fps=args.fps,
                              robot_type=args.robot_type, vcodec=args.vcodec,
                              exclude_failure_demos=args.exclude_failure_demos)
    print(f"\n  Dataset: {path}")

    # Validate
    report = validate_lerobot(Path(path))
    status = "PASS" if report["valid"] else "FAIL"
    print(f"  Validation: {status}")
    print(f"  Info: {report.get('info', {})}")
    print(f"  Parquet files: {report.get('data_parquet_count', 0)}")
    print(f"  MP4 videos: {report.get('video_count', 0)}")


if __name__ == "__main__":
    main()
