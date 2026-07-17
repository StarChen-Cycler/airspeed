"""AIRS-standard HDF5 writer: one file per episode, flat per-stream groups.

Uses chunked append-backed datasets: frames are buffered in small batches
and flushed periodically, keeping memory constant regardless of episode
length while amortizing HDF5 resize overhead.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
from typing import Any

import h5py
import numpy as np


class AirsHdf5WriterError(ValueError):
    """Raised when AIRS HDF5 writing cannot proceed safely."""


# Flush every N frames to amortize HDF5 resize cost
_VECTOR_BATCH = 50
_IMAGE_BATCH = 20


class AirsHdf5Writer:
    """Write one AIRS-standard HDF5 file per episode.

    Layout::

        <episode_id>.h5
        ├── / (root attrs: description, robot_type, series_number,
        │        sample_rate, frames, success, termination_reason)
        ├── <stream_name>/
        │     ├── attrs: type, columns (vector) / width,height,channels,encoding (image)
        │     ├── data:       (N, D) float32  or  (N,) vlen uint8
        │     └── timestamps: (N,) uint64
        └── ...
    """

    def __init__(
        self,
        output_dir: str | Path,
        *,
        description: str = "",
        robot_type: str = "",
        series_number: str = "",
    ) -> None:
        self._output_dir = Path(output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._description = description
        self._robot_type = robot_type
        self._series_number = series_number
        self._file: h5py.File | None = None
        self._streams: dict[str, _StreamBuffer] = {}
        self._task_name: str | None = None
        self._task_meta: dict | None = None

    def set_task(self, task_name: str | None, task_meta: dict | None = None) -> None:
        """Set active task. Episodes are written to {output_dir}/{task_name}/.

        task_meta carries the fields written as root attrs at close_episode:
        task_prompt, task_id, task_structure, deformable_objects.
        """
        self._task_name = task_name
        self._task_meta = task_meta

    # -- episode lifecycle --

    def open_episode(self, episode_id: str) -> None:
        if self._file is not None:
            raise AirsHdf5WriterError("an episode is already open; close it first")
        out_dir = self._output_dir
        if self._task_name:
            out_dir = out_dir / self._task_name
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"{episode_id}.h5"
        self._file = h5py.File(path, "w")
        self._streams = {}

    def close_episode(self, *, task_completed: bool,
                      termination_reason: str) -> str:
        """Close the open episode. Freshly closed episodes are valid;
        recording_valid is stamped False only by stamp_recording_invalid."""
        if self._file is None:
            raise AirsHdf5WriterError("no episode is open")
        total_frames = 0
        empty_streams = []
        for name, buf in self._streams.items():
            buf.flush_remaining()
            total_frames = max(total_frames, buf.frame_count)
            if buf.frame_count == 0:
                empty_streams.append(name)
        # Streams that produced no samples are omitted from the file: a
        # zero-length dataset fails dataset validation and cannot be
        # displayed by HDF viewers. A never-fed vector stream may not even
        # have a 'data' dataset (dims are detected at first sample). Stream
        # absence remains observable at runtime via the stream tracker.
        for name in empty_streams:
            del self._file[name]
            del self._streams[name]
        self._file.attrs["description"] = self._description
        self._file.attrs["robot_type"] = self._robot_type
        self._file.attrs["series_number"] = self._series_number
        self._file.attrs["frames"] = total_frames
        self._file.attrs["task_completed"] = bool(task_completed)
        self._file.attrs["recording_valid"] = True
        self._file.attrs["termination_reason"] = termination_reason
        if self._task_meta:
            self._file.attrs["task_name"] = str(self._task_meta.get("task_name", ""))
            self._file.attrs["task_prompt"] = str(self._task_meta.get("task_prompt", ""))
            self._file.attrs["task_id"] = str(self._task_meta.get("task_id", ""))
            self._file.attrs["task_structure"] = str(self._task_meta.get("task_structure", ""))
            self._file.attrs["deformable_objects"] = bool(
                self._task_meta.get("deformable_objects", False)
            )
        path = self._file.filename
        self._file.close()
        self._file = None
        return path

    @staticmethod
    def stamp_recording_invalid(episode_path: str | Path) -> None:
        """Mark a closed episode as invalid data (operator deleted it).

        Stamps recording_valid=False and termination_reason=recording_invalid.
        task_completed is left untouched — it records what the robot did,
        not whether the data is usable.
        """
        with h5py.File(episode_path, "r+") as f:
            f.attrs["recording_valid"] = False
            f.attrs["termination_reason"] = "recording_invalid"

    @staticmethod
    def move_to_trash(episode_path: str | Path) -> str:
        """Move an episode file to .trash/ under its parent directory.

        Returns the new path in .trash/, or the original path if the move failed.
        """
        src = Path(episode_path)
        if not src.exists():
            return str(src)
        trash_dir = src.parent / ".trash"
        trash_dir.mkdir(parents=True, exist_ok=True)
        dst = trash_dir / src.name
        shutil.move(str(src), str(dst))
        return str(dst)

    # -- per-stream registration (creates HDF5 group + datasets immediately) --

    def register_vector_stream(
        self, name: str, dims: int = 0, *, columns: tuple[str, ...] = (),
    ) -> None:
        self._require_open()
        if name in self._streams:
            raise AirsHdf5WriterError(f"stream {name!r} already registered")
        grp = self._file.create_group(name)
        grp.attrs["type"] = "vector"
        if columns:
            grp.attrs["columns"] = json.dumps(list(columns))
        if dims > 0:
            grp.create_dataset(
                "data", shape=(0, dims), maxshape=(None, dims),
                dtype=np.float32, chunks=(_VECTOR_BATCH, dims),
            )
        grp.create_dataset(
            "timestamps", shape=(0,), maxshape=(None,),
            dtype=np.uint64, chunks=(_VECTOR_BATCH,),
        )
        self._streams[name] = _VectorBuffer(name, grp, dims)

    def register_image_stream(
        self, name: str, *, width: int, height: int, channels: int,
        encoding: str = "raw",
    ) -> None:
        self._require_open()
        if name in self._streams:
            raise AirsHdf5WriterError(f"stream {name!r} already registered")
        grp = self._file.create_group(name)
        grp.attrs["type"] = "image"
        grp.attrs["width"] = width
        grp.attrs["height"] = height
        grp.attrs["channels"] = channels
        # Placeholder only — the first frame overrides this with the actual
        # message encoding. The writer never re-encodes: compression, when
        # wanted, happens only at the source adaptor.
        grp.attrs["encoding"] = encoding
        dt = h5py.vlen_dtype(np.dtype("uint8"))
        grp.create_dataset(
            "data", shape=(0,), maxshape=(None,),
            dtype=dt, chunks=(_IMAGE_BATCH,),
        )
        grp.create_dataset(
            "timestamps", shape=(0,), maxshape=(None,),
            dtype=np.uint64, chunks=(_IMAGE_BATCH,),
        )
        self._streams[name] = _ImageBuffer(name, grp)

    # -- append (hot path — buffers in memory, flushes in batches) --

    def append_vector(self, name: str, values: object, timestamp_ns: int) -> None:
        buf = self._streams.get(name)
        if not isinstance(buf, _VectorBuffer):
            raise AirsHdf5WriterError(f"{name!r} is not a registered vector stream")
        buf.append(values, timestamp_ns)

    def append_image(
        self, name: str, raw_data: bytes, timestamp_ns: int, *,
        width: int | None = None, height: int | None = None,
        encoding: str | None = None,
    ) -> None:
        buf = self._streams.get(name)
        if not isinstance(buf, _ImageBuffer):
            raise AirsHdf5WriterError(f"{name!r} is not a registered image stream")
        buf.append(raw_data, timestamp_ns, width=width, height=height, encoding=encoding)

    # -- helpers --

    def _require_open(self) -> None:
        if self._file is None:
            raise AirsHdf5WriterError("no episode is open")


# ---------------------------------------------------------------------------
# Chunked append-backed buffers
#
# Design: we don't append one row at a time (HDF5 resize() per frame is slow).
# Instead we buffer in memory and flush every _VECTOR_BATCH (50) / _IMAGE_BATCH
# (20) frames. Each flush resizes the dataset once and writes the entire batch
# contiguously. Memory is capped at one batch — a 10-hour episode uses the same
# memory as a 10-second one.
# ---------------------------------------------------------------------------


class _VectorBuffer:
    def __init__(self, name: str, grp: h5py.Group, dims: int) -> None:
        self.name = name
        self._grp = grp
        self._dims = dims
        cols = grp.attrs.get("columns")
        self._n_columns = len(json.loads(cols)) if cols is not None else None
        self._data_buf: list[np.ndarray] = []
        self._ts_buf: list[np.uint64] = []
        self._total = 0
        self._first_ts_ns: int | None = None
        self._last_ts_ns: int | None = None

    @property
    def frame_count(self) -> int:
        return self._total + len(self._data_buf)

    def append(self, values: object, timestamp_ns: int) -> None:
        arr = np.asarray(values, dtype=np.float32).ravel()
        if self._dims == 0:
            self._dims = arr.size
        elif arr.size != self._dims:
            raise AirsHdf5WriterError(
                f"{self.name}: dimension mismatch — expected {self._dims}, got {arr.size}"
            )
        if self._n_columns is not None and arr.size != self._n_columns:
            raise AirsHdf5WriterError(
                f"{self.name}: sample has {arr.size} values but "
                f"{self._n_columns} columns are declared"
            )
        if self._first_ts_ns is None:
            self._first_ts_ns = int(timestamp_ns)
        self._last_ts_ns = int(timestamp_ns)
        self._data_buf.append(arr.astype(np.float32))
        self._ts_buf.append(np.uint64(timestamp_ns))
        if len(self._data_buf) >= _VECTOR_BATCH:
            self._flush_batch()

    def _flush_batch(self) -> None:
        if not self._data_buf:
            return
        n = len(self._data_buf)
        data_arr = np.array(self._data_buf, dtype=np.float32)
        ts_arr = np.array(self._ts_buf, dtype=np.uint64)

        # Lazy dataset creation for variable-dimension streams
        if "data" not in self._grp:
            self._grp.create_dataset(
                "data", shape=(0, self._dims), maxshape=(None, self._dims),
                dtype=np.float32, chunks=(_VECTOR_BATCH, self._dims),
            )

        dset = self._grp["data"]
        tset = self._grp["timestamps"]
        new_size = self._total + n
        dset.resize((new_size, self._dims))
        tset.resize((new_size,))
        dset[self._total:new_size] = data_arr
        tset[self._total:new_size] = ts_arr

        self._total = new_size
        self._data_buf.clear()
        self._ts_buf.clear()

    def flush_remaining(self) -> None:
        self._flush_batch()
        self._grp.attrs["frames"] = self._total
        self._grp.attrs["sample_rate"] = _measured_rate(
            self._total, self._first_ts_ns, self._last_ts_ns
        )
        if "columns" not in self._grp.attrs:
            raise AirsHdf5WriterError(
                f"{self.name}: no column names registered; vector streams "
                "must declare columns (see session YAML 'columns:' key)"
            )


class _ImageBuffer:
    def __init__(self, name: str, grp: h5py.Group) -> None:
        self.name = name
        self._grp = grp
        self._frame_buf: list[bytes] = []
        self._ts_buf: list[np.uint64] = []
        self._total = 0
        self._first_ts_ns: int | None = None
        self._last_ts_ns: int | None = None
        self._dims_finalized = False  # width/height attrs set from first frame

    @property
    def frame_count(self) -> int:
        return self._total + len(self._frame_buf)

    def append(
        self, raw_data: bytes, timestamp_ns: int,
        *, width: int | None = None, height: int | None = None,
        encoding: str | None = None,
    ) -> None:
        if self._first_ts_ns is None:
            self._first_ts_ns = int(timestamp_ns)
            self._set_dimensions(raw_data, width, height)
            # Record the actual payload encoding (rgb8, jpeg, …) from the
            # first frame; bytes are stored verbatim, never re-encoded.
            if encoding:
                self._grp.attrs["encoding"] = encoding
        self._last_ts_ns = int(timestamp_ns)
        self._frame_buf.append(raw_data)
        self._ts_buf.append(np.uint64(timestamp_ns))
        if len(self._frame_buf) >= _IMAGE_BATCH:
            self._flush_batch()

    def _set_dimensions(
        self, raw_data: bytes, width: int | None, height: int | None,
    ) -> None:
        if not self._dims_finalized:
            w = int(width) if width else 0
            h = int(height) if height else 0
            if w <= 0 or h <= 0:
                decoded = _jpeg_dimensions(raw_data)
                if decoded is not None:
                    w, h = decoded
            if w > 0 and h > 0:
                self._grp.attrs["width"] = w
                self._grp.attrs["height"] = h
                self._dims_finalized = True

    def _flush_batch(self) -> None:
        if not self._frame_buf:
            return
        n = len(self._frame_buf)
        dset = self._grp["data"]
        tset = self._grp["timestamps"]
        new_size = self._total + n
        dset.resize((new_size,))
        tset.resize((new_size,))
        for i, frame in enumerate(self._frame_buf):
            dset[self._total + i] = np.frombuffer(frame, dtype=np.uint8)
        tset[self._total:new_size] = np.array(self._ts_buf, dtype=np.uint64)

        self._total = new_size
        self._frame_buf.clear()
        self._ts_buf.clear()

    def flush_remaining(self) -> None:
        self._flush_batch()
        self._grp.attrs["frames"] = self._total
        self._grp.attrs["sample_rate"] = _measured_rate(
            self._total, self._first_ts_ns, self._last_ts_ns
        )
        self._grp.attrs["camera_name"] = self.name


def _measured_rate(n_frames: int, first_ts_ns: int | None,
                   last_ts_ns: int | None) -> float:
    """Mean sample rate in Hz from a stream's own timestamps.

    0.0 when uncomputable (fewer than 2 frames or zero duration). The writer
    records facts; regularity interpretation belongs to downstream stages.
    """
    if n_frames < 2 or first_ts_ns is None or last_ts_ns is None:
        return 0.0
    duration_s = (last_ts_ns - first_ts_ns) / 1e9
    if duration_s <= 0.0:
        return 0.0
    return (n_frames - 1) / duration_s


def _jpeg_dimensions(raw: bytes) -> tuple[int, int] | None:
    """Return (width, height) from the first SOF0/SOF2 marker in a JPEG.

    Runs without external dependencies. Returns None if no SOF marker is found.
    """
    i = 0
    n = len(raw)
    while i < n:
        # Find next marker
        if raw[i] != 0xFF:
            i += 1
            continue
        # Skip padding 0xFF bytes
        while i + 1 < n and raw[i + 1] == 0xFF:
            i += 1
        if i + 1 >= n:
            return None
        marker = raw[i + 1]
        i += 2
        # SOF0 (baseline) or SOF2 (progressive)
        if marker in (0xC0, 0xC2):
            if i + 7 >= n:
                return None
            # length (2 bytes), precision (1), height (2), width (2)
            height = int.from_bytes(raw[i + 3:i + 5], "big")
            width = int.from_bytes(raw[i + 5:i + 7], "big")
            return width, height
        # Skip marker segment if it has a length field
        if marker not in (0x00, 0x01, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9):
            if i + 2 > n:
                return None
            length = int.from_bytes(raw[i:i + 2], "big")
            i += length
    return None


__all__ = ["AirsHdf5Writer", "AirsHdf5WriterError"]
