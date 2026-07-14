# ADR 0003: Delete the threaded writer twin

## Status

Accepted (2026-07-14)

## Context

The repository contained two parallel implementations of the recording path:

- `core/storage/airs_hdf5_writer.py` + `core/runtime/ros2_collection_node.py` —
  the live, single-threaded path used by the ROS2 collection node.
- `core/storage/threaded/airs_hdf5_writer.py` + `core/runtime/threaded/ros2_collection_node.py` —
  a "thread-safe" twin intended for use with `MultiThreadedExecutor`.

The threaded twin had drifted out of sync with the live path: it lacked
`set_task`/task metadata, `move_to_trash`, `stamp_recording_invalid`,
`-T#` episode numbering, per-stream `sample_rate`, semantic `columns`,
camera `width`/`height` attrs, and the two-axis outcome model. Nothing in
launch files or documentation instantiated it, so it was dead code.

## Decision

1. **Delete the threaded subtrees** instead of porting the metadata fixes.
2. **Keep the single-threaded path** as the only supported writer.
3. **Add `tools/benchmark_writer_latency.py`** so the decision can be revisited
   with measured data if the workload changes.

## Rationale

A synthetic full-load benchmark (15 streams: 5 VR @ 62 Hz, 4 IK @ 44 Hz,
2 arm state @ 30 Hz, 3 cameras @ 14.6 Hz, with JPEG re-encode and gzip)
was run for 10 seconds on the live single-threaded writer:

| metric | value |
|--------|-------|
| total frames written | 5,898 |
| mean append latency | 0.19 ms |
| p99 append latency | 0.84 ms |
| max append latency | 46.4 ms (single outlier, likely HDF5 resize) |
| dropped frames | 0 |

The p99 latency is well below the 5 ms threshold that would justify threading,
and no frames were dropped. The threaded twin would add complexity and a
second code path to maintain without solving a measured problem. If future
camera counts, resolutions, or frame rates push p99 above 5 ms, the benchmark
script provides the starting point for a real threaded rewrite.

## Consequences

- Less code to maintain; no risk of the twin silently diverging again.
- The collection node must continue to run with a `SingleThreadedExecutor`
  or a multi-threaded executor that serializes writer access.
- Any future need for concurrent callbacks must re-implement the writer
  from the current single-threaded baseline, not revive the deleted twin.
