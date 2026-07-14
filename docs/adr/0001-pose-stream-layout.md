# ADR 0001: Pose streams record orientation-first, w-first

## Status

Accepted (2026-07-14)

## Context

Pose streams (`geometry_msgs/PoseStamped`: VR headset, VR controllers, IK
targets) are flattened by the adapter layer into a single row of 7 floats per
frame. The flattening walks message fields in **sorted-key order**, so the
recorded row is:

```
[orientation.w, orientation.x, orientation.y, orientation.z,
 position.x, position.y, position.z]
```

i.e. **quaternion first, w first; position last** — not the
`[x, y, z, qx, qy, qz, qw]` layout that downstream consumers (and earlier
pipeline documentation) assumed.

This was discovered back-propagated from a real pipeline-produced `.h5` file:
nothing in the file described what each of the 7 numbers meant, so the
assumed layout went unchallenged until values were inspected.

## Decision

1. **Keep the recorded layout as-is** — `[qw, qx, qy, qz, px, py, pz]`.
   This is a relabel, not a reorder: no data transformation is introduced in
   the hot recording path.
2. **Declare the meaning explicitly.** Every stream now carries a `columns:`
   list in the session YAML (`config/session_*.yaml`), stored verbatim into
   the HDF5 group attribute `columns` (JSON array). Pose columns are named
   `<prefix>_qw … <prefix>_qz, <prefix>_px … <prefix>_pz` to make the w-first
   order self-evident.
3. **No anonymous vector data.** Streams with opaque sequence payloads
   (e.g. `Float32MultiArray` buttons, `JointState.position`) must declare
   `columns:` in the YAML; `resolve_session` fails at startup otherwise.
   The writer rejects samples whose width differs from the declared column
   count and refuses to close an episode without column names (the old
   `dim_N` fallback naming is removed).

## Why not reorder to position-first

- Reordering adds a per-frame transformation in the recording hot path and a
  second layout to reason about (message layout vs. recorded layout).
- Sorted-key flattening is the generic rule for *all* scalar-field payloads;
  special-casing poses would break that uniformity.
- With explicit column names flowing YAML → h5 → converter → LeRobot feature
  names, the physical order inside the row is no longer load-bearing —
  consumers index by name.

## Consequences

- Column names are authored once in the session YAML and flow unchanged
  through the pipeline; the converter maps them to LeRobot features verbatim.
- Scalar-field streams (like `PoseStamped`) can auto-derive names from sorted
  field paths, which provably matches the recorded order; YAML `columns:` is
  still authored for readability.
- Legacy episodes recorded before this change have no `columns` attribute
  and are retired: downstream tooling must fail fast on missing `columns`
  rather than guess.
