# ADR 0002: Episode outcome is two axes, not one success flag

## Status

Accepted (2026-07-14)

## Context

The writer stamped a single `success` attr per episode, conflating two
independent questions:

1. **Did the robot complete the task?** (the label a trainer needs for
   success/failure learning)
2. **Is the recorded data usable?** (tracking glitches, stream dropouts,
   operator judgment)

Consequences of the conflation: an aborted demonstration was marked
`success=False` and became indistinguishable from corrupt data, so failure
demonstrations — valuable training signal — were either lost or poisoned
the "good data" pool. Deleted episodes were moved to `.trash` unstamped,
leaving no record of *why* they were rejected.

## Decision

Two orthogonal root attrs per episode, plus a disposition reason:

| attr | values | set when |
|---|---|---|
| `task_completed` | bool | at close: True on A-stop (goal reached), False on B-abort |
| `recording_valid` | bool | True at close; stamped False when the operator deletes |
| `termination_reason` | `goal_reached` / `operator_abort` / `recording_invalid` | at close, re-stamped on delete |

Control mapping (context-sensitive B button):

- **A (toggle) stop** → `task_completed=True, recording_valid=True,
  termination_reason=goal_reached` — episode kept.
- **B during recording** → `task_completed=False, recording_valid=True,
  termination_reason=operator_abort` — episode **kept as a failure demo**.
- **B in pending** (delete) → reopen the closed file, stamp
  `recording_valid=False, termination_reason=recording_invalid`, then move
  to `.trash`. `task_completed` is left untouched: it records what the robot
  did, not whether the data is usable.

The legacy `success` attr is removed entirely (old-data-retired). Internal
consumers (dataset validator, converter) read the two axes instead.

## Why not keep one flag

- A single flag forces "abort" and "bad data" into the same bucket; the
  pipeline's whole purpose is producing labeled success *and* failure demos.
- Stamping invalidity at delete time (rather than not writing the file)
  keeps the audit trail: a trashed episode says *why* it was rejected.
- The recording layer stores facts; downstream decides policy (e.g. train
  on `recording_valid=True`, weight by `task_completed`).

## Consequences

- Failure demonstrations are first-class artifacts, kept in the task folder.
- The B button is context-sensitive: the router state-gates abort (only
  while recording) vs delete (only while pending) on the same binding index.
- Legacy episodes with only `success` are retired; the validator requires
  the three outcome attrs and reports older files as invalid.
