# VR→IK Auto-Repin on Reconnect

This document summarizes the auto-repin feature added to the OpenArm IK ROS2 adaptor so that a VR dropout does not force the operator through the full B→A re-calibration sequence again.

## Problem

During teleoperation the VR headset or controller can lose tracking or power off (battery short, USB disconnect, SteamVR restart, etc.). When the VR stream came back, the previous implementation treated the reconnect as a fresh session:

- The controller had to be re-pinned with **B**.
- Then re-activated with **A**.

This is slow and error-prone while the robot is holding a pose or mid-task.

## Solution

Approach 1: **auto-repin on reconnect, preserve calibration intent.**

When VR connectivity is restored, the adaptor automatically re-pins the controller relative to the robot's current base frame using the last-known good offset logic. The key change is that the "B has already been pressed" state is now remembered, so the operator only needs to press **A** to resume control.

### Behavior matrix

| VR state | A-button effect |
|---|---|
| First connect / hard reset | Does nothing until B is pressed to pin |
| B pressed, then A pressed | Activates control (unchanged) |
| VR drops while active, then reconnects | Auto-repin runs silently; pressing A resumes control |
| `auto_repin_on_reconnect: false` | Old behavior: B must be pressed again after reconnect |

## Configuration

`config/vr.yaml` now carries a calibration flag:

```yaml
calibration:
  auto_repin_on_reconnect: true
```

`server/config_loader.py` parses this into `CalibrationConfig.auto_repin_on_reconnect` (defaults to `True`).

## Implementation details

- `server/vr_normalizer.py`
  - Tracks `_was_calibrated` and `_last_connected`.
  - Splits `_reset(...)` into hard and soft variants:
    - **Hard reset** (no head pose, explicit home reset) clears calibration state.
    - **Soft reset** (temporary disconnect) preserves `_was_calibrated`.
  - Detects reconnect via `_last_connected == False → current == True`.
  - On reconnect, if `auto_repin_on_reconnect` is enabled and the controller was previously calibrated, it calls `_pin_and_calibrate(...)` automatically and returns `READY`.
  - A-button gating now checks `_was_calibrated` instead of requiring a fresh B press.
  - **Bug fix:** `_reset` no longer mutates the `_home_ee_source` dict in place. `_home_ee` is replaced with a fresh empty dict on reset, and `_pin_and_calibrate` copies the supplied home poses. This prevents the auto-repin path from restoring an empty `_home_ee`, which caused the arms to collapse to the base-frame origin on resume.

- `server/solver_loop.py`
  - `reset_home` with no head pose now calls `_reset(hard=True)` so that starting without VR truly resets state.

## Tests

`tests/test_vr_normalizer.py` covers the new behavior:

- `test_auto_repin_after_disconnect` — reconnect auto-repins and A resumes control.
- `test_manual_b_required_when_auto_repin_disabled` — with the flag off, B is required again after disconnect.
- `test_auto_repin_does_not_go_active_without_a` — auto-repin only puts the controller in `READY`, not `ACTIVE`.
- `test_hard_reset_clears_calibration_state` — hard reset wipes the remembered calibration flag.
- `test_home_ee_source_not_mutated_on_reset` — regression test ensuring the shared `home_fk` dict is not cleared on reset, so auto-repin restores valid IK home poses.

Run them with:

```bash
cd /home/intern/ros2-test/airspeed/src/robot_interface/openarm/openarm-ik-ros2-adaptor
PYTHONPATH=.pydeps:. python -m pytest tests/test_vr_normalizer.py -v -p no:anyio
```

## Files changed

- `config/vr.yaml`
- `server/config_loader.py`
- `server/vr_normalizer.py`
- `server/solver_loop.py`
- `tests/test_vr_normalizer.py`
