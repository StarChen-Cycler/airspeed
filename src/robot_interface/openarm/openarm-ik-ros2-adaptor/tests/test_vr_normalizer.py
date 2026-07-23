"""Tests for VRNormalizer auto-repin behavior."""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

# The adaptor bundles its own JAX/jaxlie in .pydeps.
_PYDEPS = Path(__file__).resolve().parent.parent / ".pydeps"
if str(_PYDEPS) not in sys.path:
    sys.path.insert(0, str(_PYDEPS))

import jaxlie
import pytest

from server.config_loader import load_config
from server.vr_normalizer import VRNormalizer, CalibrationState
from server.vr_subscriber import VRDataStore, VRPose, VRButtonState


@pytest.fixture
def config():
    return load_config(Path(__file__).resolve().parent.parent / "config")


def _make_data_store(
    *,
    connected: bool = True,
    head_pos: list[float] | None = None,
    left_pos: list[float] | None = None,
    right_pos: list[float] | None = None,
    left_buttons: list[float] | None = None,
    right_buttons: list[float] | None = None,
) -> VRDataStore:
    """Build a VRDataStore with deterministic fresh/stale data."""
    store = VRDataStore(stale_timeout_s=1.0)
    now = time.monotonic()
    with store._lock:
        store._connected = connected
        store._last_message_s = now if connected else now - 10.0
        if head_pos is not None:
            store.head_pose = VRPose(
                position=head_pos, orientation_wxyz=[1.0, 0.0, 0.0, 0.0], timestamp_s=now
            )
        if left_pos is not None:
            store.left_pose = VRPose(
                position=left_pos, orientation_wxyz=[1.0, 0.0, 0.0, 0.0], timestamp_s=now
            )
        if right_pos is not None:
            store.right_pose = VRPose(
                position=right_pos, orientation_wxyz=[1.0, 0.0, 0.0, 0.0], timestamp_s=now
            )
        if left_buttons is not None:
            store.left_buttons = VRButtonState(buttons=left_buttons, timestamp_s=now)
        if right_buttons is not None:
            store.right_buttons = VRButtonState(buttons=right_buttons, timestamp_s=now)
    return store


def _set_buttons(store: VRDataStore, right: list[float] | None = None) -> None:
    """Set right controller button values."""
    with store._lock:
        now = time.monotonic()
        if right is not None:
            store.right_buttons = VRButtonState(buttons=right, timestamp_s=now)


def _set_connection(store: VRDataStore, connected: bool) -> None:
    """Toggle connection state by aging or freshening the last message time."""
    with store._lock:
        now = time.monotonic()
        store._connected = connected
        store._last_message_s = now if connected else now - 10.0
        # Also age/freshen pose timestamps so is_connected() and getters agree.
        for attr in ("head_pose", "left_pose", "right_pose"):
            pose = getattr(store, attr)
            if pose is not None:
                pose.timestamp_s = now if connected else now - 10.0
        for attr in ("left_buttons", "right_buttons"):
            state = getattr(store, attr)
            if state is not None:
                state.timestamp_s = now if connected else now - 10.0


def test_auto_repin_after_disconnect(config):
    """After ACTIVE -> stale -> reconnect, normalizer auto-repins to READY."""
    norm = VRNormalizer(config.vr)
    norm.set_home_ee({"left": None, "right": None})  # tests do not need real FK

    # 1. Connect and calibrate with B (index 5).
    store = _make_data_store(
        head_pos=[0.0, 1.7, 0.0],
        left_pos=[-0.2, 1.4, 0.3],
        right_pos=[0.2, 1.4, 0.3],
        right_buttons=[0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    )
    result = norm.normalize(store)
    assert result.state == CalibrationState.READY
    assert norm._was_calibrated

    # 2. Release B so the next press is a rising edge.
    _set_buttons(store, right=[0.0] * 6)
    norm.normalize(store)

    # 3. Activate with A (index 4).
    _set_buttons(store, right=[0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    result = norm.normalize(store)
    assert result.state == CalibrationState.ACTIVE

    # 4. Release A before disconnect.
    _set_buttons(store, right=[0.0] * 6)
    norm.normalize(store)

    # 5. Disconnect (stale data) and wait for the normalizer's activity timeout.
    _set_connection(store, connected=False)
    time.sleep(config.vr.calibration.stale_timeout_s + 0.1)
    result = norm.normalize(store)
    assert result.state == CalibrationState.WAITING
    assert norm._was_calibrated  # B-state preserved for auto-repin

    # 6. Reconnect with fresh data -> auto-repin to READY.
    _set_connection(store, connected=True)
    result = norm.normalize(store)
    assert result.state == CalibrationState.READY, f"expected READY, got {result.state}"

    # 7. Press A again to resume ACTIVE.
    _set_buttons(store, right=[0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    result = norm.normalize(store)
    assert result.state == CalibrationState.ACTIVE


def test_manual_b_required_when_auto_repin_disabled(config):
    """With auto_repin disabled, reconnect stays in WAITING until B is pressed."""
    config.vr.calibration.auto_repin_on_reconnect = False
    norm = VRNormalizer(config.vr)
    norm.set_home_ee({"left": None, "right": None})

    store = _make_data_store(
        head_pos=[0.0, 1.7, 0.0],
        left_pos=[-0.2, 1.4, 0.3],
        right_pos=[0.2, 1.4, 0.3],
        right_buttons=[0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    )
    norm.normalize(store)  # B pins
    _set_buttons(store, right=[0.0] * 6)

    # Disconnect and wait for the activity timeout.
    _set_connection(store, connected=False)
    time.sleep(config.vr.calibration.stale_timeout_s + 0.1)
    result = norm.normalize(store)
    assert result.state == CalibrationState.WAITING

    # Reconnect -> still WAITING because auto-repin is disabled.
    _set_connection(store, connected=True)
    result = norm.normalize(store)
    assert result.state == CalibrationState.WAITING

    # Press B again -> READY.
    _set_buttons(store, right=[0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    result = norm.normalize(store)
    assert result.state == CalibrationState.READY


def test_auto_repin_does_not_go_active_without_a(config):
    """Auto-repin reaches READY, not ACTIVE."""
    norm = VRNormalizer(config.vr)
    norm.set_home_ee({"left": None, "right": None})

    store = _make_data_store(
        head_pos=[0.0, 1.7, 0.0],
        left_pos=[-0.2, 1.4, 0.3],
        right_pos=[0.2, 1.4, 0.3],
        right_buttons=[0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    )
    norm.normalize(store)  # B
    _set_buttons(store, right=[0.0] * 6)

    # Disconnect and reconnect without pressing A.
    _set_connection(store, connected=False)
    norm.normalize(store)
    _set_connection(store, connected=True)
    result = norm.normalize(store)
    assert result.state == CalibrationState.READY


def test_hard_reset_clears_calibration_state(config):
    """Explicit hard reset forgets calibration state."""
    norm = VRNormalizer(config.vr)
    norm.set_home_ee({"left": None, "right": None})

    store = _make_data_store(
        head_pos=[0.0, 1.7, 0.0],
        right_buttons=[0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    )
    norm.normalize(store)
    assert norm._was_calibrated

    norm._reset(hard=True)
    assert not norm._was_calibrated
    assert norm._needs_pin
    assert norm.state == CalibrationState.WAITING


def test_home_ee_source_not_mutated_on_reset(config):
    """Regression: _reset must not clear the dict supplied via set_home_ee."""
    norm = VRNormalizer(config.vr)
    home_ee = {
        "left": jaxlie.SE3.identity(),
        "right": jaxlie.SE3.identity(),
    }
    norm.set_home_ee(home_ee)

    store = _make_data_store(
        head_pos=[0.0, 1.7, 0.0],
        left_pos=[-0.2, 1.4, 0.3],
        right_pos=[0.2, 1.4, 0.3],
        right_buttons=[0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    )
    norm.normalize(store)  # B pins
    assert norm._home_ee
    assert "left" in home_ee and "right" in home_ee

    # Disconnect triggers the soft _reset.
    _set_connection(store, connected=False)
    time.sleep(config.vr.calibration.stale_timeout_s + 0.1)
    norm.normalize(store)
    assert norm._was_calibrated  # soft reset kept calibration intent

    # The original source dict must still be intact.
    assert "left" in home_ee and "right" in home_ee

    # Reconnect auto-repins and restores _home_ee from the still-valid source.
    _set_connection(store, connected=True)
    result = norm.normalize(store)
    assert result.state == CalibrationState.READY
    assert "left" in norm._home_ee and "right" in norm._home_ee
