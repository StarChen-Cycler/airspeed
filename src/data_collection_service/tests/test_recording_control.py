"""Tests for recording control routing and two-axis outcome model."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from core.config import RecordingControlBinding, RecordingControlConfig, RecordingControlMode
from core.runtime.recording_control import RecordingControlRouter
from core.runtime.recording_state import RecordingStateMachine


def _make_router(mode=RecordingControlMode.SERVICE, bindings=(), on_delete=None):
    cfg = RecordingControlConfig(mode=mode, toggle_debounce_s=0.0, bindings=bindings)
    sm = RecordingStateMachine(start_handler=lambda e: None, end_handler=lambda tc, r: None)
    return RecordingControlRouter(cfg, sm, on_delete_requested=on_delete)


def _bind(action, stream_name, button_index):
    return (action, RecordingControlBinding(
        stream_name=stream_name, button_index=button_index, threshold=0.5))


def test_service_stop_maps_to_goal_reached():
    ended = {}
    sm = RecordingStateMachine(start_handler=lambda e: None, end_handler=lambda tc, r: ended.update(tc=tc, r=r))
    router = RecordingControlRouter(
        RecordingControlConfig(mode=RecordingControlMode.SERVICE), sm)
    sm.start_episode("ep")
    result = router.handle_service_action("stop")
    assert result.accepted and result.action == "stop"
    assert ended["tc"] is True and ended["r"] == "goal_reached"


def test_service_abort_maps_to_operator_abort_failure_demo():
    ended = {}
    sm = RecordingStateMachine(start_handler=lambda e: None, end_handler=lambda tc, r: ended.update(tc=tc, r=r))
    router = RecordingControlRouter(
        RecordingControlConfig(mode=RecordingControlMode.SERVICE), sm)
    sm.start_episode("ep")
    result = router.handle_service_action("abort")
    assert result.accepted and result.action == "abort"
    assert ended["tc"] is False and ended["r"] == "operator_abort"
    assert router.pending_episode is False


def test_device_binding_b_context_sensitive():
    ended = {}
    deleted = {"ok": False}
    sm = RecordingStateMachine(start_handler=lambda e: None, end_handler=lambda tc, r: ended.update(tc=tc, r=r))
    router = RecordingControlRouter(
        RecordingControlConfig(
            mode=RecordingControlMode.DEVICE_BINDING,
            bindings=(_bind("toggle", "b", 5), _bind("abort", "b", 4), _bind("delete", "b", 4)),
        ),
        sm,
        on_delete_requested=lambda: deleted.update(ok=True) or True,
    )

    def press(idx):
        data = [0.0] * 6
        data[idx] = 1.0
        return SimpleNamespace(data=data)
    def release():
        return SimpleNamespace(data=[0.0] * 6)

    # start
    assert router.handle_stream_message("b", press(5)).action == "start"
    router.handle_stream_message("b", release())
    assert sm.is_recording

    # B during recording -> abort
    r = router.handle_stream_message("b", press(4))
    assert r.accepted and r.action == "abort"
    router.handle_stream_message("b", release())

    # start, stop to enter pending
    assert router.handle_stream_message("b", press(5)).action == "start"
    router.handle_stream_message("b", release())
    assert router.handle_stream_message("b", press(5)).action == "stop"
    router.handle_stream_message("b", release())
    assert router.pending_episode

    # B in pending -> delete
    r = router.handle_stream_message("b", press(4))
    assert r.accepted and r.action == "delete"
    assert deleted["ok"]


def test_device_binding_joy_axes_field():
    """VR controllers publish button values in sensor_msgs/Joy.axes with buttons=[]."""
    sm = RecordingStateMachine(start_handler=lambda e: None, end_handler=lambda tc, r: None)
    router = RecordingControlRouter(
        RecordingControlConfig(
            mode=RecordingControlMode.DEVICE_BINDING,
            bindings=(
                ("toggle", RecordingControlBinding(
                    stream_name="vr_left_buttons", field_name="axes",
                    button_index=5, threshold=0.5)),
            ),
        ),
        sm,
    )

    def press(idx):
        axes = [0.0] * 6
        axes[idx] = 1.0
        # Real ROS Joy messages expose both axes and buttons; buttons is empty here.
        return SimpleNamespace(axes=axes, buttons=[])

    def release():
        return SimpleNamespace(axes=[0.0] * 6, buttons=[])

    r = router.handle_stream_message("vr_left_buttons", press(5))
    assert r.accepted and r.action == "start"
    router.handle_stream_message("vr_left_buttons", release())
    assert sm.is_recording


def test_service_delete_works_as_override_in_device_binding_mode():
    """Service calls must still work when the active mode is device_binding
    (e.g. VR controller battery dies and the operator needs to trash the
    pending episode via CLI/service)."""
    deleted = {"ok": False}
    sm = RecordingStateMachine(start_handler=lambda e: None, end_handler=lambda tc, r: None)
    router = RecordingControlRouter(
        RecordingControlConfig(
            mode=RecordingControlMode.DEVICE_BINDING,
            bindings=(_bind("toggle", "b", 5),),
        ),
        sm,
        on_delete_requested=lambda: deleted.update(ok=True) or True,
    )

    # Start and stop an episode via service even though mode is device_binding.
    r = router.handle_service_action("start")
    assert r.accepted and r.action == "start"
    r = router.handle_service_action("stop")
    assert r.accepted and r.action == "stop"
    assert router.pending_episode

    # Delete the pending episode via service override.
    r = router.handle_service_delete()
    assert r.accepted and r.action == "delete"
    assert deleted["ok"]
