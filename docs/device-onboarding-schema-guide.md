# Device & Stream Onboarding — Canonical Schema Guide

**Audience**: anyone connecting a new device, topic, or project to the AIRSPEED
data collection service — a new teleop device, a new robot, a new sensor, or a
remote data source on another machine.

**The one rule that matters**: the data collection service is hardware-agnostic.
It digests anything that speaks the canonical contract — the right ROS2 message
type, on a declared topic, with a creation-time `header.stamp`, described by a
session YAML stream. If your publisher honors the contract, zero collector code
changes are needed.

---

## 1. The Canonical Pipeline

```
Your device SDK
  → your ROS2 publisher (interface adaptor)        # YOU write this
  → ROS2 topic with declared message type
  → session YAML stream declaration                # YOU declare this
  → adapter (payload extraction + boundary validation)
  → WriterSample (canonical envelope)
  → AIRS HDF5 episode (per-stream data + timestamps)
  → converters (Parquet / Zarr / LeRobot v3)
```

The collector's only knowledge of your device comes from the session YAML.
Everything else — message layout, timestamps, QoS — flows through the ROS2
message itself.

## 2. Message Contract

### 2.1 Canonical message types

| Data kind | Message type | Source family | Why this type |
|-----------|--------------|---------------|----------------|
| 6-DOF pose (hand, head, tool) | `geometry_msgs/PoseStamped` | teleop / robot | Header + position + quaternion |
| Joint state / joint commands | `sensor_msgs/JointState` | robot | Header + named joints + pos/vel/effort |
| Discrete inputs (buttons, triggers, pedals) | `sensor_msgs/Joy` | teleop | Header + `axes` for analog channels |
| Camera frame | `sensor_msgs/Image` | sensor | Header + encoding + bytes |
| Camera intrinsics | `sensor_msgs/CameraInfo` | sensor | Latched (TRANSIENT_LOCAL), once per camera |

Rules:

1. **Only types with a `std_msgs/Header` are accepted.** `JointState` for joint
   data, `Joy` for discrete inputs, `PoseStamped` for poses. Header-less types
   (`Float32MultiArray` and friends) are rejected at the adapter boundary. If
   your device SDK only produces header-less data, write an edge shim adaptor
   that stamps at receipt and republishes a canonical type — see §5 Pattern B.
2. **Analog-capable channels go in `Joy.axes`** (float32). `Joy.buttons` is
   int32 on/off; truncating analog triggers there loses resolution. The bundled
   VR bridge puts all 6 controller channels in `axes` and leaves `buttons` empty.
3. **One logical signal per topic.** Don't multiplex; declare one stream per
   topic in the session YAML.

### 2.2 The creation-time timestamp rule (normative)

`header.stamp` MUST be the time the data was **created** — not the time the
message happened to be published. Each interface defines what "created" means:

| Interface | Stream kind | `header.stamp` means |
|-----------|-------------|----------------------|
| Teleoperation | VR pose / button | Bridge receipt of the device frame (device POST arrival) |
| Robot (IK) | Joint commands, target poses | Solver handoff — the instant the solution was produced |
| Robot (control) | Joint state feedback | Immediately after the motor-bus read (≈ encoder sample) |
| Sensor | Image | Immediately after frame read returns, **before** any encoding |

Consequences for your publisher:

- Stamp **before** expensive processing (JPEG encode, serialization, transport),
  never after.
- One logical sample batch = one stamp, shared by all messages derived from it
  (e.g., both arms' `JointState` from one read; pose + buttons from one POST).
- Use the machine's steady wall clock (`node.get_clock().now()`). Do not set
  `use_sim_time`.
- If your device SDK exposes a hardware acquisition timestamp in the **same
  clock domain** as the collector machine, prefer it. A device timestamp in an
  unknown clock domain is worse than a local receipt stamp — it silently breaks
  cross-stream alignment.

The collector records `header.stamp` as the canonical HDF5 timestamp for every
stream with `time_domain: ros_header` (sub-microsecond `datetime` quantization,
uniform across streams).

## 3. Session YAML Schema Reference

Top level: `schema_version`, `session`, `storage`, `streams`. Unknown keys are
rejected; duplicate mapping keys are rejected.

### 3.1 `session`

| Key | Type | Required | Notes |
|-----|------|----------|-------|
| `name` | string | yes | Session identity |
| `task_id` | string | yes | **Collection profile** id (which topics/IK mode/bindings are active — not a robot task) |
| `operator_id` | string | yes | Written to episode attrs as `series_number` |
| `devices` | map | no | See below |
| `recording_control` | map | no | Defaults to `service` mode |
| `notes` | string | no | Free text |

`devices.<name>`:

| Key | Required | Notes |
|-----|----------|-------|
| `device_id` | yes | Unique device identifier |
| `role` | yes | `teleop` / `robot` / `sensor` |
| `driver_version`, `firmware_version`, `calibration_ref`, `serial_number` | no | Provenance metadata |

`recording_control`:

| Key | Values | Notes |
|-----|--------|-------|
| `mode` | `service` / `manual_ui` / `device_binding` | All modes drive one state machine |
| `toggle_debounce_s` | float ≥ 0 | Default 0.5 |
| `bindings` | actions `toggle` / `delete` / `abort` | Required in `device_binding` mode |

Each binding: `stream_name` (required), `button_index` (int ≥ 0), `threshold`
(float), `field_name` (default `buttons` — matches the adapter payload key).

### 3.2 `storage`

| Key | Default | Notes |
|-----|---------|-------|
| `root` | `data/episodes` | Episode output directory |
| `format` | `hdf5` | Only `hdf5` is valid |
| `compression` | `gzip` | HDF5 dataset compression |
| `config_hash_algorithm` | `sha256` | Only `sha256` is valid |

### 3.3 `streams.<stream_name>`

| Key | Type / values | Required | Notes |
|-----|---------------|----------|-------|
| `source` | `teleop` / `robot` / `sensor` | yes | Selects the adapter binding family |
| `topic` | string | yes | ROS2 topic to subscribe |
| `message_type` | string | yes | e.g. `sensor_msgs/Joy` — resolved dynamically; must match a binding `(source, message_type)` |
| `time_domain` | `ros_header` only | no (default `ros_header`) | Strict contract — every stream uses the creation-time header stamp; any other value fails session load |
| `qos.reliability` | `best_effort` / `reliable` | no | Default `best_effort` |
| `qos.durability` | `volatile` / `transient_local` | no | Default `volatile` |
| `qos.history` | `keep_last` / `keep_all` | no | Default `keep_last` |
| `qos.depth` | int ≥ 1 | no | Default 10; use 1–5 for high-rate streams |
| `expected_rate_hz` | float | no | Used by mock publishers and rate monitoring |
| `frame_id` | string | no | Fallback `frame_id` for payload extraction |
| `image_encoding` | `jpeg` / `raw` | Image streams only | `jpeg` = re-encode to JPEG at write; `raw` = store bytes as-is. Rejected on non-Image streams |
| `fields` | list of field rules | yes | Declarative payload contract (see 3.4) |
| `columns` | list of unique strings | required for sequence streams | Per-dimension labels → HDF5 `columns` attr → LeRobot feature names |
| `notes` | string | no | Free text |

### 3.4 `fields` rules

Each entry: `{path, type, required}`.

- `path` — dotted message path (`pose.position.x`, `axes`, `data`). For
  sequence payloads it documents the source field; extraction itself is done by
  the binding's payload builder.
- `type` — one of `float64`, `float32`, `int32`, `uint32`, `uint64`, `string`,
  `bytes`, `bool`, `sequence`.
- `required` — default `true`.

A stream whose payload is a bare sequence (`JointState.position`, `Joy.axes`)
declares exactly one `sequence` field; dimension count is detected at runtime
and validated against `columns` width. Scalar-field streams (poses) declare
each component; recorded order is **sorted-key flattening**, so auto-derived
column names come from sorted field paths.

### 3.5 Column & feature naming

`columns` are the per-dimension semantic labels (e.g. `R_joint_state_1 …
R_gripper_state`, `head_qw … head_pz`). They flow verbatim: YAML → HDF5
`columns` attr → LeRobot feature names. Rules:

- Globally unique across streams in a session (a LeRobot dataset has one
  feature namespace).
- Count must match the flattened payload width.
- Pose streams follow the **pose stream layout**: `[qw, qx, qy, qz, px, py, pz]`
  (quaternion w-first — see `docs/adr/0001-pose-stream-layout.md`).
- Name by meaning, not by index (`vr_l_trigger`, not `dim_0`).

## 4. Adding a New Message Type

Needed only when no existing binding covers your `(source, message_type)` pair.
Current bindings:

| Source | Message types |
|--------|---------------|
| teleop | `PoseStamped`, `Joy` |
| robot | `JointState`, `PoseStamped` |
| sensor | `Image` |

To add one (three small pieces, no pipeline changes):

1. **Payload builder** in `core/adapters/registry.py` — a function
   `(msg, stream) -> dict` producing the canonical payload (e.g. `{"imu":
   {...}}`). Reuse helpers from `core/adapters/common.py`
   (`extract_header_timestamp`, `extract_numeric_sequence`, …).
2. **Binding** in `_default_bindings()`:
   `AdapterBinding("<source>", "<pkg>/<Type>", "<name>", "<profile>", builder)`.
   Exactly one binding per `(source, message_type)` — ambiguity is rejected.
3. **Payload profile** in `core/schema/adapter_profiles.py` — field presence,
   numeric/bytes shape, `min_items`. The boundary validator rejects NaN/Inf and
   malformed payloads against it. (You may reuse an existing profile when the
   payload shape matches, as `teleop_joy_buttons` reuses `teleop_buttons`.)

Add a test in `tests/test_adapter_registry.py` mirroring
`test_joy_buttons_binding_reads_axes_with_header_time`.

## 5. Remote & Long-Distance Devices

ROS2/DDS is the **local** data bus — it does not cross firewalls or subnets
reliably. For a device on a remote server (long-distance teleoperation, remote
data collection), pick one of two patterns:

### Pattern A — Relay bridge (message forwarding)

Forward messages ROS2 → JSON → WebSocket → JSON → ROS2 through an SSH tunnel,
republishing on the collector side with the **original `header.stamp`
preserved**. Use this when the remote publisher's stamps must survive (e.g.,
its own sensor capture times).

**Hard requirement: clock synchronization.** A relay preserves stamps, so the
remote machine's clock becomes part of your dataset. Remote publishers on an
unsynced clock corrupt cross-stream alignment invisibly — there is no warning.
Before recording:

- Run `chrony` on every machine, with one LAN host as the time source
  (sub-millisecond on a local network), or PTP (`linuxptp`) for sub-100 µs.
- Verify with `chronyc tracking` / `chronyc sources` — offset well under your
  tightest stream period.

### Pattern B — Interface-at-the-edge (re-stamping)

Run a thin adaptor on the **collector-side** of the slow link that receives the
remote feed (WebSocket/HTTP) and republishes locally as a canonical
header-carrying type, stamping at receipt — exactly how the VR bridge treats
the headset. No clock sync needed (one clock domain), at the cost of including
remote transport latency in the stamp. This is the right default for
long-distance teleoperation where the remote side is a browser or lightweight
device without a managed clock.

### Either pattern — bandwidth & QoS

- Images: publish JPEG (`encoding: jpeg`) — ~200 KB vs ~900 KB per 640×480 frame.
- `best_effort`, `keep_last`, small `depth` (1–5): a dropped frame beats a
  delayed one; latency shows up nowhere in the data because stamps are
  creation-time.
- Do not enable `keep_all` over a lossy link.

## 6. Onboarding Checklist

1. **Publisher**: writes canonical message types; stamps at creation time per
   §2.2; one signal per topic; semantic topic namespace (`/<subsystem>/<signal>`).
2. **Interface YAML** (adaptor-local config): device connection, calibration,
   channel mapping — no pipeline code edits.
3. **Session YAML**: one `streams.` entry per topic with `source`,
   `message_type`, `time_domain: ros_header`, `fields`, `columns`; devices and
   recording-control bindings as needed.
4. **New message type?** → §4 (builder + binding + profile + test).
5. **Remote device?** → §5: choose relay vs edge adaptor; if relaying stamps,
   sync clocks first.
6. **Smoke test** with mocks before hardware:
   `python3.10 tools/dev_mock_ros2_publishers.py --config config/<session>.yaml`
   then `ros2 topic echo --once <topic>` — check the type and that
   `header.stamp` is non-zero and creation-time.
7. **Record a short episode** and validate:
   `python3.10 tools/validate_dataset.py data/episodes/<episode>.h5`.

## 7. Reference

- [Root README](../README.md) — architecture, ROS2 topic contract
- [Teleoperation convention](../src/teleoperation_interface/README.md) — PoseStamped + Joy
- [Robot convention](../src/robot_interface/README.md) — JointState + PoseStamped
- [Sensor convention](../src/sensor_interface/README.md) — Image + CameraInfo
- [Session YAML example](../src/data_collection_service/config/session_vr_ik_robot_button_control.yaml) — the shipped 14-stream profile
- [ADR 0001: pose stream layout](adr/0001-pose-stream-layout.md)
- `CONTEXT.md` — domain vocabulary (episode, stream, collection profile, feature name)
