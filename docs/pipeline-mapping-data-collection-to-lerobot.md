# AIRS Data Collection → LeRobot Pipeline Mapping — Debug Reference

Date: 2026-07-14

Full pipeline:

```
ROS2 topic message (sensor_msgs/Image, geometry_msgs/PoseStamped, sensor_msgs/JointState, std_msgs/Float32MultiArray)
  → adapter payload extraction
  → adapter boundary sample
  → profile validation
  → WriterSample
  → writer stream buffer (memory)
  → HDF5 per-stream group + root attrs
  → LeRobot converter classification
  → LeRobot feature schema
  → nearest-timestamp resampling to canonical VR timeline
  → LeRobotDataset frames
  → MP4 + Parquet dataset on disk
```

---

## HOP 0: ROS2 subscription receives raw message

**dimension space**: wire format / ROS2 message object (object or dict-like)

```
src/data_collection_service/core/runtime/ros2_collection_node.py:152-160
```
```python
def _create_subscriptions(self) -> None:
    for name, stream in self._config.streams:
        msg_cls = _resolve_message_class(stream.message_type)
        if msg_cls is None:
            self.get_logger().warn(f"skipping {name}: cannot resolve {stream.message_type}")
            continue
        qos = _qos_profile_from_stream(stream)
        self.create_subscription(msg_cls, stream.topic, self._make_handler(name), qos)
        self.get_logger().info(f"  {name} → {stream.topic}")
```

**latex**: N/A (structural)

**explanation**: The collection node reads `session_config.yaml`, resolves the declared ROS2 message class for each stream, and creates a ROS2 subscription with the QoS declared in the YAML. The source of truth is the session YAML (`config/session_vr_ik_robot_button_control.yaml`), which names the topic, message type, source family, and time domain for every stream.

### what it does

For every configured stream, it registers a ROS2 subscriber. When a message arrives on that topic, the node calls `_handle_message(stream_name, msg)`. The message is still in its raw ROS2 wire/object representation at this point.

### output example with emphasis

Input (one incoming ROS2 message object):

```python
PoseStamped(
    header=Header(
        stamp=Time(sec=1720934400, nanosec=123456789),
        frame_id="vr_head",
    ),
    pose=Pose(
        position=Point(x=0.1, y=0.2, z=0.3),
        orientation=Quaternion(x=0.0, y=0.1, z=0.2, w=0.9),
    ),
)
```

Key characteristics:

- Message type is exactly what the YAML declared (`geometry_msgs/PoseStamped` for `vr_head_pose`).
- Header stamp may or may not be present depending on `message_type`.
- `frame_id` lives in the header.
- No validation has happened yet; the message may be missing required fields.

---

## HOP 1: Stream message is routed to recording control (device_binding mode)

**dimension space**: ROS2 message object → control action

```
src/data_collection_service/core/runtime/ros2_collection_node.py:167-169
```
```python
def _handle_message(self, stream_name: str, msg: Any) -> None:
    # Route to recording control (device_binding mode) — no-op in other modes
    self._control_router.handle_stream_message(stream_name, msg)
```

**latex**: N/A (structural)

**explanation**: In `device_binding` mode, button streams can trigger `toggle`, `abort`, or `delete` actions. The router inspects the bound button index and threshold from the session YAML and updates the recording state machine. In `service` or `manual_ui` modes this is a no-op. The source of truth is `session_config.yaml` under `session.recording_control.bindings`.

### what it does

If the session is configured to use a stream message (e.g. `vr_left_buttons`) as a physical control input, this HOP evaluates the relevant channel against a threshold and invokes the corresponding action on the state machine. Otherwise the message continues untouched to the adapter.

### output example with emphasis

Binding config from YAML:

```yaml
bindings:
  toggle:
    stream_name: vr_left_buttons
    button_index: 5
    threshold: 0.5
  abort:
    stream_name: vr_left_buttons
    button_index: 4
    threshold: 0.5
  delete:
    stream_name: vr_left_buttons
    button_index: 4
    threshold: 0.5
```

For a button message `data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.8]`:

- channel 5 crosses 0.5 → `toggle` action is triggered.
- channel 4 is below 0.5 → no abort/delete.

Key characteristics:

- Same physical button (ch4) is context-sensitive: abort during recording, delete in pending state.
- Debounce (`toggle_debounce_s`) prevents rapid toggles.
- This HOP only changes the recording lifecycle; it does not write data.

---

## HOP 2: Adapter extracts a canonical payload from the ROS2 message

**dimension space**: ROS2 message object → plain Python dict payload

```
src/data_collection_service/core/adapters/common.py:56-70
```
```python
def extract_pose_payload(message: Any, *, fallback_frame_id: str | None = None) -> dict[str, Any]:
    pose = _get_member(message, "pose") or message
    position = _get_member(pose, "position")
    orientation = _get_member(pose, "orientation") or _get_member(pose, "rotation")
    if position is None or orientation is None:
        raise AdapterError("pose message must define position and orientation")
    payload: dict[str, Any] = {
        "position": {"x": _get_float(position, "x"), "y": _get_float(position, "y"), "z": _get_float(position, "z")},
        "orientation": {"x": _get_float(orientation, "x"), "y": _get_float(orientation, "y"),
                        "z": _get_float(orientation, "z"), "w": _get_float(orientation, "w")},
    }
    frame_id = _extract_frame_id(message) or fallback_frame_id
    if frame_id is not None:
        payload["frame_id"] = frame_id
    return payload
```

**latex**: N/A (structural)

**explanation**: Each configured stream has a `payload_builder` selected by `(source, message_type)`. The builder converts the ROS2 object into a plain Python dict with deterministic keys. `_get_member` works on both object-style and dict-style messages so the code also works with mocked messages in tests. The source of truth is the YAML-declared `message_type` and the adapter binding registry in `core/adapters/registry.py`.

### what it does

For a pose message it extracts `position.{x,y,z}` and `orientation.{x,y,z,w}` into a nested dict. For a joint message it extracts `position` or the `data` array. For an image message it extracts `height`, `width`, `encoding`, `is_bigendian`, `step`, and `data` bytes.

### output example with emphasis

Input: the `PoseStamped` from HOP 0.

Output:

```python
{
    "position": {"x": 0.1, "y": 0.2, "z": 0.3},
    "orientation": {"x": 0.0, "y": 0.1, "z": 0.2, "w": 0.9},
    "frame_id": "vr_head",
}
```

Key characteristics:

- Field order is **not** determined here; flattening happens later in `_collect_scalars`.
- Missing required fields raise `AdapterError`.
- `orientation` uses the `w` component (quaternion scalar part) as extracted from the ROS message.

---

## HOP 3: Adapter builds the canonical boundary sample

**dimension space**: plain payload + received timestamp → `AdapterBoundarySample`

```
src/data_collection_service/core/adapters/common.py:20-36
```
```python
def build_boundary_sample(
    stream: StreamConfig, *, payload: Mapping[str, Any],
    received_at: datetime, source_timestamp: datetime | None,
) -> AdapterBoundarySample:
    if stream.time_domain == TimeDomain.ROS_HEADER and source_timestamp is None:
        raise AdapterError(f"{stream.name} requires header timestamp for ros_header")
    timestamp = source_timestamp if stream.time_domain == TimeDomain.ROS_HEADER else received_at
    try:
        return AdapterBoundarySample(
            stream=CanonicalStreamIdentity(stream.name, stream.source),
            timestamp=timestamp,
            payload=dict(payload),
            time_domain=stream.time_domain.value,
            source_timestamp=source_timestamp,
        )
    except ContractError as exc:
        raise AdapterError(str(exc)) from exc
```

**latex**: N/A (structural)

**explanation**: The boundary sample is the canonical envelope every ingress path must cross. It carries the stream identity, the chosen timestamp (`ros_header` vs `ros_receive`), and the payload. The source of truth is the stream's `time_domain` declared in the session YAML.

### what it does

Selects the timestamp according to the stream's `time_domain`. For `ros_header` streams (most pose and joint streams) it uses the header stamp; for `ros_receive` streams (button streams) it uses the local receive time. Then it wraps payload, identity, and timestamp into a frozen dataclass.

### output example with emphasis

For `vr_head_pose` with `time_domain: ros_header`:

```python
AdapterBoundarySample(
    stream=CanonicalStreamIdentity(stream_name="vr_head_pose", source_family="teleop"),
    timestamp=datetime(2024, 07, 14, 0, 0, 0, 123456, tzinfo=timezone.utc),
    payload={
        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
        "orientation": {"x": 0.0, "y": 0.1, "z": 0.2, "w": 0.9},
        "frame_id": "vr_head",
    },
    time_domain="ros_header",
    source_timestamp=datetime(2024, 07, 14, 0, 0, 0, 123456, tzinfo=timezone.utc),
)
```

Key characteristics:

- `timestamp == source_timestamp` when `time_domain == "ros_header"`; otherwise `source_timestamp` may be `None`.
- `source_family` comes from the YAML `source` field (`teleop`, `robot`, or `sensor`).
- Any `ContractError` (e.g. empty payload) is re-raised as `AdapterError`.

---

## HOP 4: Profile validation checks payload structure

**dimension space**: `AdapterBoundarySample` → validated `AdapterBoundarySample`

```
src/data_collection_service/core/schema/adapter_profiles.py:74-88
```
```python
def validate_payload(self, payload: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(payload, Mapping) or not payload:
        raise AdapterPayloadValidationError("payload", "payload must be a non-empty mapping")
    for rule in self.field_rules:
        _validate_rule(dict(payload), rule)
    return dict(payload)


def validate_sample(self, sample: AdapterBoundarySample) -> AdapterBoundarySample:
    if sample.source_family != self.source_family:
        raise AdapterPayloadValidationError(
            "stream.source_family",
            f"expected {self.source_family!r}, got {sample.source_family!r}",
        )
    self.validate_payload(sample.payload)
    return sample
```

**latex**: N/A (structural)

**explanation**: The adapter's payload profile (`teleop_pose`, `teleop_buttons`, `robot_joint_positions`, etc.) declares field rules: required fields, numeric ranges, and sequence lengths. The validator walks dot-separated paths in the payload and enforces those rules. The source of truth is the profile registry in `core/schema/adapter_profiles.py`.

### what it does

For a pose profile it checks that `position.{x,y,z}` and `orientation.{x,y,z,w}` exist and are numeric. For an image profile it checks `height`, `width`, `encoding`, `is_bigendian`, `step`, and that `data` is a non-empty byte sequence. Failures raise `AdapterPayloadValidationError`, which the node logs as an invalid message.

### output example with emphasis

Pose profile rules:

```python
(
    AdapterPayloadFieldRule(path="position.x", numeric=True),
    AdapterPayloadFieldRule(path="position.y", numeric=True),
    AdapterPayloadFieldRule(path="position.z", numeric=True),
    AdapterPayloadFieldRule(path="orientation.x", numeric=True),
    AdapterPayloadFieldRule(path="orientation.y", numeric=True),
    AdapterPayloadFieldRule(path="orientation.z", numeric=True),
    AdapterPayloadFieldRule(path="orientation.w", numeric=True),
)
```

For the sample from HOP 3, validation succeeds and returns the same sample unchanged.

Key characteristics:

- Validation is purely structural; it does not check semantic ranges (e.g. quaternion norm).
- `bytes` payloads bypass per-element `min_items`/`max_items` iteration for efficiency.
- Failed samples increment the stream tracker's `messages_invalid` counter.

---

## HOP 5: Adapter converts validated payload to WriterSample

**dimension space**: validated payload → `WriterSample` (vector tuple or image bytes + timestamp)

```
src/data_collection_service/core/adapters/registry.py:200-224
```
```python
def _writer_sample_from_payload(
    stream_name: str, payload: dict[str, Any], ts_ns: int,
) -> WriterSample:
    data_bytes = payload.get("data")
    if isinstance(data_bytes, (bytes, bytearray)):
        return WriterSample(
            stream_name=stream_name, timestamp_ns=ts_ns,
            image_data=bytes(data_bytes),
            width=_int_or_none(payload.get("width")),
            height=_int_or_none(payload.get("height")),
        )

    values = _collect_scalars(payload)
    return WriterSample(
        stream_name=stream_name, timestamp_ns=ts_ns,
        values=tuple(values),
    )
```

**latex**: N/A (structural)

**explanation**: This is the generic flattening step. If the payload contains a `data` key with bytes, it is treated as an image. Otherwise every leaf numeric value is collected depth-first in **sorted key order** into a flat tuple. This sorted-key order is the source of truth for how 7-D pose vectors become `[qw, qx, qy, qz, px, py, pz]` and is documented in ADR 0001.

### what it does

For pose payloads it flattens the nested dict into a 7-element tuple. For joint payloads it flattens the joint position list. For image payloads it keeps the raw byte buffer plus `width`/`height` from the message metadata.

### output example with emphasis

Input payload from HOP 3:

```python
{
    "position": {"x": 0.1, "y": 0.2, "z": 0.3},
    "orientation": {"x": 0.0, "y": 0.1, "z": 0.2, "w": 0.9},
    "frame_id": "vr_head",
}
```

Output `WriterSample`:

```python
WriterSample(
    stream_name="vr_head_pose",
    timestamp_ns=1720934400123456789,
    values=(0.0, 0.1, 0.2, 0.9, 0.1, 0.2, 0.3),
    #      ^  orient.x
    #         ^  orient.y
    #            ^  orient.z
    #               ^  orient.w  ← w is fourth because sorted-key order puts orientation before position
    #                  ^  pos.x
    #                     ^  pos.y
    #                        ^  pos.z
    image_data=None,
    width=None,
    height=None,
)
```

Key characteristics:

- `orientation` keys are sorted before `position` keys, so quaternion comes first.
- Within `orientation`, keys are sorted alphabetically: `w, x, y, z`. Hence `w` is the **first** element, not the last.
- `frame_id` is a string and is skipped by `_collect_scalars` because it is not numeric.
- The timestamp is converted to integer nanoseconds since epoch.

> **FIX (2026-07-14)**: The original LeRobot converter assumed pose vectors were `[x, y, z, qx, qy, qz, qw]`. The recorded data was actually `[qw, qx, qy, qz, px, py, px]` because of sorted-key flattening. The fix was to add explicit `columns` in the YAML matching the true recorded order, rather than reordering the data. See ADR 0001.

---

## HOP 6: Collection node appends WriterSample to writer stream buffer

**dimension space**: `WriterSample` → in-memory batch buffer

```
src/data_collection_service/core/runtime/ros2_collection_node.py:174-186
```
```python
now = datetime.now(timezone.utc)
sample = adapter.adapt(msg, received_at=now)
self._stream_tracker.record_valid(stream_name, sample.timestamp_ns)
if not self._state_machine.is_recording:
    return
if sample.image_data is not None:
    self._writer.append_image(
        stream_name, sample.image_data, sample.timestamp_ns,
        width=sample.width, height=sample.height,
    )
elif sample.values is not None:
    self._writer.append_vector(stream_name, sample.values, sample.timestamp_ns)
```

**latex**: N/A (structural)

**explanation**: The node is a thin shell: it adapts the message, records the stream metric, and if recording is active, appends the sample to the writer. The writer's hot path only buffers in memory; it does not touch HDF5 on every message. The source of truth for whether recording is active is the `RecordingStateMachine`.

### what it does

If the state machine is in `recording`, the sample is handed to `AirsHdf5Writer.append_vector` or `append_image`. If not recording, the sample is dropped after metrics are recorded. The writer does not see non-recording samples.

### output example with emphasis

For a vector sample:

```python
writer.append_vector("vr_head_pose", (0.0, 0.1, 0.2, 0.9, 0.1, 0.2, 0.3), 1720934400123456789)
```

For an image sample:

```python
writer.append_image(
    "camera_head",
    b"\xff\xd8\xff\xe0...",  # JPEG bytes
    1720934400123456789,
    width=640,
    height=480,
)
```

Key characteristics:

- `stream_tracker` receives the timestamp even when not recording, so observed rate estimation works across episodes.
- The node never inspects the payload contents; it only checks `image_data` vs `values`.
- Appends are cheap memory operations; no HDF5 I/O happens here.

---

## HOP 7: Writer registers stream schema when episode opens

**dimension space**: `StreamConfig` + adapter → HDF5 group attributes

```
src/data_collection_service/core/storage/airs_hdf5_writer.py:143-186
```
```python
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
```

**latex**: N/A (structural)

**explanation**: When the state machine starts a recording, `_start` opens a new HDF5 file and every adapter registers its stream with the writer. For vector streams this creates the group, writes the `type` and `columns` attributes, and creates empty `data` and `timestamps` datasets. For image streams it creates a variable-length uint8 `data` dataset. The source of truth for column names is the YAML `columns` key or auto-derived field paths; for image dimensions it is the message metadata (see HOP 8).

### what it does

Creates the HDF5 group structure for one stream. Vector streams get a 2-D `data` dataset (frames × dims) and a 1-D `timestamps` dataset. Image streams get a 1-D variable-length `data` dataset and a 1-D `timestamps` dataset.

### output example with emphasis

After registration, the HDF5 group for `vr_head_pose` looks like this (before any frames):

```
/vr_head_pose
  attrs:
    type = "vector"
    columns = '["head_qw", "head_qx", "head_qy", "head_qz", "head_px", "head_py", "head_pz"]'
  data: shape (0, 7), dtype float32, maxshape (None, 7)
  timestamps: shape (0,), dtype uint64, maxshape (None,)
```

Key characteristics:

- `columns` is stored as a JSON-encoded string attribute.
- `dims` may be 0 for variable-length sequence streams; the data dataset is then created lazily on first flush.
- Dataset chunks are sized to the batch size (`_VECTOR_BATCH = 50`, `_IMAGE_BATCH = 20`).

---

## HOP 8: Writer buffers frames and flushes in batches

**dimension space**: single frame → batch buffer → resized HDF5 dataset

```
src/data_collection_service/core/storage/airs_hdf5_writer.py:240-289
```
```python
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
```

**latex**: N/A (structural)

**explanation**: The writer keeps at most one batch in memory. When the batch reaches `_VECTOR_BATCH` (50) frames, it resizes the HDF5 dataset once and writes the entire batch contiguously. This amortizes resize cost and caps memory usage. The source of truth for batch sizes is the writer constants `_VECTOR_BATCH` and `_IMAGE_BATCH`.

### what it does

Appends the vector to a Python list, validates its dimension against the declared columns, tracks first/last timestamps, and flushes when the batch is full. Image buffers do the same with raw JPEG bytes.

### output example with emphasis

After 50 frames have been appended to `vr_head_pose`:

```python
self._data_buf  # list of 50 np.ndarray float32, each shape (7,)
self._ts_buf    # list of 50 np.uint64
```

After `_flush_batch()`:

```
/vr_head_pose/data: shape (50, 7), dtype float32
/vr_head_pose/timestamps: shape (50,), dtype uint64
```

Key characteristics:

- Memory is bounded by one batch regardless of episode length.
- Dimension mismatch raises `AirsHdf5WriterError` immediately.
- For sequence streams with `dims=0`, the first sample sets the dimension.
- `_n_columns` is derived from the JSON `columns` attribute at buffer construction.

---

## HOP 9: Writer computes per-stream sample_rate at episode close

**dimension space**: buffered timestamps → Hz

```
src/data_collection_service/core/storage/airs_hdf5_writer.py:372-384
```
```python
def _measured_rate(n_frames: int, first_ts_ns: int | None,
                   last_ts_ns: int | None) -> float:
    if n_frames < 2 or first_ts_ns is None or last_ts_ns is None:
        return 0.0
    duration_s = (last_ts_ns - first_ts_ns) / 1e9
    if duration_s <= 0.0:
        return 0.0
    return (n_frames - 1) / duration_s
```

**latex**:

$$
\text{sample_rate} = \frac{N - 1}{(t_{last} - t_{first}) / 10^9} \; \text{Hz}
$$

where $N$ is the total frame count and $t_{last}, t_{first}$ are nanosecond timestamps.

**explanation**: When `close_episode` flushes remaining buffers, each stream stores its measured mean rate. The formula counts intervals, not frames, which is why the numerator is $N - 1$. The source of truth is the stream's own timestamps; there is no longer a single global sample rate.

### what it does

Computes the average frame rate from the first and last timestamp of the stream. Returns 0.0 if there are fewer than 2 frames or if timestamps are invalid.

### output example with emphasis

For `vr_head_pose` with 620 frames and timestamps spanning 10 seconds:

```
/vr_head_pose/attrs/sample_rate = 61.9   # approximately 62 Hz
```

Key characteristics:

- Rate is mean, not instantaneous.
- 0.0 means "uncomputable" (used for very short or single-frame streams).
- The converter later compares this to `--fps` and warns if the deviation exceeds 20%.

---

## HOP 10: Writer stamps episode-level root attributes

**dimension space**: episode outcome + task metadata → HDF5 root attrs

```
src/data_collection_service/core/storage/airs_hdf5_writer.py:84-112
```
```python
def close_episode(self, *, task_completed: bool,
                  termination_reason: str) -> str:
    if self._file is None:
        raise AirsHdf5WriterError("no episode is open")
    total_frames = 0
    for buf in self._streams.values():
        buf.flush_remaining()
        total_frames = max(total_frames, buf.frame_count)
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
```

**latex**: N/A (structural)

**explanation**: At close, the writer stamps root attributes that describe the episode. The two-axis outcome model (`task_completed` and `recording_valid`) is documented in ADR 0002. Task metadata comes from the UI via `set_task`. The source of truth for the attribute schema is the writer implementation and ADR 0002.

### what it does

Flushes all buffers, writes the maximum frame count across streams, records whether the task was completed, marks the recording as valid, stores the termination reason, and writes task metadata if available.

### output example with emphasis

Root attributes of a completed episode:

```yaml
description: "vr_ik_robot_button_control"
robot_type: "robot_sensor_teleop"
series_number: "example_operator"
frames: 64
task_completed: True
recording_valid: True
termination_reason: "goal_reached"
task_prompt: "Pick up the red cube and place it in the bin"
task_id: "pick-red-cube-v1"
task_structure: "rigid_cube_on_table"
deformable_objects: False
```

Key characteristics:

- `recording_valid` is `True` on fresh close; it is later set to `False` by `stamp_recording_invalid` when the operator deletes the episode.
- `task_completed` is orthogonal to `recording_valid`: a failed demo can still be valid data.
- `frames` is the maximum frame count across all streams, not a per-stream value.

---

## HOP 11: Operator deletion marks recording invalid and moves to trash

**dimension space**: closed episode file → invalidated + relocated file

```
src/data_collection_service/core/storage/airs_hdf5_writer.py:114-139
```
```python
@staticmethod
def stamp_recording_invalid(episode_path: str | Path) -> None:
    with h5py.File(episode_path, "r+") as f:
        f.attrs["recording_valid"] = False
        f.attrs["termination_reason"] = "recording_invalid"


@staticmethod
def move_to_trash(episode_path: str | Path) -> str:
    src = Path(episode_path)
    if not src.exists():
        return str(src)
    trash_dir = src.parent / ".trash"
    trash_dir.mkdir(parents=True, exist_ok=True)
    dst = trash_dir / src.name
    shutil.move(str(src), str(dst))
    return str(dst)
```

**latex**: N/A (structural)

**explanation**: When the operator deletes the most recent episode (via the context-sensitive B button in pending mode or a service call), the file is first stamped `recording_valid=False` with reason `recording_invalid`, then moved to a `.trash/` subdirectory. The original `task_completed` value is preserved so downstream tools can still distinguish "deleted success" from "deleted failure" if needed.

### what it does

Opens the HDF5 file in read/write mode, updates the two root attributes, closes it, and moves the file into `.trash/` under its parent directory.

### output example with emphasis

Before delete:

```
episode-T5-20260714T090000000000Z.h5
  attrs/task_completed = True
  attrs/recording_valid = True
  attrs/termination_reason = "goal_reached"
```

After delete:

```
.trash/episode-T5-20260714T090000000000Z.h5
  attrs/task_completed = True        ← preserved
  attrs/recording_valid = False      ← changed
  attrs/termination_reason = "recording_invalid"
```

Key characteristics:

- The file is modified before moving, so the invalid state is durable even if the move is interrupted.
- `.trash/` is a sibling of the original file's parent directory.
- The converter skips `.trash/` paths entirely.

---

## HOP 12: LeRobot converter loads HDF5 and classifies streams

**dimension space**: HDF5 file → classified stream lists

```
src/data_collection_service/tools/convert_h5_to_lerobot.py:41-96
```
```python
with h5py.File(h5_path, "r") as f:
    if "task_prompt" not in f.attrs:
        raise ValueError(...)
    if "recording_valid" in f.attrs and not bool(f.attrs["recording_valid"]):
        raise ValueError(...)

    vr_vectors = []
    ik_vectors = []
    arm_vectors = []
    image_streams = []
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
```

**latex**: N/A (structural)

**explanation**: The converter first enforces the new-vintage guardrails: `task_prompt` must exist and `recording_valid` must be True. Then it classifies streams by reading the `type` attribute and the stream name prefix. The source of truth is the HDF5 schema produced by the writer and the naming convention in the session YAML.

### what it does

Reads the HDF5 file, validates the root attributes, and partitions streams into four categories used for feature construction and timeline selection.

### output example with emphasis

For a typical episode:

```python
vr_vectors = [
    "vr_head_pose", "vr_left_pose", "vr_right_pose",
    "vr_left_buttons", "vr_right_buttons",
]
ik_vectors = [
    "ik_left_joint_commands", "ik_right_joint_commands",
    "ik_left_target_pose", "ik_right_target_pose",
]
arm_vectors = [
    "arm_left_joint_state", "arm_right_joint_state",
]
image_streams = [
    "camera_head", "camera_left_wrist", "camera_right_wrist",
]
```

Key characteristics:

- Legacy episodes (no `task_prompt` or `dim_N` columns) are rejected here.
- Trashed episodes are rejected by `recording_valid=False`.
- Classification depends on stream names matching the `vr_`, `ik_`, `arm_` prefixes.

---

## HOP 13: LeRobot converter selects canonical timeline

**dimension space**: multi-rate stream timestamps → single canonical timestamp vector

```
src/data_collection_service/tools/convert_h5_to_lerobot.py:78-84
```
```python
canonical = "vr_head_pose"
if canonical not in f:
    canonical = vr_vectors[0]
canon_ts = f[canonical]["timestamps"][:]
n_frames = len(canon_ts)
```

**latex**: N/A (structural)

**explanation**: LeRobot expects a single uniform frame rate and one frame per timestep. The converter treats the fastest VR stream as the canonical timeline because it has the densest sampling and is the teleop source. If `vr_head_pose` is absent, it falls back to the first available VR vector stream.

### what it does

Loads the timestamps of the canonical stream and uses its length as the output frame count. Every other stream will be resampled to these timestamps.

### output example with emphasis

```python
canonical = "vr_head_pose"
canon_ts = array([1720934400000000000, 1720934400161290322, 1720934400322580645, ...])  # 64 entries
n_frames = 64
```

Key characteristics:

- Timestamps are nanoseconds since epoch stored as `uint64`.
- Canonical stream must be a `vr_*` vector; otherwise conversion fails.
- Output FPS is decoupled from the canonical stream's actual rate; the caller supplies `--fps`.

---

## HOP 14: LeRobot converter validates semantic columns and detects duplicates

**dimension space**: per-stream `columns` attr → ordered global feature list

```
src/data_collection_service/tools/convert_h5_to_lerobot.py:98-124
```
```python
all_vectors = vr_vectors + ik_vectors + arm_vectors
seen: set[str] = set()
duplicates: set[str] = set()
state_names: list[str] = []
for name in all_vectors:
    cols = stream_cache[name]["columns"]
    if not cols:
        raise ValueError(...)
    for c in cols:
        if c.startswith("dim_"):
            raise ValueError(...)
        if c in seen:
            duplicates.add(c)
        seen.add(c)
        state_names.append(c)
if duplicates:
    raise ValueError(...)
```

**latex**: N/A (structural)

**explanation**: The converter concatenates all vector streams into one `observation.state` feature vector. It checks that every stream has semantic column names, rejects legacy `dim_N` names, and errors if any name collides across streams. The source of truth is the `columns` JSON attribute written by the writer.

### what it does

Collects all column names in stream order (VR, then IK, then arm) and verifies global uniqueness. The final `state_names` list defines both the `observation.state` and `action` feature names in LeRobot.

### output example with emphasis

```python
state_names = [
    "head_qw", "head_qx", "head_qy", "head_qz", "head_px", "head_py", "head_pz",
    "vr_l_trigger", "vr_l_grip", ..., "vr_r_button_5",
    "L_joint_cmd_1", ..., "R_gripper_cmd",
    "ik_l_qw", ..., "ik_r_pz",
    "L_joint_state_1", ..., "R_gripper_state",
]
total_state_dim = 79
```

Key characteristics:

- Order matters: it matches the concatenation order of stream data.
- Duplicate feature names are a hard error because LeRobot feature names must be unique.
- `dim_0` etc. trigger a hard error because legacy episodes are retired.

---

## HOP 15: LeRobot converter resamples all streams to canonical timestamps

**dimension space**: per-stream native timestamps → nearest timestamp on canonical timeline

```
src/data_collection_service/tools/convert_h5_to_lerobot.py:21-30
```
```python
def _nearest_idx(timestamps: np.ndarray, query_ts: int) -> int:
    idx = np.searchsorted(timestamps, query_ts, side="left")
    idx = max(0, min(idx, len(timestamps) - 1))
    if idx > 0:
        dist_left = abs(int(timestamps[idx - 1]) - query_ts)
        dist_curr = abs(int(timestamps[idx]) - query_ts)
        if dist_left < dist_curr:
            idx -= 1
    return idx
```

```
src/data_collection_service/tools/convert_h5_to_lerobot.py:184-210
```
```python
for i in range(n_frames):
    query_ts = int(canon_ts[i])
    frame = {}
    state_parts = []
    for name in all_vectors:
        sc = stream_cache[name]
        idx = _nearest_idx(sc["ts"], query_ts)
        vals = sc["data"][idx]
        ...
    for cam in image_streams:
        sc = stream_cache[cam]
        idx = _nearest_idx(sc["ts"], query_ts)
        jpeg_bytes = bytes(sc["data"][idx])
        ...
```

**latex**:

For each canonical timestamp $t_c$ and per-stream timestamps $T_s$:

$$
i^* = \underset{i}{\arg\min} \; |T_s[i] - t_c|
$$

**explanation**: Because streams run at independent rates with independent clocks, the converter resamples each stream to the canonical VR timeline using nearest-neighbor lookup. The source of truth is the per-stream `timestamps` datasets; no interpolation is performed.

### what it does

For every canonical frame timestamp it finds the closest sample in each stream and uses that sample's data. Vectors are concatenated into `observation.state` and `action`; images are decoded from JPEG to RGB.

### output example with emphasis

Canonical frame 5 has `query_ts = 1720934400806451612`.

For `arm_left_joint_state` (30 Hz, slower than VR):

```python
arm_ts = array([1720934400000000000, 1720934400333333333, 1720934400666666666, ...])
idx = _nearest_idx(arm_ts, query_ts)  # returns 2
arm_values = array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.0])  # shape (8,)
```

Key characteristics:

- Nearest-neighbor can repeat or skip slower-stream samples.
- The `head_qw` etc. values from `vr_head_pose` are used directly as canonical frames; they are not resampled against themselves.
- Image decoding uses OpenCV `imdecode` + `cvtColor(BGR→RGB)`.

---

## HOP 16: LeRobot converter builds LeRobotDataset features and writes frames

**dimension space**: AIRS streams → LeRobot v3 feature namespace

```
src/data_collection_service/tools/convert_h5_to_lerobot.py:154-218
```
```python
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

dataset = LeRobotDataset.create(
    repo_id=repo_id, root=output_path, fps=fps,
    robot_type=robot_type, features=features,
    vcodec=vcodec,
)
```

**latex**: N/A (structural)

**explanation**: The converter maps AIRS vector streams to LeRobot's `observation.state` and `action` features, and image streams to `observation.images.<camera>`. In this teleop pipeline `action` is set equal to `observation.state` because the recorded state is the command. The source of truth is the LeRobot v3 dataset schema.

### what it does

Creates a LeRobotDataset with the feature schema, then iterates over canonical frames, builds each frame dict, and calls `dataset.add_frame`. After all frames it calls `save_episode()` and `finalize()`.

### output example with emphasis

One frame dict passed to `dataset.add_frame`:

```python
{
    "observation.state": array([0.0, 0.1, 0.2, 0.9, 0.1, 0.2, 0.3,  # vr_head_pose
                                0.0, 0.0, ..., 0.0,                  # vr_left_buttons
                                ...], dtype=float32),                # total shape (79,)
    "action":          <same array>,                                  # teleop: action = state
    "task":            "Pick up the red cube and place it in the bin",
    "observation.images.camera_head": array([[[1,2,3], ...]], dtype=uint8),  # shape (480, 640, 3)
    "observation.images.camera_left_wrist": array(...),  # shape (480, 848, 3)
    "observation.images.camera_right_wrist": array(...), # shape (480, 848, 3)
}
```

Key characteristics:

- `observation.state` and `action` share the same concatenated vector.
- `task` is the verbatim `task_prompt` root attribute.
- Image arrays are decoded RGB, not raw JPEG.
- LeRobot internally encodes images to H.264 MP4 and writes state/action to Parquet.

---

## HOP 17: LeRobot dataset is finalized on disk

**dimension space**: in-memory dataset → `meta/`, `data/`, `videos/` directories

```
src/data_collection_service/tools/convert_h5_to_lerobot.py:217-219
```
```python
dataset.save_episode()
dataset.finalize()
```

**latex**: N/A (structural)

**explanation**: `LeRobotDataset` handles the final on-disk layout: `meta/info.json`, `meta/stats.json`, `meta/tasks.parquet`, `meta/episodes/...`, `data/...parquet`, and `videos/<video_key>/...mp4`. The converter then runs a lightweight validation that checks these files exist.

### what it does

Persists the dataset in LeRobot v3 format and produces a validation report.

### output example with emphasis

Output directory structure:

```
/tmp/lerobot_dataset/
├── meta/
│   ├── info.json
│   ├── stats.json
│   ├── tasks.parquet
│   └── episodes/
│       └── chunk-000/
│           └── file-000.parquet
├── data/
│   └── chunk-000/
│       └── file-000.parquet
└── videos/
    ├── observation.images.camera_head/
    │   └── chunk-000/file-000.mp4
    ├── observation.images.camera_left_wrist/
    │   └── chunk-000/file-000.mp4
    └── observation.images.camera_right_wrist/
        └── chunk-000/file-000.mp4
```

Key characteristics:

- `tasks.parquet` contains one row per unique `task_prompt`.
- `info.json` records `fps`, `robot_type`, `total_episodes`, `total_frames`, and the feature schema.
- MP4 videos are encoded with the codec passed via `--vcodec` (default `h264`).

---

## Summary: AIRS Data Collection → LeRobot

### Main pipeline (controls training data format):

```
ROS2 topic message (raw object/dict)
    |
    v HOP 0: subscribe by YAML (topic, msg_type, QoS)
    |         [ROS2 wire format]
    |
    v HOP 1: recording-control routing (device_binding mode)
    |         [ROS2 wire format -> lifecycle action]
    |
    v HOP 2: adapter payload extraction
    |         [plain Python dict payload]
    |
    v HOP 3: build AdapterBoundarySample
    |         [canonical envelope with chosen timestamp]
    |
    v HOP 4: profile validation
    |         [validated canonical envelope]
    |
    v HOP 5: flatten to WriterSample
    |         [tuple[float,...] or bytes + timestamp_ns]
    |
    v HOP 6: node appends to writer buffer (only while recording)
    |         [in-memory batch buffer]
    |
    v HOP 7: writer registers HDF5 stream schema on episode start
    |         [HDF5 group + attrs + empty datasets]
    |
    v HOP 8: writer buffers and flushes frames in batches
    |         [resized HDF5 datasets]
    |
    v HOP 9: writer computes per-stream sample_rate
    |         [Hz value in stream attrs]
    |
    v HOP 10: writer stamps episode root attrs
    |         [HDF5 root attrs: task_prompt, task_completed, recording_valid, ...]
    |
    v HOP 11: deletion stamps invalid and moves to .trash/
    |         [invalidated HDF5 file in .trash/]
    |
    +--> [Branch: file stays in task folder] -----------------------------+
    |                                                                    |
    v HOP 12: converter loads HDF5 and classifies streams                |
    |         [classified stream lists]                                  |
    |                                                                    |
    v HOP 13: converter selects canonical VR timeline                    |
    |         [canonical uint64 timestamp vector]                        |
    |                                                                    |
    v HOP 14: converter validates semantic columns                       |
    |         [ordered unique state_names]                               |
    |                                                                    |
    v HOP 15: converter resamples all streams to canonical timestamps    |
    |         [nearest-neighbor matched frames]                          |
    |                                                                    |
    v HOP 16: converter builds LeRobot features and writes frames        |
    |         [LeRobotDataset in memory]                                 |
    |                                                                    |
    v HOP 17: LeRobot finalizes dataset to disk                          |
              [meta/ + data/ + videos/ on disk]
```

### Key Source-of-Truth Documents:

1. **`config/session_vr_ik_robot_button_control.yaml`**: Declares streams, topics, message types, source families, time domains, QoS, recording-control bindings, and semantic `columns`.
2. **ADR 0001 (`docs/adr/0001-pose-stream-layout.md`)**: Defines the recorded pose layout `[qw, qx, qy, qz, px, py, pz]` and the relabel-only decision.
3. **ADR 0002 (`docs/adr/0002-episode-outcome-model.md`)**: Defines the two-axis outcome model (`task_completed`, `recording_valid`) and context-sensitive B button behavior.
4. **ADR 0003 (`docs/adr/0003-delete-threaded-writer-twin.md`)**: Documents the threaded-writer deletion decision and the synthetic benchmark results.
5. **`core/schema/adapter_profiles.py`**: Defines payload profiles used in HOP 4 validation.
6. **LeRobot v3 dataset schema**: Defines `observation.state`, `action`, `observation.images.*`, and `task` feature namespaces used in HOPs 16–17.
