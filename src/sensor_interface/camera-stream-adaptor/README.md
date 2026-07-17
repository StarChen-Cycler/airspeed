# Camera Stream

RealSense camera Image + CameraInfo publishing to ROS2. Streams at the camera's
native rate via a background thread — no artificial frame rate cap.

Deploy independently of arm controllers. Add cameras and stream types by editing
`config/camera.yaml` only — zero code changes.

## Published ROS2 Topics

Topics are derived from the config. Default three-camera setup:

| Topic | Type | Content |
|-------|------|---------|
| `/camera/right_wrist/image_raw` | `sensor_msgs/Image` | Raw rgb8 by default (JPEG with `--jpeg`), native rate (~30 Hz single / ~15 Hz multi-stream) |
| `/camera/right_wrist/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics (latched, TRANSIENT_LOCAL) |
| `/camera/head/image_raw` | `sensor_msgs/Image` | Raw rgb8 (JPEG with `--jpeg`) |
| `/camera/head/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics (latched) |
| `/camera/left_wrist/image_raw` | `sensor_msgs/Image` | Raw rgb8 (JPEG with `--jpeg`) |
| `/camera/left_wrist/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics (latched) |

CameraInfo is published once at startup with TRANSIENT_LOCAL durability (latched) —
late subscribers receive intrinsics without waiting for a republish.

`Image.header.stamp` is captured immediately after each frame read returns, before
color conversion and JPEG encoding — recorded timestamps reflect frame return
(≈ exposure time), not publish or post-encode time.

## Zenoh side channel (raw frames)

Raw frames (`encoding != "jpeg"`) do **not** travel on ROS2 image topics: rclpy
costs ~130 ms per ~1 MB message, so raw pixels are published over a zenoh side
channel at ~0.9 ms (measured). JPEG frames and CameraInfo stay on ROS2.

- Listens on `tcp/0.0.0.0:7447`, one publisher per stream key (the ROS topic
  path without the leading slash, e.g. `camera/head/image_raw`).
- Envelope: `struct <QQHHB (ts_ns, seq, width, height, enc_len)` + encoding
  bytes + frame bytes. `ts_ns` is the same creation-time stamp (frame return).
- Requires `pip install eclipse-zenoh`.
- The collector consumes these streams with `transport: zenoh` in the session
  YAML (see examples below).
- `--no-side-channel` restores the old ROS2 raw path (slow; A/B debugging only).

## Quick Start

```bash
cd camera-stream-adaptor
source /opt/ros/humble/setup.bash
python3 camera_publisher.py

# Compress at source instead of raw pixels (off by default)
python3 camera_publisher.py --jpeg

# Custom config path
python3 camera_publisher.py --config-dir /path/to/config
```

## Configuration

All in `config/camera.yaml`. Per-camera stream configuration — each camera
declares which streams (color, depth, infra) to publish. Streams not listed
under a camera are not opened for that camera.

### Default Config

```yaml
cameras:
  right_wrist:
    serial: "auto"
    topic: "/camera/right_wrist"
    frame_id: "camera_right_wrist_optical_frame"
    streams:
      color:
        enabled: true
        width: 640
        height: 480
        fps: 30
        encoding: "rgb8"
        jpeg_quality: 70

  head:
    serial: "auto"
    topic: "/camera/head"
    frame_id: "camera_head_optical_frame"
    streams:
      color:
        enabled: true
        width: 640
        height: 480
        fps: 30
        encoding: "rgb8"
        jpeg_quality: 70

  left_wrist:
    serial: "auto"
    topic: "/camera/left_wrist"
    frame_id: "camera_left_wrist_optical_frame"
    streams:
      color:
        enabled: true
        width: 640
        height: 480
        fps: 30
        encoding: "rgb8"
        jpeg_quality: 70

max_cameras: 3

intrinsics:
  right_wrist: { ... }
  head: { ... }
  left_wrist: { ... }
```

### Adding a Camera

Add an entry under `cameras:` and intrinsics under `intrinsics:`:

```yaml
cameras:
  my_new_camera:
    serial: "auto"                 # "auto" or specific serial number
    topic: "/camera/my_new_camera"
    frame_id: "my_camera_optical_frame"
    streams:
      color:
        enabled: true
        width: 1280
        height: 720
        fps: 30
        encoding: "rgb8"
        jpeg_quality: 80

intrinsics:
  my_new_camera:
    width: 1280
    height: 720
    fx: 920.0
    fy: 920.0
    cx: 640.0
    cy: 360.0
    distortion_model: "plumb_bob"
    distortion_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
```

### Stream Types

| Stream | Supported | Notes |
|--------|-----------|-------|
| `color` | Yes (via lerobot SDK) | RGB frame, raw rgb8 by default (JPEG with `--jpeg`) |
| `depth` | Not yet | Requires pyrealsense2 direct access |
| `infra_left` | Not yet | Requires pyrealsense2 direct access |
| `infra_right` | Not yet | Requires pyrealsense2 direct access |

If a stream type is configured but the backend doesn't support it, a `[WARN]`
is printed at startup and the stream is skipped — the node does not crash.

### Camera Discovery

Cameras are matched to config entries by **discovery order** — the first
discovered camera gets the first config entry, the second gets the second, etc.
Use `serial: "<number>"` to pin a specific physical camera to a config entry.

If a config entry can't be matched to any discovered camera, a `[WARN]` is
printed and that entry is skipped.

### Rate

Frames stream at the camera's native rate — no `create_timer` cap. A single
color stream runs at ~30 Hz. With multiple streams enabled per camera, the
composite rate drops to ~15 Hz (USB 3.2 bandwidth limit). The console prints
the actual achieved rate every 10 seconds.

## Session YAML (Data Collection Service)

```yaml
streams:
  - name: "right_wrist_camera"
    source: sensor
    topic: "/camera/right_wrist/image_raw"
    message_type: "sensor_msgs/Image"
    time_domain: ros_header
    image_encoding: raw
    transport: zenoh
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "data"
        type: bytes
        required: true

  - name: "head_camera"
    source: sensor
    topic: "/camera/head/image_raw"
    message_type: "sensor_msgs/Image"
    time_domain: ros_header
    image_encoding: raw
    transport: zenoh
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "data"
        type: bytes
        required: true

  - name: "left_wrist_camera"
    source: sensor
    topic: "/camera/left_wrist/image_raw"
    message_type: "sensor_msgs/Image"
    time_domain: ros_header
    image_encoding: raw
    transport: zenoh
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "data"
        type: bytes
        required: true
```

## Reference

- [Sensor Interface Convention](../README.md) — Image + CameraInfo spec
- [Device & Stream Onboarding Guide](../../../docs/device-onboarding-schema-guide.md) — image_encoding rules
- [Data Collection Service](../../data_collection_service/README.md) — session YAML format
