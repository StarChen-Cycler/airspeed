# OpenArm Controller

Arm state publishing and motor control for the OpenArm bimanual robot.
Deploy independently of camera streams.

## Contents

| Script | Role | ROS2 required? |
|--------|------|---------------|
| `arm_controller.py` | Receives IK commands via WebSocket → drives motors; publishes `JointState` **in-process** from its own observation reads (default) | Yes (for state publishing) |
| `joint_state_publisher.py` | In-process `JointState` publisher used by `arm_controller.py` | Yes |
| `arm_state_publisher.py` | Legacy standalone publisher (rollback only, `--external-publisher`) | Yes |

## Published ROS2 Topics

| Topic | Type | Content |
|-------|------|---------|
| `/arm/left/joint_state` | `sensor_msgs/JointState` | Left arm: 7 joints + gripper (position/velocity/effort) |
| `/arm/right/joint_state` | `sensor_msgs/JointState` | Right arm: 7 joints + gripper (position/velocity/effort) |

Each `JointState` includes `header.stamp` (taken at observation read time),
`header.frame_id=base_link`, and `name[]` from `config/robot.yaml`.
`velocity[]` and `effort[]` are filled from the motor status frames for live
consumers; the data collection service still records position only (no
session YAML change).

**Why in-process publishing:** the controller already reads fresh motor
states every control cycle for gravity compensation. Publishing from the
same process makes it the single SocketCAN owner on can0/can1 — the legacy
subprocess opened a second unfiltered socket whose kernel RX buffer
saturated, so it published stale positions with fresh timestamps (~0.3–1 s
recorded cmd→state lag artifact).

## Quick Start

**Launch script** (starts controller with in-process publisher):

```bash
cd openarm-control-ros2-adaptor
bash launch/start.sh
```

Calibrates, homes, waits for ENTER, then streams motor commands and publishes
`JointState` to ROS2.

**Rollback — legacy external publisher** (spawns `arm_state_publisher.py` as
a subprocess, mutually exclusive with the in-process one):

```bash
python3 arm_controller.py --external-publisher
```

If the in-process publisher fails to start (e.g. ROS2 environment not
sourced), the controller logs a warning and falls back to the legacy
subprocess automatically. `--no-publisher` disables state publishing.

**Standalone — Arm state publisher (legacy):**

```bash
cd openarm-control-ros2-adaptor
source /opt/ros/humble/setup.bash
python3 arm_state_publisher.py
```

**Standalone — Arm controller** (requires IK adaptor running on port 5200):

```bash
cd openarm-control-ros2-adaptor
python3 arm_controller.py --ws-uri ws://localhost:5200/ws/arm
```

## Prerequisites

The lerobot SDK is vendored in `lerobot/` — no separate install needed.
`LEROBOT_SRC` auto-detects from the adaptor directory.

| Requirement | Check | Install |
|-----------|-------|---------|
| Python 3.10+ | `python3 --version` | Activate your env (conda/venv) before running |
| ROS2 Humble | `source /opt/ros/humble/setup.bash` | Needed for state publishing (in-process default and legacy publisher) |
| SocketCAN | `ip link show can0` | `sudo ip link set can0 type can bitrate 1000000; sudo ip link set up can0` |
| IK adaptor running | `curl -s http://localhost:5200/ws/arm` | Start `openarm-ik-ros2-adaptor` first |
| Python packages | `python3 -c "import numpy, yaml, websockets"` | `pip install numpy pyyaml websockets` |
| Gravity compensation | Auto-detected | Meshes auto-copied from IK adaptor on first start; no manual setup |

## Python Environment

Activate your environment before running. The scripts use `python3` from PATH:

```bash
# conda
source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot

# venv
source ~/venvs/myenv/bin/activate
```

## Configuration

Edit `config/robot.yaml`: CAN bus ports, joint names, Kp/Kd gains, publish rate,
safety clamps, URDF path for gravity compensation, WebSocket URI.
Home positions are in `../robot_shared.yaml` — shared with the IK adaptor.

## Session YAML

```yaml
streams:
  - name: "left_joint_state"
    source: robot
    topic: "/arm/left/joint_state"
    message_type: "sensor_msgs/JointState"
    time_domain: ros_header
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "position"
        type: sequence
        required: true

  - name: "right_joint_state"
    source: robot
    topic: "/arm/right/joint_state"
    message_type: "sensor_msgs/JointState"
    time_domain: ros_header
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "position"
        type: sequence
        required: true
```
