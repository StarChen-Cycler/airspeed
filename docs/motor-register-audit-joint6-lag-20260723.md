# Joint-6 Lag Investigation — Motor Flash Register Audit (2026-07-23)

**Question.** Joint 6 (wrist pitch) shows a visible command→state lag while the other
joints track tightly. Do the motor-internal "deep hyperparameters" (Damiao flash
registers) differ between joints — in particular between J6 and its wrist siblings
(J5/J7), or between the left and right arm?

**Answer. No.** Every configurable flash register is identical across all four wrist
motors on both arms, and identical left-vs-right for every joint position. The J6 lag
is not a motor-configuration problem; the remaining causes are mechanical/load and the
software gain ceiling documented in
`.memo/memodocs/latency_recovery_control_pipeline_20260722.md`.

## Method

1. Software-side audit of every per-joint setting in the control path
   (`arm_controller.py`, `config/robot.yaml`, `config_openarms_follower.py`,
   `lerobot/motors/damiao/`).
2. Read-only dump of all Damiao flash registers (RID 0–36, 50–55, 80–81) for all
   16 motors (can0 = right arm, can1 = left arm, ESC IDs 1–8) using the DM
   register-query command `0x33` to CAN ID `0x7FF`. No write (`0x55`), save (`0xAA`),
   enable (`0xFC`) or disable (`0xFD`) commands were sent; torque stayed disabled.
   Dump script: `/tmp/dump_motor_regs.py` on 226; raw output: `/tmp/motor_regs_dump.txt`.

> **Protocol note (corrected 2026-07-23, second probe):** the motors **accept
> classical CAN command frames but always reply with CAN FD frames**. A socket opened
> without `fd=True` (e.g. `can.interface.Bus(channel=..., interface="socketcan")`)
> cannot receive FD frames, so classical queries *look* like they're being ignored —
> the motor actually processed them; the reply was invisible to the non-FD socket.
> Any diagnostic tool must open the bus with `fd=True`; the *send* framing
> (classical vs FD) does not matter to the motor.

## Software-side audit results

Every per-joint value the driver sends is either identical for J5/J6/J7 or *stronger*
for J6:

| Setting | J5 | J6 | J7 | Source |
|---|---|---|---|---|
| kp / kd on the wire | 18 / 1.6 | **36 / 2.6** | 18 / 1.6 | `config/robot.yaml` |
| Motor model (encode/decode ranges) | DM4310 | DM4310 | DM4310 | `config_openarms_follower.py:75-77` |
| PMAX / VMAX / TMAX scaling | 12.5 / 30 / 10 | same | same | `tables.py:97` |
| Velocity feed-forward (MIT dq) | 0 | 0 | 0 | `arm_controller.py` |
| Slew clamp | 1.8°/cycle | same | same | `config/robot.yaml` |

- The per-joint friction model (`friction_fc/k/fv/fo`,
  `config_openarms_follower.py:95-106`, where J6 has unique values fc=0.093, k=242)
  is **dead code in the control path** — `arm_controller.py` only adds gravity torque
  (`_compute_gravity` → `_gravity_from_q`), never friction. It cannot cause the lag.
- Upstream OpenArm defaults also give J6 extra gain (`position_kp` default 31 for J6
  vs 24/25 for J5/J7, `config_openarms_follower.py:85`) — the arm's designers also
  knew wrist pitch needs more authority.

## Flash register dump — key rows (all 16 motors)

| Register | J1–J2 (DM8009) | J3–J4 (DM4340) | J5–J8 (DM4310) | Left = Right? |
|---|---|---|---|---|
| `CTRL_MODE` | 1 (MIT) | 1 | 1 | yes |
| `OC_VALUE` (over-current) | 0.8 | 0.8 | 0.8 | yes |
| `I_BW` (current-loop BW) | 1000 | 1000 | 1000 | yes |
| `V_BW` | 40 | 40 | 40 | yes |
| `MAX_SPD` | 600 | 600 | 600 | yes |
| `PMAX` / `VMAX` / `TMAX` | 12.5 / 45 / 54 | 12.5 / 10 / 28 | 12.5 / 30 / 10 | yes |
| `GR` (gear ratio) | 9 | 40 | 10 | yes |
| `NPP` (pole pairs) | 21 | 14 | 14 | yes |
| `KP_ASR` / `KI_ASR` | 0.0068 / 0.002 | 0.00384 / 0.002 | 0.00372 / 0.002 | yes |
| `UV_VALUE` / `OV_VALUE` / `OT_VALUE` | 15 / 54 / 100 | 15 / 32 / 100 | 15 / 32 / 100 | yes |
| `TIMEOUT` | 0 (disabled) | 0 | 0 | yes |
| `GREF` / `DETA` / `IQ_C1` / `VL_C1` | 1 / 4 / 2500 / 100 | same | same | yes |
| `SW_VER` | 925971510 | 925970741 | 925970485 | yes |

Within the wrist group, **J6 is byte-for-byte identical to J5/J7/gripper on every
configured register**, on both arms.

Per-unit values that legitimately differ (factory calibration / measurements, not
configuration): `M_OFF` (encoder zero offset), `U_OFF`/`V_OFF`, `RS`/`LS`/`FLUX`
(winding parameters, all within normal unit-to-unit spread), `K1`/`K2`, and the
measured `DAMP`/`INERTIA` registers. One mildly interesting data point: the measured
`DAMP` on the left wrist motors reads lower than the right (L-J6 0.000226 vs
R-J6 0.000300) — consistent with, but not proof of, different mechanical friction on
the left wrist.

## CAN framing source-code audit

This audit was prompted by the discovery that all register access must be done on
an FD-capable socket. The question was: is the existing source code using the
"wrong" framing method?

**Short answer:** the current Linux SocketCAN path *works*, but it is fragile and
contains dormant broken paths.

### What was checked

Every `can.Message(...)` and `can.interface.Bus(...)` call in
`src/robot_interface/openarm/openarm-control-ros2-adaptor/lerobot/motors/damiao/damiao.py`
(and the rest of `src/`) was inspected. All CAN traffic goes through this driver.

### Findings

1. **Bus is opened FD-enabled on the working path**
   - `damiao.py:160-166`: when `can_interface == "socketcan"` and
     `use_can_fd and data_bitrate is not None`, the bus is opened with
     `fd=True`, `bitrate=1_000_000`, `data_bitrate=5_000_000`.
   - This is the path actually used by `arm_controller.py` / `OpenArmsFollower`
     and is why the system works.

2. **Every outbound frame is classical CAN**
   - None of the ten `can.Message(...)` constructions in `damiao.py` pass
     `is_fd=True`:
     - `_enable_motor` / `_disable_motor` (`:254`, `:263`)
     - `_refresh_motor` (`:328`)
     - `_mit_control` (`:446`)
     - `_mit_control_batch` (`:506`)
     - `sync_read` / `sync_read_all_states` (`:663`, `:727`)
     - sync-write helper (`:808`)
   - This happens to be accepted by the DM4310-2EC V1.1 firmware: **the motors
     accept classical command frames but always reply with FD frames**. The
     FD-capable socket is therefore the critical requirement, not FD sends.

3. **Dormant broken paths in the same driver**
   - `damiao.py:169-173`: if `use_can_fd=False` (or `data_bitrate=None`), the
     SocketCAN bus is opened **without `fd=True`**. Replies would be invisible,
     breaking `_handshake`, `get_observation`, and control feedback.
   - `damiao.py:178-183`: the `slcan` branch (macOS / USB adapters) is opened
     without FD support. The config class still advertises `can_interface:
     "slcan"` and `use_can_fd`. Against these motors this path is broken.
   - `damiao.py:211-215`: `_handshake()` calls `_refresh_motor()` for every
     motor at connect. If the socket were non-FD, the motor would answer but
     the driver would see no reply and assume motors are missing.

4. **No other CAN code in the repo bypasses the driver**
   - A repo-wide grep found no other `can.Message(...)` or `can.interface.Bus(...)`
     calls in `src/` outside this driver. The in-process `joint_state_publisher`
     does not open its own socket; it consumes states already decoded from the
     FD replies.

### Practical impact

- **Today, on 226:** classical sends + FD replies + FD socket works. The system
  is not broken in production.
- **If anyone uses `use_can_fd=False` or the `slcan` path:** it will fail
  silently — motors will accept commands, but the driver will never see replies,
  so observations will be stale/missing and control may fault.
- **For diagnostics (e.g. the dump script used here):** the diagnostic must open
  the bus with `fd=True`; the command frame can be classical or FD.

### Recommendation (documentation only, no code change made)

Either remove the non-FD SocketCAN / SLCAN branches from `damiao.py`, or add an
assertion that the socket is FD-capable when these motors are used. The
`use_can_fd` config knob should be hard-coded to `True` for this hardware, and
`slcan` should not be advertised as supported.

## Conclusion

- Motor flash hyperparameters are **uniform** — no joint, including J6, has been
  mis-configured at the ESC level. The "different deep parameters" hypothesis is
  ruled out by direct readback.
- Software already gives J6 *more* gain than its siblings (kp 36 vs 18), so the
  residual lag is below both software and firmware configuration: mechanical
  (wrist-pitch fights gravity directly; left wrist has extra load/cable drag — see
  episode T4 where R-J6 tracking was fixed by kp 36 while L-J6 still under-reached
  command amplitude) plus the safety gain ceiling (kp 90 max on this mount, kd
  hard-capped at 5.0 by the 12-bit MIT wire format).
- No configuration change is recommended from this audit. If loaded left-J6 tracking
  must improve, the next step is mechanical inspection (cable routing, bearing
  friction, motor health), not parameters.
