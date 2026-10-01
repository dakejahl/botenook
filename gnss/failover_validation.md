# GNSS failover validation

Status: **plan**, 2026-09-30. Nothing here is implemented. This is step 7 of [pipeline_migration.md](pipeline_migration.md) §3, after reporting (step 6): show in SIH, then in flight, that a failed receiver hands over to the good one and EKF2 follows it. Scope is failures the checks can see; a selected receiver that is wrong but passes its checks (stuck, slow drift) is left out.

## 1. Pass criteria

For one failure of the selected receiver, armed, in the air, in a position-controlled mode, with a healthy standby:

- The selection changes once, to the standby: at the 2 s data timeout for a silent receiver, within the selector's hold time (2 s) for a failed check.
- Every EKF2 instance resets horizontal position once, and height if GNSS is the height reference. The published reset delta equals the offset between the receivers after lever arms.
- Local and global position stay valid. No failsafe triggers other than `gnss_lost` when `SYS_HAS_NUM_GNSS` asks for it. The flight mode does not change.
- In Hold the vehicle does not move (SIH ground truth): the setpoint follows the reset delta. In a mission it converges onto the track in the new receiver's frame.
- With `SENS_GNSS_PRIME` set, the selection returns to the recovered primary receiver only after it has been usable for 10 s while armed (at once while disarmed) and is about as available as the standby, with one more switch and one more reset. With -1 it doesn't return unless the recovered receiver is clearly more accurate.
- One event names the switch and its reason, and `gnss_fusion_state` shows why GNSS was not fused in between (both from step 6).

## 2. What exists

| Piece | State on main (`d5223850dd`) |
|---|---|
| Failure injection | `failure_injection_manager` takes `MAV_CMD_INJECT_FAILURE` (shell: `failure gps <type> -i <n>`, 1-based, 0 = all) and publishes `failure_injection`. Applied per receiver as the last step before `sensor_gnss` is published in `gps`, `septentrio`, the DroneCAN GNSS bridge (`gnss.cpp:575`), `sensor_gps_sim` and gz_bridge, so everything downstream (hub checks, timeout, selection, EKF2, commander, MAVLink streams) sees exactly what a real failure would produce for those fields. Not covered: driver-internal reactions (serial restart after data loss, DroneCAN NodeStatus), ramps, and `sensor_gnss_relative` (heading, gap 7). Needs `SYS_FAILURE_EN` (reboot). |
| GNSS failure types | `off`: nothing published. `stuck`: last good sample replayed with fresh timestamps. `wrong`: fix type from `SYS_FAIL_GPS_WRG`, jamming state from `SYS_FAIL_GPS_JAM`, position untouched. The other MAVLink types do nothing. |
| RC trigger | `SYS_FAIL_RC_SRC/UNIT/MODE/INST`: an aux switch injects while it is held on. |
| Second SIH receiver | `sensor_gps_sim` publishes instance 1 when `SENS_GNSS1_OFFX` or `_OFFY` is non-zero: instance 0's sample shifted by that lever arm, with its own `device_id`. |
| SIH integration tests | `test/mavsdk_tests`, `configs/sih-sitl.json`, per-entry `PX4_PARAM_*` overrides, run in CI. One GNSS case: every receiver `off` during a mission, blind land. |
| Unit tests | [#28798 feat(sensors/gps): Improve GPS selection](https://github.com/PX4/PX4-Autopilot/pull/28798) adds selector cases and an EKF reset-on-switch case. |

## 3. Gaps

1. **The second SIH receiver can't produce a reset delta.** It is a copy of instance 0 with the lever arm added, so after the lever-arm correction the receivers agree exactly and share one noise sequence. Needed: an explicit enable, independent noise, and a per-receiver position and height bias.
2. **No injection reaches the accuracy checks, the satellite count or the update rate.** In flight a receiver is unusable on fix < 3D, eph or epv > 50 m, sacc > 10 m/s, spoofing or jamming, and the ranked selection compares eph/epv and update interval; only fix and jamming can be injected. Planned, in `process_gnss` so every driver gets it: `wrong` also overrides eph, epv, speed accuracy, satellite count and spoofing state from new `SYS_FAIL_GPS_*` parameters (0 = unchanged, and `SYS_FAIL_GPS_WRG` gets an Unchanged value), and `slow` publishes one sample in N. Position error is out of scope.
3. **No hardware build has the manager.** `CONFIG_MODULES_FAILURE_INJECTION_MANAGER` is set only in `px4_sitl`. The flight test uses a local build for the test airframe's board; upstream board configs don't change.
4. **An injected failure is latched.** It holds until `ok` arrives; if the companion or its link dies, the receiver stays failed for the rest of the flight. Accepted for the flight test: the standby carries the vehicle and the pilot lands.
5. **MAVLink does not carry the selected receiver.** `GPS_RAW_INT` and `GPS2_RAW` are fixed receivers. Tests assert vehicle behaviour over MAVLink and the selection and resets from the log.
6. **SIH runs one EKF2 instance.** Multi-EKF is out of scope for this validation (few users).
7. **Heading ignores injection.** `sensor_gnss_relative` has no `process_gnss`, so `off` on a receiver leaves its heading alive. Required (Jake, 2026-09-30): an injected `off` takes the heading with it the way a real failure does. Heading comes from one receiver with two antennas (`SENS_GNSSn_HDG = 2`) or from a moving-base rover fed by another receiver (`SENS_GNSSn_HDG = 1`), so two parts: the `sensor_gnss_relative` publishers (`gps`, `septentrio`, DroneCAN `gnss_relative.cpp`) apply the generic `process()` on the publishing receiver's instance, which covers dual-antenna receivers completely; and for `HDG = 1` only, the hub drops the rover's heading sample while the receiver in the base slot (the other slot) is silent, injected or real. The hub part is real behaviour, not injection: a rover can't hold a moving-base heading without its base, and it mirrors the receiver's own corrections timeout. A dual-antenna receiver's heading is never gated on another receiver.
8. **SIH has no heading.** `sensor_gps_sim` publishes no `sensor_gnss_relative`; cases 13–14 need a simulated relative heading from ground-truth yaw on a chosen instance, as dual antenna or as a rover with instance 0 as its base.

## 4. SIH cases

Two receivers, `SENS_GNSS_PRIME = 0`, S the selected receiver, B the standby, B biased by a known offset.

| # | Injection | Exercises | Expect |
|---|---|---|---|
| 1 | S `off`, Hold | data timeout | §1 |
| 2 | S `wrong` (2D fix), Hold | check failure | §1 |
| 3 | S `off`, mission leg | reset while moving | §1, mission completes |
| 4 | S `off`, GNSS height reference, B biased in height | height reset | §1, altitude held |
| 5 | S `off`, then S `ok` | return hold while armed | stays on B for at least 10 s after S recovers, then returns to S with one more switch and reset; with `SENS_GNSS_PRIME = -1` and equal accuracy, stays on B |
| 6 | B `off` | standby failure | no switch, no reset; `gnss_lost` per `SYS_HAS_NUM_GNSS` |
| 7 | S and B `off`, then B `ok` | total loss and recovery | position invalid after `EKF2_NOAID_TOUT`, failsafe; fusion resumes on B |
| 8 | S toggled `off`/`ok` at 1 Hz | hysteresis | one switch to B and one reset; no return while S keeps failing (the return hold needs 10 s of uninterrupted usable samples) |
| 9 | 1 with `SENS_GNSS_PRIME = -1` | ranked selection | §1 |
| 10 | S accuracy above the relaxed gate (gap 2) | check failure on eph/sacc | §1 |
| 11 | `SENS_GNSS_PRIME = -1`, disarmed, B more accurate by more than the ratio (gap 2) | ranking on accuracy | selection moves to B after the hold |
| 12 | `SENS_GNSS_PRIME = -1`, disarmed, S `slow` (gap 2) | ranking on update rate | selection moves to B after the hold |
| 13 | S is the heading source (dual antenna, `SENS_GNSS0_HDG = 2`) or the moving base of B's heading (`SENS_GNSS1_HDG = 1`), GNSS yaw fused, S `off` | position failover plus heading loss | §1 for position; yaw fusion stops without a yaw reset, EKF2 continues on the mag |
| 14 | 13, then S `ok` | heading recovery | GNSS yaw resumes on the heading gate (#28846), no position switch back while armed |

Cases 1–9 need gap 1 only; case 10 and the ranked-selection cases on eph/epv and rate need gap 2; cases 13–14 need gaps 7 and 8.

## 5. Flight-test tool

One MAVSDK C++ codebase in `test/mavsdk_tests`: scenario functions (set the `SYS_FAIL_*` payload with `Param`, `Failure::inject`, wait, assert, restore) in one tester class. The catch2 cases wrap them with arm, takeoff and flight for SIH in CI; a `gnss_failover` binary from the same CMake project wraps them for the companion: connects to the vehicle, never arms or changes mode, runs one scenario with a duration, restores on exit. Builds natively on arm64. Live observation: `Telemetry` for position, position health, mode and `GPS_RAW_INT`; `MavlinkPassthrough` for `GPS2_RAW` and `ODOMETRY.reset_counter`. The selected receiver comes from the log, or the switch event after step 6. Described in `docs/en/debug/failure_injection.md`.

- **Inject**: `MAV_CMD_INJECT_FAILURE` for unit GPS with a type, an instance and a duration, then `ok`. `ok` is also sent on exit and on a signal.
- **Refuse to inject** unless `SYS_FAILURE_EN` is set, both receivers report at least a 3D fix (`GPS_RAW_INT`, `GPS2_RAW`), and the vehicle is armed, in the air and in a position-controlled mode.
- **Record** the command acks, both receivers, events, and position validity with wall and vehicle time.
- **Log report**: Python (pyulog) in `Tools/`, reads the ULog and checks §1: selected `device_id` over time (`vehicle_gnss.receiver.device_id`), `usable` and `failed_checks`, the reset counters and deltas on `vehicle_local_position`, position error against the setpoint. The same script grades SIH and flight logs.

The flight vehicle carries two DroneCAN receivers, so the injection is applied in the FC's DroneCAN GNSS bridge (`src/drivers/uavcan/sensors/gnss.cpp`); the nodes keep running. Flight firmware needs the manager built in (gap 3) and `SYS_FAILURE_EN = 1`.

## 6. Sequence

1. After step 6 merges: sim gap 1 and injection gap 2, then the cases in CI.
2. The tool against SIH; the log report passes.
3. After step 9, once everything has landed. Bench, props off, outdoors with a fix on both receivers: each injection from the companion, log report.
4. Flight, Position mode, pilot ready to take Altitude or Stabilized: S `off` in hover, S `wrong` in hover, S `off` in slow forward flight, S `off` on a mission leg. If S is the heading source (dual antenna, or the base of a moving-base rover), every S `off` is also case 13: position fails over and GNSS yaw hands over to the mag.

## 7. Open

- The FC board and companion computer of the flight vehicle are not named yet.
