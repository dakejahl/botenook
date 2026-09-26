# GNSS pipeline: target architecture and migration

Status: **plan**, revised 2026-09-25 after the RFC review. Only [#27102 refactor(ekf2): separate GNSS heading from position into independent topic](https://github.com/PX4/PX4-Autopilot/pull/27102) is settled; everything after it is design, decided as below; §5 lists what the review deferred. The case against blending and the selection policy are in [selection_fusion_and_heading.md](selection_fusion_and_heading.md) §6.

## 1. Today

GNSS quality is decided in four places:

| Where | Decides | Input |
|---|---|---|
| EKF2 `GnssChecks`, in every EKF instance | Fusion start and stop. Strict `EKF2_REQ_*` thresholds until the initial pass, re-armed whenever disarmed on the ground. In flight only fix < 3D, eph/epv > 50 m, sacc > 10 m/s, spoofing, jamming. | `vehicle_gps_position` (blended) at the EKF delayed horizon |
| `GpsBlending` | Blend eligibility: fix ≥ 2D, 2 s timeout, accuracy fields present | each `sensor_gps` |
| Commander `gnssRedundancyCheck` | Receiver offline, fix < 3D, divergence between receivers → `gnss_lost` failsafe (`SYS_HAS_NUM_GNSS`, `COM_GNSSLOSS_ACT`) | each `sensor_gps` |
| ~30 consumers | Their own fix-type gates: geofence (raw GPS at fix ≥ 2), RemoteID, HomePosition, mag and baro calibration, open_drone_id | `vehicle_gps_position` |

The in-flight relaxation is [#24909 \[EKF2\] Run simplified GNSS checks after initial fix](https://github.com/PX4/PX4-Autopilot/pull/24909) (v1.17); strict re-arming on the ground is [#26346 EKF: Constantly use strict GNSS checks while not in the air yet](https://github.com/PX4/PX4-Autopilot/pull/26346) and [#28678 fix(ekf2): keep GNSS checks relaxed between arming and takeoff](https://github.com/PX4/PX4-Autopilot/pull/28678).

A receiver's speed accuracy reaches the user like this: `sensor_gps.s_variance_m_s` → blending takes the minimum across receivers → `vehicle_gps_position` → EKF2 sample buffer → `GnssChecks` in each EKF instance, at the delayed horizon → `estimator_status.gps_check_fail_flags`, masked by `EKF2_GPS_CHECK` → commander `estimatorCheck`, which raises an event only while disarmed and only for the first failing flag. `estimator_gps_status` carries the same bits, has no subscriber, and is logged only by the logger's 2 Hz `estimator*` catch-all. In flight, commander reports spoofing, jamming, and "GNSS data fusion stopped/started", with no reason.

Telemetry and the EKF can also disagree on the receiver: since [#27868 fix(mavlink): select GPS_RAW_INT/GPS2_RAW instance via SENS_GPS_PRIME](https://github.com/PX4/PX4-Autopilot/pull/27868), `GPS_RAW_INT` follows `SENS_GPS_PRIME` while the EKF flies the blend.

## 2. Target

```
drivers ──► sensor_gnss[i]            driver report, never gated or annotated
        └─► sensor_gnss_relative[i]
                    │
sensors/vehicle_gnss (replaces vehicle_gps_position + GpsBlending)
  GnssChecks per receiver ─► SensorGnssSelector ─┬─► vehicle_gnss          selected sample + its check result + selection state
                                                 └─► sensors_status_gnss   per receiver: healthy, failed checks, diagnostics, inconsistency
  heading ─► vehicle_gnss_heading
                    │
EKF2, every instance   fuses a vehicle_gnss sample only if it is usable; resets on a receiver switch;
                       keeps innovation checks, timeouts, EKF2_VEL_LIM; reports why GNSS is not fused
commander              pre-arm and gnss_lost from sensors_status_gnss; loss reason from EKF2
MAVLink                GPS_RAW_INT = preferred receiver, GPS2_RAW = the other one, fixed for the session
```

### Decisions

1. **The check result rides on the sample.** Every `vehicle_gnss` sample carries `usable` and its failed checks, and EKF2 fuses a sample only if that sample is usable. The gate is exact, needs no re-check inside EKF2, and replays with the sample. A latest-wins status topic can't gate fusion: it describes the newest sample, which is the GNSS delay plus buffer ahead of the one EKF2 is fusing. [#28520 feat(sensors): Move gnss checks to vehicle_gps_position file](https://github.com/PX4/PX4-Autopilot/pull/28520) had to re-run checks per sample inside EKF2 to cover that gap.
2. **`VehicleGnss` nests `SensorGnss`.** The hub output gets its own type, as `vehicle_imu`, `vehicle_air_data` and `vehicle_magnetometer` do. It is a strict superset: `receiver` is the selected receiver's `sensor_gnss` sample, unmodified, and what the hub derives sits beside it: the corrected `timestamp_sample`, the antenna offset (moved off the driver message), the selection state and the check result. Heading stays on `vehicle_gnss_heading`; `receiver.heading` is whatever the driver reported.
3. **Per-receiver status is its own `SensorsStatusGnss`** on `sensors_status_gnss`, with per-receiver arrays like `SensorsStatusImu`: `healthy` is `usable`, the failed checks, a strict flag (held to `GNSS_REQ_*`), drift rates and filtered speed, `inconsistency` is the horizontal distance to the selected receiver after lever arms, `priority` marks the preferred receiver. The diagnostics don't fit the generic `SensorsStatus` (ThomasRigi on the RFC), and a standby receiver's diagnostics are what explain a failover, so they can't live on `vehicle_gnss`. `VehicleGnss` has no `check_mode`.
4. **Checks run once per receiver, in the hub.** `GnssChecks` needs only the sample plus armed, `in_air` and `at_rest`, which the hub takes from `vehicle_status` and `vehicle_land_detected` as EKF2 does. Strict until the first pass and again whenever disarmed on the ground, relaxed after arming. A receiver that never passed stays strict, so a standby must clear the strict bar before it can be selected.
5. **A receiver switch is an accounted reset, not an innovation.** Receivers disagree by more than noise: an RTK receiver sits in the base's frame (survey-in error, or the correction service's datum such as NAD83 or ETRS89), and AMSL differs with each receiver's geoid model (mosaic-X5 uses the STANAG 4294 10°×10° table). When `selection_count` changes, EKF2 resets horizontal position, and height if GNSS height is in use, and publishes reset deltas, the same path as `EKF2Selector` instance changes.
6. **EKF2 says why GNSS isn't fused**: `gnss_fusion_state` on `estimator_status_flags` (fused, no data, unusable, rejected by the innovation gate, `EKF2_VEL_LIM`, inactive). Commander reports it instead of inferring it from flags.
7. **Thresholds belong to the hub**: `GNSS_CHECK` and `GNSS_REQ_*`. EKF2 reads `GNSS_REQ_SACC` for yaw-estimator gating in `gps_control.cpp`.
8. `estimator_gps_status` is deleted. `estimator_status.gps_check_fail_flags` is filled from `vehicle_gnss.failed_checks` until commander and HIGH_LATENCY2 read the new topics, then removed.
9. **`GPS_RAW_INT` and `GPS2_RAW` stay on fixed receivers for the session**: the preferred receiver (instance 0 with `SENS_GNSS_PRIME = -1`) and the other one. Commander reports a switch as an event. Per-receiver GNSS messages wait for [mavlink/mavlink#2146 development.xml: add GNSS message](https://github.com/mavlink/mavlink/pull/2146).
10. **Consumers that care check `usable`** instead of their own fix-type tests. HomePosition must; geofence, RemoteID, open_drone_id and the calibrations are reviewed one by one in step 5.
11. **EKF2 keeps its own state, not the checker's.** The restart hold-off after a GNSS stop, today `_gnss_checks.reset()`, becomes a per-instance timer on `usable`; `resetHard()` on an EKF reset goes; the "quality poor" timeout tracks the last usable sample. `vehicle_gnss_heading` is published only from a receiver that passes its own heading checks (decision 12), so heading is no longer gated by the position receiver.
12. **Heading has its own gate, not the position checks.** GNSS yaw starts on the heading sample itself: σ under the yaw-reset threshold ([#28846](https://github.com/PX4/PX4-Autopilot/pull/28846)), fresh data, tilt alignment, and no spoofing or jamming reported by the heading receiver. EPH, EPV, PDOP, drift and speed don't gate it, and stopping position/velocity fusion doesn't stop yaw. The hold-off after a yaw fault, today the `GnssChecks` reset in `stopGnssFusion()`, becomes a yaw timer. Measured on an ARK G5 P3H outdoors (2026-09-26): heading σ under 2° from 36 s after power-up, yet yaw fused only from 104 s in one run and 128 s in the next, held back by EPV while Galileo HAS converged (epv 117 m at first fix, under `EKF2_REQ_EPV` 5 m at 115 s). Lifting the airframe failed PDOP for 0.45 s, and `stopGnssFusion()` then stopped yaw for 10 s with yaw innovations under 5.4°.

### Selection (replaces blending)

`SensorGnssSelector` (today `SensorGpsSelector`, which only resolves `SENS_GPS_PRIME` to an instance) grows into the state machine.

- The preferred receiver is `SENS_GNSS_PRIME` (instance or DroneCAN node ID). The selected receiver changes only when it is not usable for `T_fail` or times out, and only to a usable one. It returns to the preferred receiver with hysteresis; while armed, only a failure causes a switch.
- `SENS_GNSS_PRIME = -1` ranks receivers when there is no natural preference (dual independent RTK), on eph/epv, rate and latency, later EKF consistency. Fix type and satellite count aren't comparable across receivers, and on rover + moving base the rover's RTK Fixed is the worse navigation source. [#28798 feat(sensors/gps): Improve GPS selection](https://github.com/PX4/PX4-Autopilot/pull/28798) implements this: ranking switches only while disarmed, the 30% margin and 2 s hold as constants, and the per-receiver `usable` from step 3 instead of its own `EKF2_REQ_*` checks.
- Divergence between receivers is computed here and published as `sensors_status_gnss.inconsistency`; commander's `gnss_lost` divergence test reads it instead of computing its own.
- Two receivers can't vote: the hub sees that they disagree, not which one is wrong. Attributing the fault needs the EKF state (shadow innovations, §5).

### Renames

Names only; types and semantics are unchanged unless noted. Params go through `param_translation`.

| Today | Proposed |
|---|---|
| `SensorGps.msg`, `sensor_gps` | `SensorGnss.msg`, `sensor_gnss`, with unit-free field names (§6) |
| `vehicle_gps_position` (`SensorGps`) | `vehicle_gnss` (`VehicleGnss`, new) |
| `sensors/vehicle_gps_position/`, `VehicleGPSPosition` | `sensors/vehicle_gnss/`, `VehicleGnss` |
| `lib/gnss/SensorGpsSelector` | `lib/gnss/SensorGnssSelector` |
| ROS 2 `/fmu/out/vehicle_gps_position` | `/fmu/out/vehicle_gnss` |
| `SENS_GPS_PRIME` | `SENS_GNSS_PRIME` |
| `SENS_GPSn_ID`, `_OFFX`, `_OFFY`, `_OFFZ`, `_DELAY`, and `_ROT`, `_ROLL`, `_PITCH`, `_YAW` added in step 1 | `SENS_GNSSn_*` |
| `EKF2_GPS_CHECK`, `EKF2_REQ_*` | `GNSS_CHECK`, `GNSS_REQ_*` (`SENS_GNSS_REQ_EPH` would exceed 16 characters) |
| `EKF2_REQ_GPS_H` | `GNSS_REQ_TIME` |
| `SENS_GPS_MASK`, `SENS_GPS_TAU` | unchanged, deleted with blending |

Driver `GPS_*` params stay. The unit-free field names come from [#24399 Update SensorGps.msg to improve naming](https://github.com/PX4/PX4-Autopilot/pull/24399).

**DroneCAN speed accuracy is off by a square at both ends; fix it with the `speed_accuracy` rename.** `Fix2.covariance` carries velocity variance in (m/s)². cannode pushes `s_variance_m_s`, a 1σ in m/s, into it unsquared (`src/drivers/uavcannode/Publishers/GnssFix2.hpp:143`); [#26389 uavcannode: publisher: Fix2: fix eph/epv off by sqrt bug](https://github.com/PX4/PX4-Autopilot/pull/26389) fixed position and missed velocity. The bridge copies the variance into `s_variance_m_s` without a square root (`src/drivers/uavcan/sensors/gnss.cpp:451`). PX4 node to PX4 FC cancels out; mixed setups don't. An AP_Periph node's 0.5 m/s reaches EKF2 as 0.25 m/s, under the 0.3 m/s `EKF2_GPS_V_NOISE` floor, and passes `EKF2_REQ_SACC` = 0.5 up to 0.71 m/s. An ArduPilot FC reads a PX4 node's 0.1 m/s as 0.32 m/s. Nodes already in the field keep sending 1σ, which a fixed bridge would square-root, so either the bridge tells the two conventions apart (node software version) or nodes and FCs change together.

## 3. Order

0. **Merged:** relaxed in-flight checks and strict re-arming on the ground (§1).
1. **Heading on its own topic**: [#27102 refactor(ekf2): separate GNSS heading from position into independent topic](https://github.com/PX4/PX4-Autopilot/pull/27102). The baseline rotation moves to the per-receiver `SENS_GPSn_ROT` (`GPS_YAW_OFFSET`, `SEP_YAW_OFFS` and `EKF2_GPS_YAW_OFF` migrate to `SENS_GPSn_YAW`), drivers report the measured baseline and `sensor_gps.heading_offset` is deleted, and the heading path builds only with a consumer (`SENSORS_VEHICLE_GNSS_HEADING`). GNSS yaw also gets its own start and stop gate (decision 12): the PR already drops `_gps_intermittent` from the yaw start conditions, and `_gnss_checks.passed()` there and `stopGnssYawFusion()` in `stopGnssFusion()` are the remaining places where position quality holds heading back. Driver side merged as [PX4/PX4-GPSDrivers#240 fix(ubx): real UTC time and no fake sample timestamp on NAV-RELPOSNED/DAHEADING](https://github.com/PX4/PX4-GPSDrivers/pull/240) and [PX4/PX4-GPSDrivers#241 refactor(gps): drop heading_offset, report the raw baseline heading](https://github.com/PX4/PX4-GPSDrivers/pull/241). `SENS_GPS_MASK` defaults to 0 right after it, with a warn-once, since blending no longer keeps heading available.
2. **Rename, and `VehicleGnss` as the hub output.** `SensorGnss`/`sensor_gnss` and `VehicleGnss`/`vehicle_gnss` with `receiver`, `antenna_offset` and the selection fields; `SENS_GNSS*` params; module, selector, logger, replay and ROS 2 topic names. Every consumer moves once. Flight Review, pyulog-based tools and PlotJuggler layouts learn both names. No behavior change: until step 4 the output can still be the blend (`SELECTION_BLENDED`). The one exception is the DroneCAN speed accuracy fix (§2, Renames). Can be two PRs (message rename, then hub output) in one release, so logs change layout once.
3. **`GnssChecks` → `src/lib/gnss`, one per receiver in the hub**: own params struct, `run(sample, armed, in_air, at_rest)`, `GNSS_CHECK`/`GNSS_REQ_*` params, `sensors_status_gnss`. `EKF2_VEL_LIM` moves out of it into EKF2. Diagnostics only: EKF2 still runs its own checks, so no behavior change. Independent of step 2. Proposed to ThomasRigi and Matheo-13 on [#28520 feat(sensors): Move gnss checks to vehicle_gps_position file](https://github.com/PX4/PX4-Autopilot/pull/28520).
4. **Selection replaces blending**, on step 3's per-receiver `usable`: `SensorGnssSelector` state machine, live `selection_count`, EKF2 reset on a switch, commander's `gnssRedundancyCheck` reads `sensors_status_gnss`. Same release: the ARK RTK pages and `tuning_the_ecl_ekf.md`. a-bernasconi on [#28798 feat(sensors/gps): Improve GPS selection](https://github.com/PX4/PX4-Autopilot/pull/28798).
5. **Check result on the sample.** `usable` and `failed_checks` on `vehicle_gnss`; EKF2 drops `GnssChecks` and publishes `gnss_fusion_state`; commander's pre-arm and in-flight GNSS messages move to `vehicle_gnss` and `sensors_status_gnss`; consumers gate on `usable`; `estimator_gps_status` is deleted.
6. **Delete blending** (`GpsBlending`, `SENS_GPS_MASK`, `SENS_GPS_TAU`) one release after step 4.

Why this order:

- **Rename right after the heading split.** The heading split is the only in-flight change that survives, so every later step is written once against final names, and consumers, ROS 2 users and log tools migrate once. The step-5 fields are additions to `VehicleGnss`.
- **`GnssChecks` as a lib before the selector.** The selector needs a check per receiver; the lib gives it one without the hub depending on EKF2.
- **Selector before the check result on the sample.** No check result is ever computed for a blend, and after step 4 EKF2 still checks the same receiver with the same thresholds, so step 5 can be compared against it.
- **Heading split before the selector.** Blending is what keeps CAN moving-base heading working: [#19796 Always publish GPS heading in vehicle_gps_position if available](https://github.com/PX4/PX4-Autopilot/pull/19796) was closed and [#19907 Enable GPS Blending by default](https://github.com/PX4/PX4-Autopilot/pull/19907) turned blending on instead.
- **Loss reporting (§4) depends on nothing** if it stays in commander on `estimator_status.gps_check_fail_flags` now and switches to `gnss_fusion_state` and the new topics in step 5.

Carried over from open PRs: EKF2 not resetting checks when fusion stops ([#28610 feat(ekf2)!: remove gnss_checks reset inside gps_control.cpp](https://github.com/PX4/PX4-Autopilot/pull/28610)) falls out of step 5, and `EKF2_VEL_LIM` as an EKF-side sample limit ([#28663 fix(ekf2): reject GNSS samples with vel above EKF2_VEL_LIM in EKF instead of in the on ground GNSS checks](https://github.com/PX4/PX4-Autopilot/pull/28663)) is part of step 3.

## 4. Loss reporting

[#24355 \[Bug\] GPS failure, notify user](https://github.com/PX4/PX4-Autopilot/issues/24355). The logs there are v1.15, which ran the strict thresholds in flight. Freefly `9c2fc42d`, single receiver, `EKF2_REQ_EPH = 1.0`, RTK stream stopped:

| t (s) | |
|---|---|
| 83.52 | fix 6 → 3, eph 0.02 → 3.5 m |
| 83.56 | `gps_check_fail_flags` hacc + vacc (hacc only from 84.16) |
| 83.74 | last GNSS fusion; samples skipped from here |
| 88.35 | `xy_valid` false (`EKF2_NOAID_TOUT` 5 s) → "Failsafe: blind land" |
| 90.34 | `cs_gps` false (`reset_timeout_max` 7 s) → "GNSS data fusion stopped", at Info because position was already invalid |
| 91.56 | checks pass |
| 92.54 | fusion resumes, `xy_valid` true |

On main this does not trip: the in-flight hacc gate is 50 m. The ARK logs (sacc over `EKF2_REQ_SACC`) are the same class; main's in-flight sacc gate is 10 m/s.

What is left is the reporting: the failsafe gave no reason, and the one GNSS event came two seconds later, at Info.

- Trigger on local position going invalid, the failsafe the user sees. `cs_gps` falling lags it by `reset_timeout_max − EKF2_NOAID_TOUT`, 2 s with defaults.
- Reason precedence: GNSS data timeout; quality checks (the failing flag); innovation rejection (`cs_gnss_fault`, aid-source `innovation_rejected`); otherwise generic.
- One event, the same text in the log, and suppress the cascade that follows it.
- Until step 5, commander infers the reason from `gps_check_fail_flags` and the fault flags. From step 5, `gnss_fusion_state` gives it directly and `sensors_status_gnss` names the receiver.
- SITL: [#28398 fix(gz_bridge): support GPS failure injection](https://github.com/PX4/PX4-Autopilot/pull/28398) injects `off`, `stuck` and `wrong`. A check-failure reason on main needs fix loss or a new accuracy-degradation mode, since eph must exceed 50 m or sacc 10 m/s.

## 5. Deferred follow-ups

- **Height reference on a switch** (deferred at the RFC review). Fuse ellipsoid height so a switch has no geoid-model step. EKF2 fuses AMSL today and converts with a geoid height learned from the current receiver (`_geoid_height_lpf`).
- **Shadow innovations** (deferred at the RFC review; both reviewers against). Comparing each receiver against the EKF state would let the hub attribute a disagreement, but it makes the hub subscribe to EKF output: a feedback loop from estimator to its own input selection.
- **ROS 2 versioning of `VehicleGnss`** (deferred at the RFC review). `SensorGps` is unversioned, so `/fmu/out/vehicle_gps_position` breaks at the rename either way. Versioning `VehicleGnss` would also version the nested `SensorGnss`, which puts every driver-message change through the translation process.
- **Rover reconfiguration for fallback use** (rover + moving base). From the side note in [#25516 Concept: Separate GNSS position and heading topics](https://github.com/PX4/PX4-Autopilot/pull/25516), implemented in [aviant-tech/PX4-Autopilot#104 Dual F9P for yaw with proper redundancy behaviour](https://github.com/aviant-tech/PX4-Autopilot/pull/104). When the selector promotes the rover to position source, reconfigure it to standard mode: disable RTCM input and issue a soft position reset, which brings it back within 1–2 s instead of navigating on rover output that goes ~900 ms stale when moving-base corrections degrade. The moving base failing is what degrades the rover, so the selector may start the reconfiguration as the moving base's health collapses rather than after the switch. Needs the step-4 selector plus driver support; DroneCAN receivers need it in node firmware.
- **Per-receiver EKF lanes** (`EKF2_MULTI_GPS`), intentionally deferred. They would need a check result per sample for every receiver, not just the selected one: `vehicle_gnss` multi-instance like `vehicle_imu`.

## 6. Prototype messages

### `SensorGnss.msg` (renamed from `SensorGps.msg`)

Field renames: `latitude_deg` → `latitude`, `longitude_deg` → `longitude`, `altitude_msl_m` → `altitude_msl`, `altitude_ellipsoid_m` → `altitude_ellipsoid`, `s_variance_m_s` → `speed_accuracy`, `c_variance_rad` → `course_accuracy`, `noise_per_ms` → `noise`, `vel_m_s` → `ground_speed`, `vel_n_m_s`/`vel_e_m_s`/`vel_d_m_s` → `vel_north`/`vel_east`/`vel_down`, `cog_rad` → `course`. `antenna_offset_x/y/z` move to `VehicleGnss`. `vehicle_gps_position` leaves the topic list.

```text
# GNSS receiver report
#
# One instance per receiver, published by its driver: the receiver's own solution, unmodified.
# The sensors module publishes the selected receiver on vehicle_gnss.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Measurement time if the driver knows it, else 0. vehicle_gnss carries the corrected value

uint32 device_id # [-] Unique device ID for the sensor that does not change between power cycles

float64 latitude           # [deg] Latitude, allows centimeter level RTK precision
float64 longitude          # [deg] Longitude, allows centimeter level RTK precision
float64 altitude_msl       # [m] Altitude above MSL, from the receiver's own geoid model
float64 altitude_ellipsoid # [m] Altitude above the ellipsoid

float32 speed_accuracy  # [m/s] Speed accuracy estimate
float32 course_accuracy # [rad] Course accuracy estimate

uint8 fix_type                             # [@enum FIX_TYPE] Value 0 is also valid to represent no fix
uint8 FIX_TYPE_NONE                   = 1
uint8 FIX_TYPE_2D                     = 2
uint8 FIX_TYPE_3D                     = 3
uint8 FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4
uint8 FIX_TYPE_RTK_FLOAT              = 5
uint8 FIX_TYPE_RTK_FIXED              = 6
uint8 FIX_TYPE_EXTRAPOLATED           = 8

float32 eph  # [m] Horizontal position accuracy
float32 epv  # [m] Vertical position accuracy
float32 hdop # [-] Horizontal dilution of precision
float32 vdop # [-] Vertical dilution of precision

int32 noise                   # [-] Noise level per millisecond
uint16 automatic_gain_control # [-] Automatic gain control monitor

uint8 jamming_state           # [@enum JAMMING_STATE]
uint8 JAMMING_STATE_UNKNOWN   = 0
uint8 JAMMING_STATE_OK        = 1
uint8 JAMMING_STATE_MITIGATED = 2
uint8 JAMMING_STATE_DETECTED  = 3
int32 jamming_indicator       # [-] Jamming indicator

uint8 spoofing_state           # [@enum SPOOFING_STATE]
uint8 SPOOFING_STATE_UNKNOWN   = 0
uint8 SPOOFING_STATE_OK        = 1
uint8 SPOOFING_STATE_MITIGATED = 2
uint8 SPOOFING_STATE_DETECTED  = 3

uint8 authentication_state              # [@enum AUTHENTICATION_STATE] Combined signal authentication state (e.g. Galileo OSNMA)
uint8 AUTHENTICATION_STATE_UNKNOWN      = 0
uint8 AUTHENTICATION_STATE_INITIALIZING = 1
uint8 AUTHENTICATION_STATE_ERROR        = 2
uint8 AUTHENTICATION_STATE_OK           = 3
uint8 AUTHENTICATION_STATE_DISABLED     = 4

float32 ground_speed # [m/s] Ground speed
float32 vel_north    # [m/s] North velocity
float32 vel_east     # [m/s] East velocity
float32 vel_down     # [m/s] Down velocity
float32 course       # [rad] [@range -PI, PI] Course over ground, not heading
bool vel_ned_valid   # NED velocity is valid

int32 timestamp_time_relative # [us] timestamp + timestamp_time_relative = time of the UTC timestamp since system start
uint64 time_utc_usec          # [us] UTC time from the receiver, 0 until known

uint8 satellites_used # [-] Satellites used in the solution

uint32 system_error                      # [@enum SYSTEM_ERROR] Bitmask of receiver errors
uint32 SYSTEM_ERROR_OK                   = 0
uint32 SYSTEM_ERROR_INCOMING_CORRECTIONS = 1
uint32 SYSTEM_ERROR_CONFIGURATION        = 2
uint32 SYSTEM_ERROR_SOFTWARE             = 4
uint32 SYSTEM_ERROR_ANTENNA              = 8
uint32 SYSTEM_ERROR_EVENT_CONGESTION     = 16
uint32 SYSTEM_ERROR_CPU_OVERLOAD         = 32
uint32 SYSTEM_ERROR_OUTPUT_CONGESTION    = 64

float32 heading          # [rad] [@range -PI, PI] [@invalid NaN] Measured dual-antenna baseline heading, for receivers without sensor_gnss_relative. Use vehicle_gnss_heading, rotated by SENS_GNSSn_ROT
float32 heading_accuracy # [rad] Heading accuracy

float32 rtcm_injection_rate  # [Hz] Correction injection rate
uint8 selected_rtcm_instance # [-] uORB instance used for corrections

uint8 corrections_protocol         # [@enum CORRECTIONS_PROTOCOL] Protocol of the last correction message the receiver parsed
uint8 CORRECTIONS_PROTOCOL_UNKNOWN = 0
uint8 CORRECTIONS_PROTOCOL_RTCM3   = 1
uint8 CORRECTIONS_PROTOCOL_SPARTN  = 2
uint8 CORRECTIONS_PROTOCOL_HAS     = 3 # Galileo High Accuracy Service, received on E6
uint8 CORRECTIONS_PROTOCOL_PMP     = 4 # SPARTN over L-band (u-blox NEO-D9S)
uint8 CORRECTIONS_PROTOCOL_QZSS_L6 = 5 # QZSS CLAS
bool corrections_crc_failed        # Last correction message failed its CRC or content check

uint8 corrections_msg_used          # [@enum CORRECTIONS_MSG_USED] Whether the receiver used the last correction message
uint8 CORRECTIONS_MSG_USED_UNKNOWN  = 0
uint8 CORRECTIONS_MSG_USED_NOT_USED = 1
uint8 CORRECTIONS_MSG_USED_USED     = 2

# TOPICS sensor_gnss
```

### `VehicleGnss.msg` (new)

The check-result block arrives in step 5; everything above it lands in step 2.

```text
# Selected GNSS solution
#
# Published by the sensors module for every sample of the selected receiver, usable or not:
# the receiver's report, the selection state, and the check result for that sample.
# Heading is on vehicle_gnss_heading.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Measurement time, corrected by SENS_GNSSn_DELAY or PPS. Use this, not receiver.timestamp_sample

SensorGnss receiver # The selected receiver's sensor_gnss sample, unmodified

float32[3] antenna_offset # [m] [@frame FRD] Antenna position of the selected receiver (SENS_GNSSn_OFFX/Y/Z)

uint8 selected_instance       # [-] [@invalid 255 while blended] sensor_gnss instance of the selected receiver
uint8 selection_count         # [-] Increments when the selected receiver changes; EKF2 resets position on a change
uint8 selection_reason        # [@enum SELECTION] Why this receiver is selected
uint8 SELECTION_PREFERRED = 0 # The SENS_GNSS_PRIME receiver
uint8 SELECTION_FAILOVER  = 1 # The preferred receiver is unusable or timed out
uint8 SELECTION_RANKED    = 2 # SENS_GNSS_PRIME = -1
uint8 SELECTION_ONLY      = 3 # The only receiver present
uint8 SELECTION_BLENDED   = 4 # Blended output, receiver.device_id = 0. Removed with blending

# Check result for this sample (step 5)
bool usable                 # Passes the checks enabled in GNSS_CHECK, and has for long enough (GNSS_REQ_TIME)
uint16 failed_checks        # [@enum CHECK] Failed checks among those enabled in GNSS_CHECK, in GNSS_CHECK bit order
uint16 CHECK_NSATS   = 1    # Satellites below GNSS_REQ_NSATS
uint16 CHECK_PDOP    = 2    # PDOP above GNSS_REQ_PDOP
uint16 CHECK_EPH     = 4    # eph above GNSS_REQ_EPH, 50 m when relaxed
uint16 CHECK_EPV     = 8    # epv above GNSS_REQ_EPV, 50 m when relaxed
uint16 CHECK_SACC    = 16   # Speed accuracy above GNSS_REQ_SACC, 10 m/s when relaxed
uint16 CHECK_HDRIFT  = 32   # Horizontal drift at rest above GNSS_REQ_HDRIFT
uint16 CHECK_VDRIFT  = 64   # Vertical drift at rest above GNSS_REQ_VDRIFT
uint16 CHECK_HSPEED  = 128  # Horizontal speed at rest above GNSS_REQ_HDRIFT
uint16 CHECK_VSPEED  = 256  # Vertical speed at rest above GNSS_REQ_VDRIFT
uint16 CHECK_SPOOFED = 512  # Receiver reports spoofing
uint16 CHECK_FIX     = 1024 # Fix below GNSS_REQ_FIX, 3D when relaxed
uint16 CHECK_JAMMED  = 2048 # Receiver reports jamming

# TOPICS vehicle_gnss
```

### `SensorsStatusGnss.msg`

```text
# Per-receiver GNSS health and check diagnostics
#
# Published by the sensors module from its per-receiver GnssChecks. Latest-wins: to gate on a sample, use vehicle_gnss.usable.

uint64 timestamp # [us] Time since system start

uint32 device_id_selected # [-] Receiver that vehicle_gnss carries

uint32[4] device_ids                 # [-] One entry per receiver, 0 where unused
bool[4] healthy                      # Passes its checks, and has for long enough (GNSS_REQ_TIME)
uint8[4] priority                    # [-] Highest for the SENS_GNSS_PRIME receiver
float32[4] inconsistency             # [m] Horizontal distance to the selected receiver after lever arms, for commander's divergence check
uint16[4] failed_checks              # [@enum CHECK] VehicleGnss CHECK_* bits
bool[4] strict                       # Held to GNSS_REQ_*: never passed yet, or disarmed on the ground
float32[4] drift_rate_horizontal     # [m/s] Horizontal position drift rate at rest
float32[4] drift_rate_vertical       # [m/s] Vertical position drift rate at rest
float32[4] speed_horizontal_filtered # [m/s] Filtered horizontal speed at rest

# TOPICS sensors_status_gnss
```

### `EstimatorStatusFlags.msg`

```diff
 bool fs_bad_acc_clipping      # 11 - true if delta velocity data contains clipping (asymmetric railing)
+
+# GNSS fusion
+uint8 gnss_fusion_state           # [@enum GNSS_FUSION] Why the latest GNSS sample was or was not fused
+uint8 GNSS_FUSION_FUSED     = 0
+uint8 GNSS_FUSION_NO_DATA   = 1   # No vehicle_gnss sample within the fusion timeout
+uint8 GNSS_FUSION_UNUSABLE  = 2   # vehicle_gnss.usable is false, see failed_checks
+uint8 GNSS_FUSION_REJECTED  = 3   # Innovation outside the gate
+uint8 GNSS_FUSION_VEL_LIMIT = 4   # Velocity above EKF2_VEL_LIM
+uint8 GNSS_FUSION_INACTIVE  = 5   # Not in use: EKF2_GPS_CTRL, alignment, or a declared GNSS fault
```

### Removed

`EstimatorGpsStatus.msg` in step 5. `EstimatorStatus.gps_check_fail_flags` and its `GPS_CHECK_FAIL_*` constants once commander and HIGH_LATENCY2 read `vehicle_gnss` and `sensors_status_gnss`.
