# GNSS pipeline: target architecture and migration

Status: **plan**, revised 2026-09-22 against PX4 main `d551d33b61`. Only #27102 (heading on its own topic) is settled; everything after it is design. The case against blending and the selection policy are in [selection_fusion_and_heading.md](selection_fusion_and_heading.md) §6.

## 1. Today

GNSS quality is decided in four places:

| Where | Decides | Input |
|---|---|---|
| EKF2 `GnssChecks`, in every EKF instance | Fusion start and stop. Strict `EKF2_REQ_*` thresholds until the initial pass, re-armed whenever disarmed on the ground (#26346, #28678). In flight only fix < 3D, eph/epv > 50 m, sacc > 10 m/s, spoofing, jamming (#24909, v1.17). | `vehicle_gps_position` (blended) at the EKF delayed horizon |
| `GpsBlending` | Blend eligibility: fix ≥ 2D, 2 s timeout, accuracy fields present | each `sensor_gps` |
| Commander `gnssRedundancyCheck` | Receiver offline, fix < 3D, divergence between receivers → `gnss_lost` failsafe (`SYS_HAS_NUM_GNSS`, `COM_GNSSLOSS_ACT`) | each `sensor_gps` |
| ~30 consumers | Their own fix-type gates: geofence (raw GPS at fix ≥ 2), RemoteID, HomePosition, mag and baro calibration, open_drone_id | `vehicle_gps_position` |

A receiver's speed accuracy reaches the user like this: `sensor_gps.s_variance_m_s` → blending takes the minimum across receivers → `vehicle_gps_position` → EKF2 sample buffer → `GnssChecks` in each EKF instance, at the delayed horizon → `estimator_status.gps_check_fail_flags`, masked by `EKF2_GPS_CHECK` → commander `estimatorCheck`, which raises an event only while disarmed and only for the first failing flag. `estimator_gps_status` carries the same bits, has no subscriber, and is logged only by the logger's 2 Hz `estimator*` catch-all. In flight, commander reports spoofing, jamming, and "GNSS data fusion stopped/started", with no reason.

Telemetry and the EKF can also disagree on the receiver: `GPS_RAW_INT` follows `SENS_GPS_PRIME` (#27868) while the EKF flies the blend.

## 2. Target

```
drivers ──► sensor_gps[i]               driver report, never gated or annotated
        └─► sensor_gnss_relative[i]
                    │
sensors hub (VehicleGnss, replaces VehicleGPSPosition + GpsBlending)
  per receiver   GnssChecks ──► vehicle_gnss_status[i]   latest-wins, for commander, GCS, logs
  selection      primary + failover ──► vehicle_gnss     selected sample + its verdict + switch count
  heading        ──► vehicle_gnss_heading                 #27102
                    │
EKF2, every instance   fuses a vehicle_gnss sample only if it says usable; resets on a receiver switch;
                       keeps innovation checks, timeouts, EKF2_VEL_LIM; publishes why GNSS stopped
commander              pre-arm checks and gnss_lost from vehicle_gnss_status; loss reason from EKF2
```

### Decisions

1. **The verdict rides on the sample.** `vehicle_gnss` carries the selected receiver's fields plus `usable`, the failed-check flags, the antenna offset, the selected instance and a switch counter. EKF2 fuses a sample only if that sample says `usable`, so the gate is exact, needs no re-check inside EKF2, and replays with the sample. A latest-wins status topic can't gate fusion: it describes the newest sample, which is the GNSS delay plus buffer ahead of the one EKF2 is fusing. #28520 had to re-run the simplified checks per sample inside EKF2 to cover that gap.
2. **The hub output gets its own type, `VehicleGnss`, as every other sensor's does.** `sensor_accel`/`sensor_gyro` → `vehicle_imu`, `sensor_baro` → `vehicle_air_data`, `sensor_mag` → `vehicle_magnetometer`. `vehicle_gps_position` sharing `SensorGps` is why hub-only fields (`antenna_offset_*`) sit on the driver message.
3. **Per-receiver status is for monitoring only.** `vehicle_gnss_status[i]` is latest-wins, like `vehicle_imu_status`: `usable`, failed checks, check mode (strict or relaxed), drift metrics, rate, latency, inconsistency with the other receiver, and whether it is selected. Commander, GCS and logs read it; EKF2 does not.
4. **Checks run once per receiver, in the hub.** `GnssChecks` needs only the sample plus armed, `in_air` and `at_rest`, which the hub takes from `vehicle_status` and `vehicle_land_detected` as EKF2 does. Strict until the first pass and again whenever disarmed on the ground, relaxed after arming. A receiver that never passed stays strict, so a standby must clear the strict bar before it can be promoted.
5. **A receiver switch is an accounted reset, not an innovation.** Receivers disagree by more than noise: an RTK receiver sits in the base's frame (survey-in error, or the correction service's datum such as NAD83 or ETRS89), and AMSL differs with each receiver's geoid model (mosaic-X5 uses the STANAG 4294 10°×10° table). When the switch counter changes, EKF2 resets horizontal position, and height if GNSS height is in use, and publishes reset deltas, the same path as `EKF2Selector` instance changes.
6. **EKF2 says why it stopped using GNSS.** A reason on `estimator_status_flags`: data timeout, samples unusable (details in `vehicle_gnss_status`), innovations rejected, `EKF2_VEL_LIM`. Commander reports the reason instead of inferring it from flags.
7. **Thresholds belong to the hub**: `GNSS_CHECK` and `GNSS_REQ_*`, translated from `EKF2_GPS_CHECK` and `EKF2_REQ_*`. EKF2 still needs a speed-accuracy threshold for yaw-estimator gating in `gps_control.cpp`.
8. `estimator_gps_status` is deleted. `estimator_status.gps_check_fail_flags` is filled from the fused sample's flags until commander and HIGH_LATENCY2 read `vehicle_gnss_status`.

### Selection (replaces blending)

- `SENS_GPS_PRIME` stays authoritative. The primary is demoted when it is not `usable` for `T_fail` or times out, and only to a standby that is `usable`. It returns with hysteresis; while armed, only a failure causes a switch.
- `SENS_GPS_PRIME = -1` ranks receivers when there is no natural primary (dual independent RTK), on eph/epv, rate and latency, later EKF consistency. Fix type and satellite count aren't comparable across receivers, and on rover + moving base the rover's RTK Fixed is the worse navigation source. #28798's hold timer and EKF-requirements gate belong here.
- Divergence between receivers is computed here and published per receiver; commander's `gnss_lost` divergence test reads it instead of computing its own.
- Two receivers can't vote: the hub sees that they disagree, not which one is wrong. Attributing the fault needs the EKF state (shadow innovations, §5).

## 3. Order

| # | Step | Needs | State |
|---|---|---|---|
| 0 | Relaxed in-flight checks; strict checks re-armed only while disarmed on the ground (#24909, #26346, #28678) | | merged |
| 1 | Heading on its own topic (#27102) | | merging as-is |
| 2 | `GnssChecks` → `src/lib/gnss`, self-contained: own params struct, `run(sample, armed, in_air, at_rest)`. `EKF2_VEL_LIM` moves out of it into EKF2. EKF2 uses it from the lib with no behavior change. | | design |
| 3 | Selection replaces blending: one `GnssChecks` per receiver in the hub, primary + failover, `vehicle_gnss_status[i]`. `vehicle_gps_position` becomes the selected receiver verbatim with its real `device_id`, and EKF2 resets when `device_id` changes. Same release: `SENS_GPS_MASK` default 0 with a warn-once, ARK RTK pages, `tuning_the_ecl_ekf.md`. | 1, 2 | design (#28798 open) |
| 4 | `vehicle_gnss` with the per-sample verdict. EKF2 drops `GnssChecks` and reports why it stopped; commander moves to `vehicle_gnss_status`; consumers move off `vehicle_gps_position`, which is published alongside until they have; `GNSS_*` params. | 3 | design |
| 5 | Delete `GpsBlending`, `SENS_GPS_MASK`, `SENS_GPS_TAU` and `vehicle_gps_position` | 3 + one release, 4 | |
| 6 | Optional: `sensor_gps` → `sensor_gnss`, `SensorGps` field cleanup (#24399) | 5 | draft |

Why this order:

- **Selection before the verdict move.** The selector needs per-receiver health, and step 2 gives it `GnssChecks` without making EKF2 depend on the hub. Moving the verdict first would mean computing one for a blend while blending is still the default. Selection also removes the defect users see sooner, and after step 3 EKF2 still checks the same receiver with the same thresholds, so step 4 can be compared against it.
- **1 before 3.** Blending is what keeps CAN moving-base heading working (#19796 was closed, then #19907 made blending the default).
- **Rename where the semantics change.** `vehicle_gps_position` becomes `vehicle_gnss` in step 4, when the output becomes one receiver plus a verdict. Old logs keep the old topic, so log tools need to read both either way; Flight Review reads `vehicle_gps_position` in 8 files. The driver-side rename in step 6 is cosmetic: ~180 files, ~1350 references.
- **Reporting (#24355) depends on nothing** if it stays in commander on `estimator_status.gps_check_fail_flags` now and switches to the EKF2 reason and `vehicle_gnss_status` in step 4.

Carried over from #28610 and #28663: EKF2 not resetting checks when fusion stops falls out of step 4, and `EKF2_VEL_LIM` as an EKF-side sample limit is part of step 2.

## 4. Loss reporting (#24355)

The logs on #24355 are v1.15, which ran the strict thresholds in flight. Freefly `9c2fc42d`, single receiver, `EKF2_REQ_EPH = 1.0`, RTK stream stopped:

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
- Reason precedence: GNSS data timeout; quality checks (the failing flag from `gps_check_fail_flags`); innovation rejection (`cs_gnss_fault`, aid-source `innovation_rejected`); otherwise generic.
- One event, the same text in the log, and suppress the cascade that follows it.
- Until decision 6 lands, commander infers the reason from flags.
- After step 4, take the reason from EKF2 and name the receiver from `vehicle_gnss_status`.
- SITL: #28398 injects `off`, `stuck` and `wrong`. A check-failure reason on main needs fix loss or a new accuracy-degradation mode, since eph must exceed 50 m or sacc 10 m/s.

## 5. Open questions

1. `VehicleGnss` layout: a flat copy of the `SensorGps` fields, or `SensorGps` embedded as a nested field (supported, e.g. `EscStatus`)? Flat leaves consumer code unchanged apart from the type; nested keeps one field list.
2. Selector code: grow `SensorGpsSelector` (#27868) into the state machine, or extend the sensors voter (`DataValidatorGroup`)? The voter has priority and failover but no dwell times, no arm freeze, and no GNSS quality input.
3. Also publish `sensors_status_gnss` (`SensorsStatus`, as for baro and mag) so commander reuses the baro/mag consistency path, or is `vehicle_gnss_status[i]` enough?
4. `GPS_RAW_INT`: keep following `SENS_GPS_PRIME` (stable identity for the GCS, #27868), or follow the selected receiver (what the vehicle flies on)?
5. Height reference: fuse ellipsoid height, so a switch has no geoid-model step? EKF2 fuses AMSL today and converts with a geoid height learned from the current receiver (`_geoid_height_lpf`).
6. Shadow innovations make the hub subscribe to EKF output, closing a loop. Acceptable as demotion-only evidence behind hysteresis?
7. Ad-hoc consumer gates (geofence, RemoteID, HomePosition …): switch them to `vehicle_gnss.usable` in step 4, or later?
8. Per-receiver EKF lanes (Layer 2) would need a per-sample verdict for every receiver, not just the selected one: `vehicle_gnss` multi-instance like `vehicle_imu`, plus a selection. Only if Layer 2 happens.
