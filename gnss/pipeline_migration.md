# GNSS pipeline: target architecture and migration

Status: **plan**, 2026-09-22, against PX4 main `d551d33b61`. The case against blending and the selection policy are in [selection_fusion_and_heading.md](selection_fusion_and_heading.md) §6. This page covers who owns which decision, the topic contracts, and the PR order.

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
drivers ──► sensor_gnss[i]            driver report, never gated or annotated
        └─► sensor_gnss_relative[i]
                    │
sensors hub (VehicleGnss)
  per receiver   GnssChecks ──► sensor_gnss_status[i]        latest-wins
  selection      primary + failover ──► vehicle_gnss         one receiver, verbatim, real device_id
                                    └─► vehicle_gnss_status  selected receiver's checks + selection state
  heading        ──► vehicle_gnss_heading                    #27102
                    │
EKF2, every instance   fuses vehicle_gnss + vehicle_gnss_heading, gates on vehicle_gnss_status,
                       owns innovation checks, timeouts, resets; publishes why GNSS stopped
commander              health from the status topics, loss reason from EKF2
MAVLink                GPS_RAW_INT = vehicle_gnss.device_id, GPS2_RAW = the other receiver
```

Names are post-rename. Today and in #28520: `sensor_gps`, `vehicle_gps_position`, `sensor_gps_status`, `vehicle_gps_position_status`, `VehicleGPSPosition`.

### Who owns what

| Concern | Owner | Reason |
|---|---|---|
| Fix, sats, DOP, eph/epv/sacc, spoofing, jamming, drift and speed at rest | hub, once per receiver | Needs only the sample plus `in_air`/`at_rest`. Today it runs once per EKF instance, on a blend. |
| Staleness, rate, latency | hub | The selector needs it before any EKF sees the data. |
| Divergence between receivers | hub, as a selector input; commander reports it | Same data as selection. |
| Which receiver the vehicle flies on | hub selector | |
| Innovation consistency, fusion timeouts, resets, `EKF2_VEL_LIM` (#28663) | EKF2 | Needs estimator state or the delayed horizon. |
| Why GNSS stopped contributing | EKF2 publishes, commander reports | Only EKF2 knows whether it was data loss, the checks, or innovations. |

### Contracts (settled on #28520)

1. `sensor_gnss` is the driver's report. No health or "primary" fields.
2. `vehicle_gnss` has the same type as `sensor_gnss` and is always published, never withheld when a check fails. Several consumers only want telemetry.
3. Status topics are latest-wins, like `vehicle_imu_status`. EKF2 never joins them to a sample by timestamp, times check failure on its own clock, and keeps a per-sample guard (`isGnssSampleUsable`) against the race.
4. `GnssChecks` lives in `src/lib/gnss/` and takes `in_air`/`at_rest` from the same sources as EKF2 (`vehicle_status`, then `vehicle_land_detected`, 3 s staleness).
5. Thresholds are `GNSS_CHECK` / `GNSS_REQ_*`, translated from `EKF2_GPS_CHECK` / `EKF2_REQ_*`.
6. `estimator_status.gps_check_fail_flags` stays, filled from hub flags (`estimatorCheck`, HIGH_LATENCY2, log tools). `estimator_gps_status` is deleted.

Not yet in any PR: EKF2 publishing why it stopped using GNSS (§4).

### Selection (replaces blending)

- `SENS_GPS_PRIME` stays authoritative. The primary is demoted only on sustained failure: `checks_passed` false for `T_fail`, timeout, stale data, jamming or spoofing. It returns with hysteresis, and while armed only hard failures switch.
- `SENS_GPS_PRIME = -1` ranks receivers for setups with no natural primary, such as dual independent RTK. Rank on eph/epv, rate and latency, later EKF consistency. Fix type and satellite count are not comparable across receivers, and on rover + moving base the rover's RTK Fixed is the worse navigation source. #28798's hold timer and EKF-requirements gate belong here.
- Without blending, `vehicle_gnss_status` is the selected receiver's `sensor_gnss_status` plus selection state, and #28520's second check run on the blended output goes away.
- Two receivers cannot vote. The selector can see that they disagree, not which one is wrong. Attributing the fault needs a third opinion, the EKF state (shadow innovations, §5).

## 3. Order

| # | Step | PR | Needs | State |
|---|---|---|---|---|
| 0 | Relaxed in-flight checks; strict checks re-armed only while disarmed on the ground | #24909, #26346, #28678 | | merged |
| 1 | Heading on its own topic | #27102 | | draft |
| 2a | EKF2 stops resetting the checks when fusion stops | #28610 | | draft |
| 2b | `EKF2_VEL_LIM` moves out of `GnssChecks` into EKF2 | #28663 | | open |
| 2c | `GnssChecks` self-contained: own params and input struct, `in_air`/`at_rest` as arguments | | 2a, 2b | not opened |
| 2d | `GnssChecks` → `src/lib/gnss`; hub runs it per receiver and on its output; `VehicleGpsStatus`; `GNSS_*` params. No EKF2 behavior change. | | 2c | not opened |
| 2e | EKF2 gates on the hub status | #28520 | 2d | draft |
| 3 | Loss-reason reporting (#24355) | | | offered by Saibernard |
| 4 | Selection replaces blending. Same release: `SENS_GPS_MASK` default 0 with a warn-once, ARK RTK pages, `tuning_the_ecl_ekf.md` | #28798 reworked, or new | 1, 2e | #28798 open |
| 5 | Delete `GpsBlending`, `SENS_GPS_MASK`, `SENS_GPS_TAU` | | 4 + one release | |
| 6 | Rename gps → gnss: messages, topics, fields | #24399 + new | 5 | draft |

States as of 2026-09-22. 2a–2e is Jonas's split of #28520.

Why this order:

- **2 before 4.** The selector's health input is the per-receiver status that step 2 publishes. Selector-first means building fix/eph/timeout checks into the selector and deleting them again.
- **1 before 4.** Blending is what keeps CAN moving-base heading working (#19796 closed, #19907 made blending the default). Changing the default before #27102 lands brings that regression back.
- **1 and 2 are independent.** They meet at EKF2's check reset when GNSS yaw fails, which 2a removes; #27102 must not bring it back.
- **6 last.** Renaming first forces rebases of #27102, #28520 and the selector, and renames #28520's new topics twice. Scale today: ~180 files and ~1350 references in `src msg ROMFS Tools docs/en`. Flight Review reads `vehicle_gps_position` in 8 files (map, 3D view, GPS plots). Old logs keep the old names, so log tools must accept both.
- **3 depends on nothing** as long as it reads `estimator_status.gps_check_fail_flags`, not `estimator_gps_status`.

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
- Missing today: EKF2 does not publish why it stopped using GNSS, so commander infers it from flags. A reason field on `estimator_status_flags` would make it direct. Not needed for a first version.
- After 2e, name the receiver from `sensor_gnss_status`.
- SITL: #28398 injects `off`, `stuck` and `wrong`. A check-failure reason on main needs fix loss or a new accuracy-degradation mode, since eph must exceed 50 m or sacc 10 m/s.

## 5. Open questions

1. Selection state: fields on `vehicle_gnss_status`, or a topic of its own?
2. Selector code: grow `SensorGpsSelector` (prime resolution, #27868) into the state machine, or reuse the sensors voter (`DataValidatorGroup`) with GNSS health inputs?
3. Shadow innovations make the hub subscribe to EKF output, closing a loop. Acceptable as demotion-only evidence behind hysteresis?
4. Rename scope: messages, topics and fields only, or `SENS_GPS*` params too? Driver `GPS_*` params stay.
5. The ad-hoc consumer gates (geofence, RemoteID, HomePosition …): switch them to `vehicle_gnss_status.checks_passed` in step 2, or later?
6. `VehicleGpsStatus.flags` follows the `GNSS_CHECK` bit order; `estimator_status.gps_check_fail_flags` has its own. EKF2 maps between them until the latter is retired.
7. Per-receiver EKF lanes (Layer 2) would read `sensor_gnss[i]` and `sensor_gnss_status[i]` directly. Keep the status contract free of single-consumer assumptions.
