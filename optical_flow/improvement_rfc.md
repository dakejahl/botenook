# [RFC] Optical flow position hold

Status: draft plan, 2026-10-02. This replaces the earlier problem survey, which is in git history. Research behind it: `~/Downloads/project_notes/flow_testing/analysis/2026-10-02-rfc-research/`.

Evidence tags: **[code]** read in PX4 main at [e6db1b3a14](https://github.com/PX4/PX4-Autopilot/commit/e6db1b3a14), **[measured]** in one of our logs, **[reported]** by a third party, **[derived]** arithmetic or inference never observed in flight.

## Goal

A quadcopter with an ARK Flow (PAW3902) or ARK Flow MR (PAA3905) on DroneCAN and no GNSS holds position in Position mode. Flow as a second velocity source next to GNSS is out of scope.

Three symptoms define the work:

1. Hold is fine below about 6 m and degrades with height into wandering or toilet-bowling.
2. Indoors, or whenever the barometer corrupts height above ground (HAGL), the vehicle drifts, because flow velocity is angular rate times HAGL.
3. Over low-texture surfaces such as indoor concrete the flow data is bad and gets fused anyway.

Proposed acceptance, to be confirmed by step 0:

| Envelope | Target |
|---|---|
| Below 5 m over a surface the chip tracks | Within 1 m for 60 s hands-off (the existing MC_06 test card criterion) |
| 5 m to the rangefinder's daylight reach, about 15 m | Bounded drift, no oscillation |
| Range lost or surface not trackable | Velocity stays bounded for a stated time, then a defined failsafe. No fly-away |
| Above rangefinder reach | Not supported. The height ceiling keeps the vehicle out |

No comparable product documents a hold at 20 m: DJI's camera-based system claims 0.3 to 13 m, Parrot below 5 m **[reported]**.

## Evidence so far

- No test flight has exercised the goal. Every Position-mode flight had GNSS position and velocity fused and GNSS as the height reference **[measured]**.
- The only flow-only hold data is 9 unplanned seconds at the start of flight 5 (`16_26_40`), over sunlit black asphalt before GNSS fusion began: velocity 0.85 m/s RMS off GNSS, about 5 m of position divergence, pilot takeover **[measured]**.
- No log exists for: a hover above 4 m, anything above 16 m, indoors, concrete, the barometer as height reference, the rangefinder invalid for more than 0.6 s with flow fused, or a PAW3902 with flow fused.

The mechanisms below therefore come from code. Their ranking is a hypothesis until step 0 is flown.

## How it works today

The node driver publishes one sample per chip frame. The node's `VehicleOpticalFlow` pairs it with a gyro integral and sends it over DroneCAN without a timestamp. The FC stamps it on receipt, re-accumulates to `SENS_FLOW_RATE` (70 Hz), and EKF2 fuses line-of-sight rate against `velocity / HAGL`, where HAGL is the terrain state plus altitude **[code]**.

With `EKF2_RNG_CTRL` 1, the default and what the ARK docs prescribe, the estimator has two structures **[code]**:

| Condition | Height reference | Range innovation corrects | Flow updates terrain | Terrain process noise |
|---|---|---|---|---|
| HAGL below `EKF2_RNG_A_HMAX` (5 m), speed below `EKF2_RNG_A_VMAX` (1 m/s), range healthy | Range | Height | No | Off |
| Higher or faster, range healthy | Barometer | Terrain only | Yes | On |
| Range unhealthy | Barometer | Nothing | Yes | On |

Other facts that shape the plan **[code]**:

- Flow noise is 0.36 to 0.50 rad/s for every realistic frame. That is about 1 m/s per sample at 2 m and 7 to 10 m/s at 20 m. There is no window, rotation-rate or height term.
- Flow is either fused at that weight or rejected. After 1 s without fusion flow stops, it cannot restart for 2 s, and 5 s after the last fusion (`EKF2_NOAID_TOUT`) position goes invalid and Position mode falls back to Altitude.
- Multicopter drag fusion is the only dead-reckoning aid. It is built on every ARK FC, off by default, and not counted as horizontal aiding.
- No position-controller gain or filter depends on HAGL. `vxy_max` is `4 × HAGL` m/s.
- The height ceiling `hagl_max_xy` exists but is 99 m at the defaults (`SENS_FLOW_MAXHGT` 100, `UAVCAN_RNG_MAX` 999), and it is dropped when HAGL goes invalid.
- The default log profile records `distance_sensor` and `sensor_optical_flow` at 1 Hz, so none of this is visible in a customer log.

## Symptom 1: hold degrades with height

| Mechanism | Evidence | Step |
|---|---|---|
| The estimator changes structure at 5 m (table above) | **[code]**. `cs_rng_hgt` dropped at 5.03 m in flight 6 **[measured]**. Whether it degrades the hold is untested. `boards/ark/pi6x` already sets `EKF2_RNG_A_HMAX` 25 | 0.3, 1 |
| A derotation residual (chip-to-gyro scale, timing, mount flex) is a false rate that HAGL turns into a false velocity, and the fixed-gain controller turns that into attitude. The loop gain of that path is proportional to HAGL | **[derived]**. ArduPilot detunes with height "to prevent body rate feedback into flow rates destabilising the control loop", and logged a 0.5 Hz, ±20° limit cycle at 50 m without it ([ArduPilot/ardupilot#33568](https://github.com/ArduPilot/ardupilot/pull/33568)) **[reported]**. PX4's docs tell users to reduce the flow scale factor above 10 to 20 m, which is a gain reduction by mis-scaling | 0.3, 4.1 |
| Flow weight falls as 1/HAGL, so the estimator coasts on the IMU between weak corrections | **[derived]** from the noise model | 0.3, 4.2 |
| A line-of-sight residual of 0.15 to 0.3 rad/s at 11 to 16 m that the gyro source, range source, timing and window length do not explain. The chip still reports 94 % of expected counts there | **[measured]**, one 16 s manoeuvring segment in flight 6. Cause unknown | 4.3 |
| Quantization at a fixed window | **[derived]**, and the same segment contradicts it as the dominant term: error does not fall with window length | Deprioritised |
| Above rangefinder reach, symptom 2 applies permanently. Daylight reach over asphalt is 13 to 21 m (LV85D) and 16 to 19 m (LX85D) | **[measured]** on ARK DIST boards; 15.2 m over grass on the Flow MR | 1 |

A heading error cannot cause toilet-bowling in flow-only flight, because measurement and control share the same yaw. A flow mounting-yaw error can, and so can HAGL-scaled rotation leakage **[derived]**. `SENS_FLOW_ROT` has 45° steps.

## Symptom 2: wrong HAGL

While range is fused, HAGL is right in every `EKF2_HGT_REF` and `EKF2_RNG_CTRL` combination **[code]**. Replacing EKF2's HAGL with the raw range changed velocity error by less than 0.1 m/s in two of three flights **[measured]**. The damage happens in the gaps:

| Mechanism | Evidence | Step |
|---|---|---|
| Range is invalid for 0.7 to 0.9 s at every lift-off and 1 to 7 s before every touchdown, plus a few 0.2 to 0.6 s runs in between: 3 to 9 % of airborne samples | **[measured]**, flights 1 to 6 | 2.4 |
| Each invalid frame costs `EKF2_RNG_QLTY_T` (1 s) of range health, and its innovation ends conditional range aid in the same step | **[code]** | 2.4 |
| During a gap the barometer is an unopposed reference (frozen bias, one-sided ground-effect deadzone, no thrust model), flow keeps fusing at full weight on that HAGL with no time limit, and flow moves the terrain state | **[code]**. Barometer error under thrust on an ARK FPV: +8 to +10 m at 1 m true height **[measured]** | 2.3 to 2.6 |
| At takeoff flow starts by resetting velocity to a filtered flow velocity scaled by a stale HAGL. The reset does not check HAGL freshness | bresch's trace of an indoor log in [PX4/PX4-Autopilot#24653](https://github.com/PX4/PX4-Autopilot/issues/24653) **[reported]**, **[code]** | 2.1 |
| With terrain invalid and flow unfused for 1 s, `resetTerrainToFlow()` sets HAGL to `EKF2_MIN_RNG` (0.01 m). That is below `SENS_FLOW_MINHGT`, so flow stops until range returns | **[code]** | 2.2 |

No log has been reduced to "HAGL was off by x %, so flow velocity was off by x %". Height steps under the vehicle (toilet-bowling over furniture, [PX4/PX4-Autopilot#27282](https://github.com/PX4/PX4-Autopilot/issues/27282)) are a separate path tracked in [PX4/PX4-Autopilot#27110](https://github.com/PX4/PX4-Autopilot/issues/27110).

## Symptom 3: low-texture surfaces

| Mechanism | Evidence | Step |
|---|---|---|
| Main fuses every frame with quality of at least 1 at nearly full weight. A zero-motion frame passes the 3σ gate | **[code]**. Flight 5: SQUAL median 61, 56 % of frames zero while moving, 94 % fused, flow-to-truth gain 0.3 **[measured]** | 3.1 |
| The PAA3905 challenging-surface flag only prints a warning | **[code]** | 3.1 |
| Low SQUAL under-reports motion. In daylight, frames between SQUAL 60 and 85 carried 50 to 80 % of true motion; at night in super-low-light mode the same band was noise | **[measured]**. Chan et al. 2010 found the same multiplicative bias on other mouse sensors **[reported]** | 0.5, 3.1 |
| Gating instead of fusing hands the vehicle to the 1 s, 2 s, 5 s timeline above with no dead-reckoning aid. With a floor of 85 over grass, flow was active 39 % of airborne time | **[code]**, **[measured]** with GNSS carrying the estimate. How fast velocity diverges in a blind window is unmeasured | 0.5, 3.2 |

## Plan

### Step 0: measure

No firmware changes. Every flight logs flow, range, estimator flags and aid sources at full rate, with RTK GNSS logged and not fused (`EKF2_GPS_CTRL` 0).

| # | Test | Decides |
|---|---|---|
| 0.1 | Desk: reduce the public logs in [#24653](https://github.com/PX4/PX4-Autopilot/issues/24653), [#27282](https://github.com/PX4/PX4-Autopilot/issues/27282) and [#26924](https://github.com/PX4/PX4-Autopilot/pull/26924), and `hover_with_throttle_punches.ulg` | Size of the HAGL error at lift-off, how long range is unfused, and whether flow velocity is off by the same fraction |
| 0.2 | Bench: test-vehicle barometer in sun and shade. It swung 100 to 450 m in both daylight flights | Prerequisite for any barometer-referenced daylight flight on that airframe |
| 0.3 | Height ladder: flow-only hold, 60 s hands-off at 2, 4, 6, 8, 10 and 15 m over a tracking surface. Repeat 6 and 15 m with (a) `EKF2_RNG_A_HMAX` raised, (b) `MPC_XY_P` and `MPC_XY_VEL_P_ACC` halved, (c) FC `SENS_FLOW_RATE` 10 | Where the hold degrades, and which symptom-1 mechanism dominates. A 0.2 to 1 Hz attitude-setpoint peak that shrinks with gain is the control loop; a slow walk that does not is the estimator |
| 0.4 | ARK FPV with an ARK Flow, barometer reference: five takeoffs to 1.5 m, then at 3 m cover the rangefinder for 20 s while stepping throttle | HAGL error and flow velocity scale during a range gap. First PAW3902 data |
| 0.5 | Node on a cart over indoor concrete at 1 and 2 m, both chips. Then a flow-only hold over an outdoor concrete slab with main, floor 60 and floor 85. Then EKF replay with flow masked, drag fusion off and on | Whether either chip tracks concrete and at what SQUAL, fuse versus gate, blind-window drift rate, and whether drag fusion bridges it |

### Step 1: defaults, docs and logging

- ARK Flow docs and board defaults: `SENS_FLOW_MAXHGT` and `UAVCAN_RNG_MAX` inside measured daylight reach, `EKF2_RNG_QLTY_T`, `SENS_FLOW_MAXR` on the MR page, and `EKF2_RNG_A_HMAX` if 0.3(a) moves the boundary.
- Remove the docs' advice to reduce the flow scale factor at height.
- Default log profile: flow, range and their aid sources at a rate that shows dropouts.
- Land the driver fixes in [#28632](https://github.com/PX4/PX4-Autopilot/pull/28632) that flight 4 verified (no reset on low SQUAL, no double reads, matched gyro window: 0 resets, 1 bad pair in 3555 reads). The SQUAL floor, `EKF2_OF_QMAX` and the noise defaults are unflown and wait for 0.5.

### Step 2: HAGL integrity

1. Reset velocity to flow only when HAGL comes from a recent range fusion.
2. Remove the `resetTerrainToFlow()` dead end when flow is the only horizontal aid.
3. Flow does not update the terrain state when it is the only horizontal aid. Height is unobservable from flow in a hover (Grabe 2015, de Croon 2016), and ArduPilot has never allowed it. Terrain validity then lapses about 1 s after range stops below 5 m, so this is designed together with item 4.
4. One range-gap policy: an invalid frame costs neither a second of range health nor conditional range aid; during a gap HAGL coasts from the last range without following barometer error; after a bound, HAGL and flow velocity are invalid and the failsafe is the defined one.
5. Ground effect: inflate barometer variance for both signs over a window with a minimum hold time, and freeze barometer bias learning inside it. Needs the logs from 0.1 and 0.4.
6. Barometer thrust compensation, [#27885](https://github.com/PX4/PX4-Autopilot/pull/27885). Runs in parallel and gates nothing here.

Exit: 0.4 repeated, with flow velocity scale error bounded through takeoff and through a 20 s range gap.

### Step 3: low-texture surfaces

1. Weighting from 0.5 data: a noise ramp that spans the SQUAL range that occurs, the challenging-surface flag as an input, and a per-chip, per-mode floor if one floor does not fit.
2. Blind-window behaviour: drag fusion learns wind and accelerometer bias while flow is good and counts as aiding for a bounded time when flow is blind. Published IMU-plus-drag velocity error is about 0.6 m/s RMS indoors (Leishman 2014), so this bounds velocity; it does not hold position.
3. Offline experiment on the raw captures: correct the under-report by `1 / P(SQUAL)`. Only worth building if `P` is stable across modes, which the day and night data so far say it is not.

Exit: 0.5 repeated, with hold error and time to failsafe stated for a trackable and an untrackable surface.

### Step 4: height

1. Height-scaled horizontal gain when flow is the only aid, applied to both the position and velocity loops, with a configurable knee. ArduPilot's fixed 4 m knee left 9 % gain at 45 m and lost position in wind ([ArduPilot/ardupilot#33569](https://github.com/ArduPilot/ardupilot/pull/33569)). Build only if 0.3(b) shows a gain-dependent peak.
2. A longer accumulation window, if 0.3(c) helps. ArduPilot fuses 100 ms windows at every height.
3. Desk: name the term behind the 0.15 to 0.3 rad/s residual at 12 m before changing the noise model. The measured lag-1 error autocorrelation of +0.8 is a slowly varying error, not quantization, which would be negative.
4. Chip-to-gyro scale and mounting yaw from an offline log fit (`compare_raw_flow.py` already produces both), and a finer alignment parameter than 45° steps.
5. Speed limit with a rotation margin and a higher floor, the direction review gave [#26786](https://github.com/PX4/PX4-Autopilot/pull/26786). Height ceiling kept when HAGL is invalid.

Exit: 0.3 repeated, meeting the acceptance table.

## Decisions needed

1. The acceptance table, in particular a supported ceiling at rangefinder reach instead of 20 m.
2. Splitting [#28632](https://github.com/PX4/PX4-Autopilot/pull/28632) into verified driver fixes and the unflown gate and noise defaults.
3. Ordering against [#28000](https://github.com/PX4/PX4-Autopilot/pull/28000), which rewrites `optical_flow_control.cpp` and renames every `EKF2_OF_*` parameter. Steps 2.1 to 2.3 are small enough to land first; new parameters should follow it.
4. Whether to coordinate with andyp1per, who is reworking the same area in ArduPilot on an ARK Flow (seven open PRs, range unusable for 25 to 79 % of his flights).
5. Which airframe flies step 0, given the test vehicle's barometer.

## Upstream constraints

- Noise-default changes need flight results. [#25365](https://github.com/PX4/PX4-Autopilot/pull/25365) lowered flow noise, was approved pending flights, and died without them. [#28632](https://github.com/PX4/PX4-Autopilot/pull/28632) raises it.
- Per-sensor logic belongs in the sensors layer (bresch on [#28000](https://github.com/PX4/PX4-Autopilot/pull/28000) and [#26924](https://github.com/PX4/PX4-Autopilot/pull/26924)). That rules out EKF2 summing flow itself.
- The terrain state is the intended mechanism. bresch answers HAGL problems with terrain process noise ([#25258](https://github.com/PX4/PX4-Autopilot/issues/25258)); a second height filter has no sponsor.
- Ground-effect changes need a log (dagar on [#26655](https://github.com/PX4/PX4-Autopilot/pull/26655)).
- Thrust and airspeed barometer compensation must be fitted together (bresch on [#26924](https://github.com/PX4/PX4-Autopilot/pull/26924)), so [#27885](https://github.com/PX4/PX4-Autopilot/pull/27885) is not close.
- Gain scheduling has not been raised with a control maintainer.

## Not doing

- Estimating height or flow scale from flow in a hover. It is unobservable without acceleration.
- A separate AGL filter as ArduPilot built it ([ArduPilot/ardupilot#32389](https://github.com/ArduPilot/ardupilot/pull/32389)). ArduPilot needed it because its terrain filter lags by seconds; EKF2's does not. Its behaviour during range gaps is what step 2.4 copies.
- ArduPilot's FlowHold mode, in-flight scale calibration (needs GPS), per-axis innovation gating, and flow noise model (constant 0.25 rad/s, `quality > 0`).
- Per-mode SQUAL normalization ([#28625](https://github.com/PX4/PX4-Autopilot/pull/28625), on main today). The tracking knee sits at the same raw SQUAL in every mode.
- A DroneCAN flow timestamp for now. Timing shifts of ±20 ms changed nothing in the reconstructions **[measured]**.
- Drag-only position hold, and innovation-adaptive noise as the texture defence. With flow as the only aid the innovation cannot tell a bad sensor from a bad prediction.

## Corrections to the previous draft

- "Rangefinder invalid on 22 to 31 % of airborne samples at 1.4 to 4.5 m" counted ground time. Airborne it is 3 to 9 %, at lift-off and touchdown.
- "The terrain state keeps its last value" when range is not fused. Flow updates it.
- "Without GNSS the barometer is the height reference." Not in a hover below 5 m under conditional range aid.
- The 13 to 21 m rangefinder reach is the LV85D on an ARK DIST, not the Flow MR.
- The node gyro's 1 rad/s was the RawIMU stream. The flow compensation integral read 0.155 rad/s, and the gyro source does not change velocity error.
- The floor-60 rebuild's "97 % of windows at 1.1 m/s" is 55 % of frame time at a gain of about 0.6.
- "Everything except floor 60 has flown." `EKF2_OF_QMAX` and the new noise defaults have not.
- The kinematic consistency check cannot trip at 15 to 20 Hz range rates; its 1σ is 3 to 14 m/s.
- ArduPilot: [#33569](https://github.com/ArduPilot/ardupilot/pull/33569) exists to raise the gain knee, [#28982](https://github.com/ArduPilot/ardupilot/pull/28982) is raw throttle on one barometer and compiled out by default, and [#32472](https://github.com/ArduPilot/ardupilot/pull/32472) uses EKF HAGL with a minimum hold, not measured range with a timeout.
- `performance_vs_altitude.md` misreads `hover_with_throttle_punches.ulg`: the vehicle was at 0.2 to 1.2 m by range while the barometer read +8 to +10 m.
