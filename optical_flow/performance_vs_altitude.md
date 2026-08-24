# Optical flow: performance vs altitude (PAW3902)

Status: **investigation notes**. Code claims verified against `ARK-Electronics/private_px4` `release_ark` @ `32ab5eb71f` and `PX4/PX4-Autopilot` main @ `6f912dcd8c` on 2026-08-24. Sensor facts from PAW3902JF-TXQT datasheet R1.10. Flight evidence is thin: only one local log with PAW3902 data (`hover_with_throttle_punches.ulg`, Bright mode, flow not fused) — used for SQUAL range only.

Goal in one line: keep position hold on an ARK Flow usable at 20 m, where users report drift and "rolling" oscillation, and know which of the levers are software, tuning, and physics.

Related PX4 work:

| Item | State | What it is |
|---|---|---|
| [PR #27418](https://github.com/PX4/PX4-Autopilot/pull/27418) | merged upstream, backported to `release_ark` | report actual integration timespan in PAW3902/PAA3905 — mine |
| `cac770e333` | merged upstream | HAGL validity check for range height ref (`isHeightAboveGroundEstimateValid()`) |
| [PR #26960](https://github.com/PX4/PX4-Autopilot/pull/26960) | merged | allow flow to start when range finder is height reference — mine |
| [PR #27963](https://github.com/PX4/PX4-Autopilot/pull/27963) | see `rangefinder/` notes | AFBR-S50 range investigation — mine |

---

## 1. Sensor facts that bound the problem

| Fact | Value | Source |
|---|---|---|
| Angular resolution | 1 count = 1/(11.914 × 39.37) rad = **2.13 mrad (0.122°)** | Fig 19 `y = 11.914x⁻¹` CPI vs height; `PAW3902.cpp:415-419` |
| Resolution register | `0x4E` = `0xA8` = max, written by every `ConfigureMode*()` | `(0xA8+1)·50/8450 = 1.0` → Fig 19 is the max-resolution curve. No headroom. |
| Field of view | 42° → ground patch 0.77·h wide | 1.5 m at 2 m, 15 m at 20 m |
| Frame rate | 126 fps (modes 0, 1), 50 fps (mode 2) | `SAMPLE_INTERVAL_MODE_*` |
| Max rate | 7.4 rad/s (modes 0, 1) | Fig 20: `v_max = 7.7554·h` m/s |
| SQUAL | number of valid features / 4, max 255; false-motion floors 0x19 / 0x46 / 0x55 per mode | §7.2 step 5, reg 0x07 |
| Accumulation | delta_x/y accumulate in the chip until `Motion_Burst` is read | §7.1 |

Ground displacement per count is 2.13 mm·h: 4 mm at 2 m, 4.3 cm at 20 m. That is the "fixed resolution" — fixed in angle, not on the ground. Only a narrower lens or a different sensor changes it.

---

## 2. Every error term is in rad/s, and the EKF multiplies by HAGL

`fuseOptFlow` predicts LOS rate = v_body / HAGL (`optical_flow_fusion.cpp:139-149`), so an error of ε rad/s in the compensated flow is a velocity error of ε·h. Four independent terms:

| Term | rad/s | at 2 m | at 20 m |
|---|---|---|---|
| Quantization per 7.94 ms frame: LSB 0.269, σ = LSB/√12 | 0.078 | 0.16 m/s | **1.6 m/s** |
| EKF observation noise with SQUAL 90–130, `EKF2_OF_QMIN`=1 → weighting 0.35–0.51 → blend of 0.15 / 0.5 | 0.32–0.38 | 0.7 m/s | **6.5–7.5 m/s** |
| Gyro-compensation residual (bias, window misalignment, delay) | whatever it is | ×2 | ×20 |
| HAGL scale error δh/h | multiplicative | — | 2 m baro drift = 10 % velocity scale |

Signal for comparison: a 0.3 m/s hover drift at 20 m is 0.015 rad/s = 7 counts/s = **one count every ~18 frames (143 ms)**. At 2 m the same drift is a count every second frame.

So at 20 m each sample is a near-binary measurement (0 or 1 count) that the EKF is told has ~7 m/s noise. The filter degenerates to accelerometer dead-reckoning with sparse, lumpy corrections: slow drift, then an overcorrection when a count lands — the "rolling" users describe. Nothing in that loop is a bug; it is the noise model and the integration window being chosen for 2 m and never revisited.

---

## 3. What the code does today

### Driver (`src/drivers/optical_flow/paw3902/PAW3902.cpp`)

- Publishes every read that reports motion, plus zero-flow samples on the backup schedule so the FC sees a continuous stream (`:433-445`, `:458-472`).
- `integration_timespan_us` is the real interval between burst reads (#27418). Backup read when the MOTION pin is quiet is `kBackupScheduleIntervalUs` = 20 ms (`PAW3902.hpp:132`), so at altitude most samples are 8–20 ms windows.
- `quality` = raw SQUAL (`:424`, `:442`), no per-mode normalization. The only per-mode logic is the datasheet false-motion discard (`:304`, `:331`, `:371`), which needs *both* low SQUAL and saturated shutter — in daylight the shutter is short, so low-SQUAL frames at altitude pass through untouched.
- Mode switch = `Reset()` → soft reset + reconfigure + 3 discarded frames ≈ 50 ms gap, which trips `VehicleOpticalFlow`'s gap-clear (`VehicleOpticalFlow.cpp:115`).

### CAN path

- Node: `SENS_FLOW_RATE 150` (`boards/ark/can-flow/init/rc.board_sensors:7`) → `VehicleOpticalFlow` forwards essentially every frame; `FlowMeasurement.hpp:78-87` sends the node's own gyro integral and mean SQUAL.
- FC: `uavcan/sensors/flow.cpp:62` stamps `timestamp_sample = hrt_absolute_time()` at receipt (marked `TODO`), so CAN + node latency lands in `EKF2_OF_DELAY` (default 20 ms). FC re-accumulates at its own `SENS_FLOW_RATE` (default 70 Hz, `sensor_params_flow.c:113`).
- `EKF2_OF_GYR_SRC=0` (Auto) uses the node's gyro integral when finite (`optical_flow_control.cpp:63-73`) — good: gyro window and flow window are built on the same MCU.

### EKF noise model (`optical_flow_fusion.cpp:164-183`)

```
weighting = clamp((quality − QMIN) / (255 − QMIN), 0, 1)
R_LOS     = (N_MIN·weighting + N_MAX·(1 − weighting))²
```

Defaults `N_MIN 0.15`, `N_MAX 0.5`, `QMIN 1`. `R_LOS` has no dependence on integration time or HAGL. The quality gate is `quality >= QMIN` (`optical_flow_control.cpp:86`), i.e. everything ≥ 1 is fused. `pi6x` overrides `N_MIN 0.05` (`rc.board_defaults:37`); `can-flow` users get stock defaults.

### HAGL

- `predictFlowHagl()` (`optical_flow_fusion.cpp:118-134`) uses the terrain state, which the rangefinder feeds as long as it returns valid data. The AFBR-S50 on ARK Flow measures to 30 m; the driver's per-module cap (`AFBRS50.cpp:172`, 10 m for MV85G) and the FC's `UAVCAN_RNG_MAX` need to say 30 so the EKF actually gets it at 20 m.
- If the rangefinder does drop out, terrain propagates with `EKF2_TERR_NOISE` 5 m/s and, when the height reference is not RANGE, flow is allowed to update terrain (`opt_flow_terrain`, `optical_flow_control.cpp:160`). Velocity and HAGL are then both estimated from the same near-binary measurement and the scale wanders. With 30 m of range this is a fallback path, not the normal one.
- `SENS_FLOW_MAXHGT` default 100 m; ARK docs recommend 25 m — must cover the operating altitude.

---

## 4. Evidence

- `hover_with_throttle_punches.ulg` (indoor rig, up to 11 m, rangefinder reading 0 m, flow never fused): Bright mode throughout, SQUAL 86–97 near the ground, 120–152 above 6 m. With `QMIN=1` that is weighting 0.35–0.6, i.e. a frame with 500 tracked features is treated as half quality and given `R_LOS ≈ 0.32 rad/s`.
- No log on hand with Low-Light or Super-Low-Light mode, and none with flow fused above 10 m. Both are needed before tuning numbers in §5.2 and §5.3.

---

## 5. Levers, ranked

### 5.1 Integrate longer when high (largest, software only)

Quantization σ ∝ 1/T_int. Replace the fixed `SENS_FLOW_RATE` accumulation in `VehicleOpticalFlow` with a target ground displacement per sample: `T_int = clamp(k·HAGL, T_frame, ~100 ms)`, keeping 8 ms at 2 m and reaching ~80 ms at 20 m.

| T_int | LSB rad/s | σ rad/s | σ_v at 20 m |
|---|---|---|---|
| 7.9 ms | 0.269 | 0.078 | 1.6 m/s |
| 50 ms | 0.043 | 0.012 | 0.25 m/s |
| 100 ms | 0.021 | 0.006 | 0.12 m/s |

EKF already timestamps at the integration midpoint (`EKF2.cpp:2356`), so latency stays honest. Cost: a 20 Hz update at 20 m, which the position loop tolerates at hover.

The cleaner version puts quantization into the noise model instead of relying on retuning: `R = R_qual² + (σ_count / dt)²` with `σ_count = 2.13e-3/√12` rad. That needs the angular resolution on the sample (new field in `sensor_optical_flow`, or an `EKF2_OF_RES` param) — then any integration rate self-tunes.

### 5.2 Renormalize SQUAL per mode, then set a real QMIN

Driver: `quality = 255 · clamp((SQUAL − floor_mode) / (ceil_mode − floor_mode), 0, 1)` with `floor_mode` = datasheet false-motion floor (0x19 / 0x46 / 0x55) and `ceil_mode` empirical. FC: `EKF2_OF_QMIN` ≈ 40 so weighting spans the real range instead of 1–255. Needs SQUAL histograms per mode from flight logs (see §6). At altitude SQUAL genuinely drops (ground pixel 0.4 m at 20 m — grass and asphalt go featureless), so this is also what stops featureless frames from being fused at all.

### 5.3 Make sure the EKF is actually getting the rangefinder at 20 m

Raise the driver cap / `UAVCAN_RNG_MAX` to 30 m and confirm in a 20 m log that `estimator_status_flags.cs_rng_terrain` stays set and `distance_sensor.signal_quality` holds. A 2 m HAGL error is a 10 % velocity scale error at 20 m, and a dropout hands scale estimation to the flow itself (§3). Lowering `EKF2_TERR_NOISE` is the cheap guard if dropouts still happen.

### 5.4 Measure and set `EKF2_OF_DELAY` for the CAN path

Cross-correlate the node's `delta_angle` (in `vehicle_optical_flow`) with the FC gyro to get the true latency; default 20 ms was set for SPI-attached sensors. A delay error τ during an attitude correction of angular acceleration α is α·τ rad/s → ×20 at 20 m, and it is in the loop the position controller closes, which is the mechanism that turns drift into rolling.

### 5.5 Verify the chip carries sub-count residual

Everything in §5.1 assumes the DSP keeps fractional motion between frames. If it truncates below one count per frame, slow drift at altitude is invisible and only a narrower lens helps. Turntable at ~0.01 rad/s for 60 s: expect ≈ 280 counts total.

---

## 6. Next

1. Get one outdoor log at 20 m with `sensor_optical_flow`, `distance_sensor`, `estimator_aid_src_optical_flow`, `estimator_status_flags` at full rate. Bin SQUAL and `signal_quality` by altitude; confirm the rangefinder is fused there (§5.3) before touching tuning.
2. Prototype §5.1 as HAGL-adaptive `SENS_FLOW_RATE` in `VehicleOpticalFlow`; fly 2 m / 10 m / 20 m with stock vs adaptive; compare `estimator_aid_src_optical_flow.test_ratio` and position-hold RMS.
3. Collect SQUAL histograms in Low-Light and Super-Low-Light (dusk flight) for §5.2 ceilings.
4. Bench test §5.5.
