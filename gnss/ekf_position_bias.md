# GNSS: coloured position error and EKF position bias states

Status: **idea, prior art reviewed**, written 2026-09-24 against PX4 main `816a234175`, with ArduPilot master `4965c4e84a` for comparison. The numbers in §3 come from a 1D simulation ([`ekf_position_bias_sim.py`](ekf_position_bias_sim.py)), not from flight logs.

Goal in one line: fuse GNSS at 10–20 Hz for the velocity gain without EKF2 claiming a position accuracy the receiver doesn't have.

Related:

- [`selection_fusion_and_heading.md`](selection_fusion_and_heading.md) §6.3.2: the same independence violation, across co-located receivers instead of across time.
- [`pipeline_migration.md`](pipeline_migration.md) decision 5: a receiver switch resets position. Bias states have to reset with it.

## 1. Today

- **Position R** = max(eph, `EKF2_GPS_P_NOISE`)² per axis (`aid_sources/gnss/gps_control.cpp:358`). **Velocity R** = max(sacc, `EKF2_GPS_V_NOISE`)², vertical ×1.5² (`gps_control.cpp:320`). Both parameters are floors, not scale factors. Nothing depends on rate.
- **Every sample is fused.** The minimum observation spacing is the delay horizon divided by the buffer length (`estimator_interface.cpp:117`), a few ms.
- **ArduPilot EKF3 uses the same model**: R from hAcc/sAcc through a peak-hold with a 5 s decay, floored by `EK3_POSNE_M_NSE`/`EK3_VELNE_M_NSE`, no rate term. It drops a sample that arrives within 50 ms of the last accepted one, so a 20 Hz u-blox fuses at 10 Hz.

A Kalman filter assumes each sample's error is independent of the last. GNSS position error is mostly wander with correlation times of tens of seconds or longer (ionosphere, troposphere, multipath, the receiver's own navigation filter). eph describes how large that wander is, not how much new information each sample carries. Doubling the rate halves the noise density the filter believes (R·Δt) while the real error stays the same, so the position covariance collapses. Doppler velocity error is closer to white, so a higher rate genuinely improves velocity.

## 2. Prior art

**PX4 and ArduPilot.** No filter in the lineage (InertialNav, EKF1–3, ECL, EKF2) has had GNSS position bias states, and nobody has proposed or rejected them in either project's issues, PRs or docs. Riseborough saw the symptom and treated it as conditioning and tuning:

- [ArduPilot#16791](https://github.com/ArduPilot/ardupilot/issues/16791#issuecomment-791074055) (2021): "The EKF is becoming over confident about its accuracy and the state variances collapse … 2) Faster GPS update rates, eg 10Hz 3) Use of RTK GPS data with small reported error values … running the GPS updates at 5Hz would help. It may be necessary with 10Hz operation to limit the observation accuracy." The fix, [#16842](https://github.com/ArduPilot/ardupilot/pull/16842), added variance floors. [PX4-ECL#771](https://github.com/PX4/PX4-ECL/issues/771) (2020) is the same pattern.
- The `*_M_NSE` floors exist "to protect against receivers that are overly optimistic" ([ArduPilot#4450](https://github.com/ArduPilot/ardupilot/issues/4450#issuecomment-329713493)), not for correlation. ArduPilot's 5 Hz default is about receiver performance at 10 Hz ([7a82898e92](https://github.com/ArduPilot/ardupilot/commit/7a82898e92), [#15893](https://github.com/ArduPilot/ardupilot/pull/15893)).
- EKF2's `BiasEstimator`s ([PX4#17944](https://github.com/PX4/PX4-Autopilot/pull/17944), [#19944](https://github.com/PX4/PX4-Autopilot/pull/19944), [#20501](https://github.com/PX4/PX4-Autopilot/pull/20501)) assume the reference sensor has no bias, and GNSS is the horizontal reference. SIH GNSS became Gauss-Markov in [PX4#26697](https://github.com/PX4/PX4-Autopilot/pull/26697); the EKF still assumes white.

**Literature.** Groves (*Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed.):

- §3.4.3: "The optimal solution is to estimate the time-correlated noise as additional Kalman filter states. However, this may not be practical due to observability or processing capacity limitations." The simplest alternative reduces the gain: longer update interval, larger R, or down-weighted K, at the cost of slower convergence.
- §14.1.2: GNSS solution errors correlate "up to 100 seconds on the position and 20 seconds on the velocity", and "Measurement-update intervals of 10 seconds are common in loosely coupled systems."
- §14.3.1: R should be "the variance of the position or velocity error multiplied by the ratio of the error correlation time to the measurement update interval", i.e. R = σ²·τ/Δt.
- Augmenting purely coloured noise (no white part) makes P ill-conditioned (Gelb §4.5; Maybeck vol. 1 §5.10; Bryson & Henrikson 1968). A white component in R avoids that.
- A wrong τ can leave the filter overconfident; size τ and σ to overbound, not to best-fit ([García Crespillo, Joerger & Langel](https://arxiv.org/abs/2009.09495)). Real receiver errors mix several time scales, e.g. about 30 s plus about an hour ([Niu et al. 2014](https://doi.org/10.1007/s10291-013-0324-x)). A loosely coupled field test with augmented states improved consistency ([Niu, Wu & Zhang 2018](https://doi.org/10.1134/S2075108718040053)).

**Practice.** No production loosely coupled estimator found carries GNSS position bias states. GNSS error states live in tightly coupled filters (clock, atmosphere, per-satellite pseudorange biases). Loosely coupled systems reduce the gain instead:

- NovAtel SPAN, stationary: "the GPS position is de-weighted nine out of ten times … This deweighting prevents an inordinate reduction of the state variances as a result of the correlated measurement errors" ([US7193559](https://patents.google.com/patent/US7193559B2/en)).
- Applanix POS processes GNSS position "typically at 1 Hz"; one patent lists GPS NED position errors as possible states ([US6834234](https://patents.google.com/patent/US6834234B2/en)).
- openpilot sets R = (10 × u-blox accuracy)²; robot_localization, KF-GINS, dRonin, INAV, Betaflight and Paparazzi carry no GNSS bias.

## 3. Simulation

1D INS/GNSS, IMU prediction at 100 Hz with IMU noise at EKF2's assumed level, R set from the reported accuracies as EKF2 does. Truth: position error is Gauss-Markov with σ 1 m and τ 60 s, plus 0.2 m white (eph 1.02 m). Velocity error is either white 0.1 m/s, or correlated: Gauss-Markov 0.05 m/s with τ 20 s plus 0.05 m/s white (sacc 0.07 m/s). 200 runs of 900 s, first 300 s discarded. "×" is the actual RMS error divided by the σ the filter reports. Gate σ is the σ the position gate tests against; NIS is the mean normalized innovation squared against it, 1 when consistent.

**White velocity error:**

| Filter | Rate | Pos RMS | Pos σ | × | Vel RMS | Gate σ | NIS |
|---|---|---|---|---|---|---|---|
| Today | 5 Hz | 0.94 m | 0.14 m | 6.6 | 0.039 m/s | 1.03 m | 0.18 |
| Today | 10 Hz | 0.92 m | 0.10 m | 9.1 | 0.033 m/s | 1.02 m | 0.18 |
| Today | 20 Hz | 0.90 m | 0.07 m | 12.6 | 0.028 m/s | 1.02 m | 0.18 |
| R × rate/5 Hz | 10 Hz | 0.88 m | 0.12 m | 7.3 | 0.033 m/s | 1.45 m | 0.12 |
| R × 100 | 10 Hz | 0.64 m | 0.32 m | 2.0 | 0.033 m/s | 10.20 m | 0.01 |
| Position R × τ/Δt | 10 Hz | 0.55 m | 0.51 m | 1.1 | 0.033 m/s | 24.99 m | 0.00 |
| Position R × τ/Δt, gate on eph² | 10 Hz | 0.57 m | 0.51 m | 1.1 | 0.033 m/s | 1.14 m | 0.78 |
| Position bias states | 10 Hz | 0.54 m | 0.55 m | 1.0 | 0.033 m/s | 0.23 m | 1.00 |
| Position bias, τ 20 s, σ 1.5 m | 10 Hz | 0.54 m | 0.54 m | 1.0 | 0.033 m/s | 0.29 m | 0.70 |
| Position bias, τ 300 s, σ 0.7 m | 10 Hz | 0.60 m | 0.57 m | 1.1 | 0.034 m/s | 0.21 m | 1.36 |

**Correlated velocity error:**

| Filter | Rate | Pos RMS | Pos σ | × | Vel RMS | Vel × | Gate σ | NIS |
|---|---|---|---|---|---|---|---|---|
| Today | 10 Hz | 1.03 m | 0.08 m | 12.2 | 0.054 m/s | 1.9 | 1.02 m | 0.52 |
| Position R × τ/Δt, gate on eph² | 10 Hz | 3.62 m | 0.44 m | 8.3 | 0.055 m/s | 2.0 | 1.11 m | 11.19 |
| Position and velocity R × τ/Δt, gate on eph² | 10 Hz | 1.21 m | 1.55 m | 0.8 | 0.083 m/s | 0.8 | 1.86 m | 0.30 |
| Position bias states | 10 Hz | 3.58 m | 0.49 m | 7.3 | 0.054 m/s | 1.9 | 0.23 m | 1.02 |
| Position and velocity bias states | 10 Hz | 0.95 m | 0.95 m | 1.0 | 0.045 m/s | 1.0 | 0.23 m | 1.00 |

- Today's model is already about 6× overconfident at 5 Hz; each doubling of the rate adds √2. Its gate is too wide as well (NIS 0.18 with white velocity): the default 5σ gate accepts a step of about 5 m, over 11× the actual innovation σ.
- Fixing position alone is only safe if velocity error is white. Both position-only fixes make the filter hold position on integrated velocity for minutes; with correlated velocity error that makes position 3.5× worse than today. Position-only bias states even show NIS ≈ 1 while 7× overconfident, so NIS can't catch the missing velocity model.
- Groves' R × τ/Δt applied to both position and velocity makes both covariances honest (slightly pessimistic) with no new states. It costs accuracy: position 1.21 m against today's 1.03 m, velocity 0.083 m/s against 0.054 m/s. Gating on today's P + eph² keeps the position gate usable, though loose (NIS 0.30).
- Position and velocity bias states are the only option better than today on every column: honest, consistent, and more accurate in both position and velocity.
- Scaling R by rate relative to a fixed rate, or by a fixed factor, doesn't reach consistency. The factor has to be τ/Δt.
- Position RMS below the raw wander (white velocity cases) comes from averaging GNSS over many τ while holding position on velocity. It relies on white velocity error and zero-mean Gauss-Markov position error, so don't expect it in flight.
- The 1D model has no tilt or accelerometer-bias states, and its IMU noise equals the filter's assumed noise. A real IMU that is better than `EKF2_ACC_NOISE` makes inflating velocity R cheaper.

## 4. Options in EKF2

### R from τ, gate from eph

Fuse GNSS position with R = max(eph, `EKF2_GPS_P_NOISE`)²·max(1, τ_p/Δt) and GNSS velocity with R = max(sacc, `EKF2_GPS_V_NOISE`)²·max(1, τ_v/Δt). Compute the position test ratio from P + max(eph, `EKF2_GPS_P_NOISE`)². Two parameters, no new states, a few lines in `gps_control.cpp`. This is the textbook loosely coupled answer (Groves §14.3.1) with the gate kept usable. R follows eph instantly, so an RTK Float → Fixed transition snaps as it does today. It must scale both: position alone is worse than today when velocity error is correlated.

### Bias states

Position and velocity bias states, north and east, four in all. Position alone is worse than today when velocity error is correlated.

- **State:** add `gnss_pos_bias = sf.V2()` and `gnss_vel_bias = sf.V2()` to `State` in `src/modules/ekf2/EKF/python/ekf_derivation/derivation.py`. SymForce regenerates `predict_covariance.h` and `state.h`. Dynamics b ← e^(−Δt/τ)·b with process noise σ_b²(1 − e^(−2Δt/τ)). Build them out behind a Kconfig option the way `EKF2_WIND` drops `wind_vel` (`--disable_wind`).
- **Fusion:** GNSS position and velocity are direct state updates today (`fuseDirectStateMeasurement`). With a bias, H has a 1 on the state and on its bias, so fusion moves to `measurementUpdate(K, H, R, innov)` (`ekf.h:288`), and R becomes the white part only.
- **Scope:** only GNSS position and velocity fusion see the biases. External vision, auxiliary global position, position resets and GNSS height don't.
- **Not the existing `BiasEstimator`s.** The GNSS height, EV position and baro bias estimators are separate one-state random-walk filters that track a secondary sensor's offset from the EKF. They aren't coupled to the EKF covariance, and they go inactive when their sensor is the reference. Primary GNSS has no bias model.

Over R from τ, bias states buy accuracy, a consistent and tighter gate, and NIS as a tuning signal, for four states per instance and the handling in §5.

## 5. Open problems

1. **τ_v is the key unknown.** Whether real GNSS velocity error is close to white decides whether position-only fixes are safe and how much velocity R scaling costs. Measure τ_p and τ_v from static logs of F9P, X20 and mosaic receivers before choosing defaults, sized to overbound.
2. **Splitting eph and sacc.** A receiver reports one total σ each. Bias states need a white R (a parameter, or a fraction of the reported accuracy) and a bias variance of the rest. Keep the white part non-zero to avoid an ill-conditioned P.
3. **Solution-type changes.** A new solution type is a new error process. On RTK Fixed ↔ Float ↔ 3D, reset the bias states to zero with the new variance, as on a receiver switch. Letting the old bias decay over τ would hold position off a newly fixed RTK solution for up to a minute.
4. **Slower response to GNSS steps.** A 2 m GNSS step without an eph change reaches 63% in 24 s with position and velocity R scaled by τ/Δt, against 14 s today. Scaling position alone takes over 5 minutes at 10 Hz. Right when the step is error, slow when eph lags a real change.
5. **Resets.** A bias belongs to one receiver. Reset the bias states and their covariance when `selection_count` changes. A blended output's error shifts whenever the blend weights change, which neither option can represent, so selection should replace blending first.
6. **Gate tightening.** A consistent gate with bias states is about 4× narrower in metres. A receiver that steps its position without raising eph in the same sample gets rejected where today it's accepted.
7. **Height.** A vertical bias state would interact with `_gps_hgt_b_est`. Start with horizontal only.
