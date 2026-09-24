# GNSS: coloured position error and EKF position bias states

Status: **idea**, written 2026-09-24 against PX4 main `816a234175`, with ArduPilot master `4965c4e84a` for comparison. The numbers in §2 come from a 1D simulation ([`ekf_position_bias_sim.py`](ekf_position_bias_sim.py)), not from flight logs.

Goal in one line: fuse GNSS at 10–20 Hz for the velocity gain without EKF2 claiming a position accuracy the receiver doesn't have.

Related:

- [`selection_fusion_and_heading.md`](selection_fusion_and_heading.md) §6.3.2: the same independence violation, across co-located receivers instead of across time.
- [`pipeline_migration.md`](pipeline_migration.md) decision 5: a receiver switch resets position. Bias states have to reset with it.

## 1. Today

- **Position R** = max(eph, `EKF2_GPS_P_NOISE`)² per axis (`aid_sources/gnss/gps_control.cpp:358`). **Velocity R** = max(sacc, `EKF2_GPS_V_NOISE`)², vertical ×1.5² (`gps_control.cpp:320`). Both parameters are floors, not scale factors. Nothing depends on rate.
- **Every sample is fused.** The minimum observation spacing is the delay horizon divided by the buffer length (`estimator_interface.cpp:117`), a few ms.
- **ArduPilot EKF3 uses the same model**: R from hAcc/sAcc, floored by `EK3_POSNE_M_NSE`/`EK3_VELNE_M_NSE`, no rate term. It drops a sample that arrives within 50 ms of the last accepted one, so a 20 Hz u-blox fuses at 10 Hz.

A Kalman filter assumes each sample's error is independent of the last. GNSS position error is mostly wander with correlation times of tens of seconds (ionosphere, troposphere, multipath, the receiver's own navigation filter). eph describes how large that wander is, not how much new information each sample carries. Doubling the rate halves the noise density the filter believes (R·Δt) while the real error stays the same, so the position covariance collapses. Doppler velocity error is much closer to white, so a higher rate genuinely improves velocity.

## 2. Simulation

1D INS/GNSS, IMU prediction at 100 Hz, R set from the reported accuracies as EKF2 does. Truth: position error is Gauss-Markov with σ 1 m and τ 60 s, plus 0.2 m white (eph 1.02 m); velocity error is white, 0.1 m/s. 200 runs of 900 s, first 300 s discarded. "×" is the actual RMS error divided by the σ the filter reports. Innovation σ is the predicted GNSS position innovation σ; NIS is the mean normalized innovation squared, 1 when the noise model is right.

| Position model | Rate | Pos RMS | Pos σ | × | Vel RMS | Innov σ | NIS |
|---|---|---|---|---|---|---|---|
| Today | 5 Hz | 0.91 m | 0.14 m | 6.4 | 0.039 m/s | 1.03 m | 0.18 |
| Today | 10 Hz | 0.92 m | 0.10 m | 9.2 | 0.033 m/s | 1.02 m | 0.18 |
| Today | 20 Hz | 0.92 m | 0.07 m | 12.9 | 0.028 m/s | 1.02 m | 0.18 |
| R × rate/5 Hz | 10 Hz | 0.91 m | 0.12 m | 7.6 | 0.033 m/s | 1.45 m | 0.11 |
| R × rate/5 Hz | 20 Hz | 0.86 m | 0.10 m | 8.5 | 0.028 m/s | 2.04 m | 0.07 |
| R × 10 | 10 Hz | 0.83 m | 0.18 m | 4.6 | 0.033 m/s | 3.23 m | 0.04 |
| R × 100 | 10 Hz | 0.63 m | 0.32 m | 2.0 | 0.033 m/s | 10.20 m | 0.01 |
| Bias states, true τ and σ | 5 Hz | 0.62 m | 0.62 m | 1.0 | 0.039 m/s | 0.25 m | 1.00 |
| Bias states, true τ and σ | 10 Hz | 0.53 m | 0.55 m | 1.0 | 0.033 m/s | 0.23 m | 1.00 |
| Bias states, true τ and σ | 20 Hz | 0.47 m | 0.49 m | 1.0 | 0.028 m/s | 0.22 m | 1.00 |
| Bias states, τ 20 s, σ 1.5 m | 10 Hz | 0.56 m | 0.54 m | 1.0 | 0.033 m/s | 0.29 m | 0.70 |
| Bias states, τ 300 s, σ 0.7 m | 10 Hz | 0.63 m | 0.57 m | 1.1 | 0.034 m/s | 0.21 m | 1.36 |

- Velocity accuracy depends on the rate, not on the position model. The 1D model has no tilt or accelerometer-bias states, so whether over-trusted position wander leaks into those is untested.
- Today's model is already about 6× overconfident at 5 Hz; each doubling of the rate adds √2.
- Today's position gate is too wide. Consecutive samples share their wander, so real innovations are much smaller than predicted (NIS 0.18): the default 5σ gate accepts a step of about 5 m, over 11× the actual innovation σ. Bias states make the innovations consistent (NIS 1.0) and the same 5σ gate accepts about 1.2 m.
- Scaling R by rate holds position trust near the 5 Hz level and keeps the velocity gain, but widens the gate further. Inflating R can't reach consistency: R × 100 (about 10 m) is still 2× overconfident with a gate ten times wider.
- Bias states keep the reported σ honest at every rate and tolerate a 3× error in τ. A wrong τ or σ shows up in NIS (0.70 and 1.36 above), which gives a way to fit them from logs. Their position RMS below the raw wander comes from averaging GNSS over many τ while holding position on velocity. That relies on the error being zero-mean Gauss-Markov, so don't expect it in flight.

## 3. Bias states in EKF2

- **State:** add `gnss_pos_bias = sf.V2()` (N, E) to `State` in `src/modules/ekf2/EKF/python/ekf_derivation/derivation.py`. SymForce regenerates `predict_covariance.h` and `state.h`. Dynamics b ← e^(−Δt/τ)·b with process noise σ_b²(1 − e^(−2Δt/τ)). Build it out behind a Kconfig option the way `EKF2_WIND` drops `wind_vel` (`--disable_wind`).
- **Fusion:** GNSS position is a direct state update today (`fuseDirectStateMeasurement`). With a bias, H has a 1 on position and on the bias, so fusion moves to `measurementUpdate(K, H, R, innov)` (`ekf.h:288`), and R becomes the white part only.
- **Scope:** only GNSS position fusion sees the bias. External vision, auxiliary global position, position resets and GNSS height don't.
- **Not the existing `BiasEstimator`s.** The GNSS height, EV position and baro bias estimators are separate one-state random-walk filters that track a secondary sensor's offset from the EKF. They aren't coupled to the EKF covariance, and they go inactive when their sensor is the reference. Primary GNSS position has no bias model.

## 4. Open problems

1. **Splitting eph.** A receiver reports one total σ. The filter needs a white R (a parameter, or a fraction of eph) and a bias steady-state variance of eph² − R. P_bb has to be inflated when eph jumps (RTK Fixed → Float → 3D), since a larger eph means the wander just grew.
2. **τ and σ by solution type.** Standalone and SBAS wander by metres over tens of seconds; RTK Fixed by centimetres, where the model barely matters; PPP converges slowly. Needs static logs from F9P, X20 and mosaic receivers to fit.
3. **Resets.** The bias belongs to one receiver. Reset the bias states and their covariance when `selection_count` changes. A blended output's bias shifts whenever the blend weights change, which this model can't represent, so selection should replace blending first.
4. **Height.** A vertical bias state would interact with `_gps_hgt_b_est`. Start with horizontal only.
5. **Gate tightening.** A consistent gate is about 4× narrower in metres. A receiver that steps its position without raising eph in the same sample gets rejected where today it's accepted.
6. **Interim without new states:** scale position R by rate/f_ref in `gps_control.cpp`, and compute the innovation gate from the unscaled variance so the gate doesn't widen.
