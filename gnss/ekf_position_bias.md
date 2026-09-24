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

1D INS/GNSS, IMU prediction at 100 Hz, R set from the reported accuracies as EKF2 does. Truth: position error is Gauss-Markov with σ 1 m and τ 60 s, plus 0.2 m white (eph 1.02 m); velocity error is white, 0.1 m/s. 200 runs of 900 s, first 300 s discarded. "×" is the actual RMS error divided by the σ the filter reports.

| Position model | Rate | Pos RMS | Pos σ | × | Vel RMS |
|---|---|---|---|---|---|
| Today | 5 Hz | 0.91 m | 0.14 m | 6.4 | 0.039 m/s |
| Today | 10 Hz | 0.93 m | 0.10 m | 9.2 | 0.033 m/s |
| Today | 20 Hz | 0.92 m | 0.07 m | 12.9 | 0.028 m/s |
| R × rate/5 Hz | 10 Hz | 0.88 m | 0.12 m | 7.3 | 0.033 m/s |
| R × rate/5 Hz | 20 Hz | 0.89 m | 0.10 m | 8.8 | 0.028 m/s |
| Bias states, true τ and σ | 5 Hz | 0.63 m | 0.62 m | 1.0 | 0.039 m/s |
| Bias states, true τ and σ | 10 Hz | 0.56 m | 0.55 m | 1.0 | 0.033 m/s |
| Bias states, true τ and σ | 20 Hz | 0.47 m | 0.49 m | 1.0 | 0.028 m/s |
| Bias states, τ 20 s, σ 1.5 m | 10 Hz | 0.54 m | 0.54 m | 1.0 | 0.033 m/s |
| Bias states, τ 300 s, σ 0.7 m | 10 Hz | 0.60 m | 0.57 m | 1.1 | 0.034 m/s |

- Velocity accuracy depends on the rate, not on the position model. The 1D model has no tilt or accelerometer-bias states, so whether over-trusted position wander leaks into those is untested.
- Today's model is already about 6× overconfident at 5 Hz; each doubling of the rate adds √2.
- Scaling R by rate holds position trust at the 5 Hz level and keeps the velocity gain. It doesn't fix the overconfidence that is already there.
- Bias states keep the reported σ honest at every rate and tolerate a 3× error in τ. Their position RMS below the raw wander comes from averaging GNSS over many τ while holding position on velocity. That relies on the error being zero-mean Gauss-Markov, so don't expect it in flight.

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
5. **Interim without new states:** scale position R by rate/f_ref in `gps_control.cpp`, and compute the innovation gate from the unscaled variance so the gate doesn't widen.
