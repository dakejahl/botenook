# 1D INS/GNSS Kalman filter: how GNSS rate and the position error model change what the filter
# believes about its position. Backs gnss/ekf_position_bias.md. Needs numpy.
# KF covariance is data-independent, so P/K are computed once and states are batched across MC runs.
import numpy as np
rng = np.random.default_rng(2)
dt = 0.01; T = 900.0; N = int(T/dt); RUNS = 200; WARM = 300.0
sig_acc = 0.35
sig_b, tau = 1.0, 60.0       # truth: slow position wander
sig_w = 0.2                  # truth: white part of position error
sig_v = 0.1                  # truth: GPS velocity noise (white)
hacc = np.hypot(sig_b, sig_w)

def sim(f_gps, mode, tau_m=tau, sig_b_m=sig_b, f_ref=5.0):
    step = int(round(1/(f_gps*dt)))
    aug = mode.startswith('bias')
    n = 3 if aug else 2
    phi_m = np.exp(-dt/tau_m)
    F = np.eye(n); F[0, 1] = dt
    Q = np.zeros((n, n)); Q[1, 1] = (sig_acc*dt)**2
    if aug:
        F[2, 2] = phi_m; Q[2, 2] = sig_b_m**2*(1-phi_m**2)
        Hp = np.array([1., 0, 1]); Rp = sig_w**2
        P = np.diag([hacc**2, 1.0, sig_b_m**2])
    else:
        Hp = np.array([1., 0]); Rp = hacc**2
        if mode == 'scaled':
            Rp *= f_gps/f_ref           # hold position trust at f_ref
        P = np.diag([hacc**2, 1.0])
    Hv = np.zeros(n); Hv[1] = 1; Rv = sig_v**2
    x = np.zeros((RUNS, n))
    phi = np.exp(-dt/tau); qb = sig_b*np.sqrt(1-phi**2)
    b = rng.normal(0, sig_b, RUNS)
    pe2 = ve2 = pp = vp = 0.0; cnt = 0
    for k in range(N):
        b = phi*b + rng.normal(0, qb, RUNS)
        a = rng.normal(0, sig_acc, RUNS)
        x = x@F.T; x[:, 1] += a*dt
        P = F@P@F.T + Q
        if k % step == 0:
            for H, z, R in ((Hp, b + rng.normal(0, sig_w, RUNS), Rp),
                            (Hv, rng.normal(0, sig_v, RUNS), Rv)):
                S = H@P@H + R; K = P@H/S
                x = x + np.outer(z - x@H, K); P = P - np.outer(K, H@P)
        if k*dt > WARM:
            pe2 += np.mean(x[:, 0]**2); ve2 += np.mean(x[:, 1]**2)
            pp += P[0, 0]; vp += P[1, 1]; cnt += 1
    return [np.sqrt(v/cnt) for v in (pe2, pp, ve2, vp)]

cases = [('today (EKF2/EKF3)', 'base', {}),
         ('pos R x rate/5Hz', 'scaled', {}),
         ('bias state (true model)', 'bias', {}),
         ('bias state (tau 20s, sig 1.5m)', 'bias_mis', dict(tau_m=20.0, sig_b_m=1.5)),
         ('bias state (tau 300s, sig 0.7m)', 'bias_mis2', dict(tau_m=300.0, sig_b_m=0.7))]
print(f"truth: pos err GM sigma={sig_b} m tau={tau} s + white {sig_w} m (hAcc {hacc:.2f}); vel white {sig_v} m/s")
print(f"{'strategy':34s} {'Hz':>3s} {'pos rms':>8s} {'pos sig':>8s} {'pos x':>6s} {'vel rms':>8s} {'vel sig':>8s} {'vel x':>6s}")
for name, mode, kw in cases:
    for f in (5, 10, 20):
        pr, ps, vr, vs = sim(f, mode, **kw)
        print(f"{name:34s} {f:3d} {pr:8.3f} {ps:8.3f} {pr/ps:6.1f} {vr:8.4f} {vs:8.4f} {vr/vs:6.2f}")
