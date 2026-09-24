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

def sim(f_gps, mode, tau_m=tau, sig_b_m=sig_b, r_scale=1.0, gate_unscaled=False):
    # gate_unscaled: test innovations against P + hAcc^2 while fusing with the scaled R
    step = int(round(1/(f_gps*dt)))
    aug = mode == 'bias'
    n = 3 if aug else 2
    phi_m = np.exp(-dt/tau_m)
    F = np.eye(n); F[0, 1] = dt
    Q = np.zeros((n, n)); Q[1, 1] = (sig_acc*dt)**2
    if aug:
        F[2, 2] = phi_m; Q[2, 2] = sig_b_m**2*(1-phi_m**2)
        Hp = np.array([1., 0, 1]); Rp = sig_w**2
        P = np.diag([hacc**2, 1.0, sig_b_m**2])
    else:
        Hp = np.array([1., 0]); Rp = hacc**2*r_scale
        P = np.diag([hacc**2, 1.0])
    Rgate = hacc**2 if gate_unscaled else None
    Hv = np.zeros(n); Hv[1] = 1; Rv = sig_v**2
    x = np.zeros((RUNS, n))
    phi = np.exp(-dt/tau); qb = sig_b*np.sqrt(1-phi**2)
    b = rng.normal(0, sig_b, RUNS)
    pe2 = ve2 = pp = 0.0; cnt = 0
    s_pos = nis = 0.0; nfuse = 0
    for k in range(N):
        b = phi*b + rng.normal(0, qb, RUNS)
        a = rng.normal(0, sig_acc, RUNS)
        x = x@F.T; x[:, 1] += a*dt
        P = F@P@F.T + Q
        if k % step == 0:
            for H, z, R in ((Hp, b + rng.normal(0, sig_w, RUNS), Rp),
                            (Hv, rng.normal(0, sig_v, RUNS), Rv)):
                S = H@P@H + R; K = P@H/S; innov = z - x@H
                if H is Hp and k*dt > WARM:
                    Sg = S if Rgate is None else H@P@H + Rgate
                    s_pos += Sg; nis += np.mean(innov**2)/Sg; nfuse += 1
                x = x + np.outer(innov, K); P = P - np.outer(K, H@P)
        if k*dt > WARM:
            pe2 += np.mean(x[:, 0]**2); ve2 += np.mean(x[:, 1]**2)
            pp += P[0, 0]; cnt += 1
    return np.sqrt(pe2/cnt), np.sqrt(pp/cnt), np.sqrt(ve2/cnt), np.sqrt(s_pos/nfuse), nis/nfuse

# (label, rates, mode, kwargs); r_scale multiplies today's R = hAcc^2
cases = [('today (EKF2/EKF3)', (5, 10, 20), 'base', {}),
         ('R x rate/5Hz', (10, 20), 'base', 'rate'),
         ('R x 10', (10,), 'base', dict(r_scale=10.0)),
         ('R x 100', (10,), 'base', dict(r_scale=100.0)),
         ('R x tau/dt', (5, 10), 'base', 'tau'),
         ('R x tau/dt, gate on hAcc^2', (5, 10), 'base', 'tau_gate'),
         ('bias states, true model', (5, 10, 20), 'bias', {}),
         ('bias states, tau 20s, sig 1.5m', (10,), 'bias', dict(tau_m=20.0, sig_b_m=1.5)),
         ('bias states, tau 300s, sig 0.7m', (10,), 'bias', dict(tau_m=300.0, sig_b_m=0.7))]
print(f"truth: pos err GM sigma={sig_b} m tau={tau} s + white {sig_w} m (hAcc {hacc:.2f}); vel white {sig_v} m/s")
print(f"{'position model':32s} {'Hz':>3s} {'pos rms':>8s} {'pos sig':>8s} {'x':>5s} {'vel rms':>8s} {'innov sig':>9s} {'NIS':>5s}")
for name, rates, mode, kw in cases:
    for f in rates:
        args = {'rate': dict(r_scale=f/5.0), 'tau': dict(r_scale=tau*f),
                'tau_gate': dict(r_scale=tau*f, gate_unscaled=True)}.get(kw, kw) if isinstance(kw, str) else kw
        pr, ps, vr, si, ni = sim(f, mode, **args)
        print(f"{name:32s} {f:3d} {pr:8.2f} {ps:8.2f} {pr/ps:5.1f} {vr:8.3f} {si:9.2f} {ni:5.2f}")
