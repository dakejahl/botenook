# 1D INS/GNSS Kalman filter: how GNSS rate and the GNSS error model change what the filter believes
# about its position and velocity. Backs gnss/ekf_position_bias.md. Needs numpy.
# KF covariance is data-independent, so P/K are computed once and states are batched across MC runs.
# IMU noise is white at the filter's assumed level (EKF2_ACC_NOISE default), so the IMU never helps
# more than the filter expects.
import numpy as np
rng = np.random.default_rng(2)
dt = 0.01; T = 900.0; N = int(T/dt); RUNS = 200; WARM = 300.0
sig_acc = 0.35
sig_b, tau_p, sig_w = 1.0, 60.0, 0.2     # truth: position wander (GM) + white
hacc = np.hypot(sig_b, sig_w)
VEL_TRUTH = {'white': (0.0, 1.0, 0.1),   # truth: velocity GM sigma, tau, white sigma
             'correlated': (0.05, 20.0, 0.05)}

def sim(f_gps, vel_truth, pos='white', vel='white', r_scale=1.0, v_scale=1.0,
        tau_m=tau_p, sig_b_m=sig_b, gate_unscaled=False):
    # pos/vel: 'white' fuses with R = acc^2 * scale; 'bias' adds a Gauss-Markov bias state.
    # gate_unscaled: test position innovations against P + hAcc^2 while fusing with the scaled R.
    sig_vb, tau_v, sig_vw = VEL_TRUTH[vel_truth]
    sacc = np.hypot(sig_vb, sig_vw)
    step = int(round(1/(f_gps*dt)))
    n = 2 + (pos == 'bias') + (vel == 'bias')
    F = np.eye(n); F[0, 1] = dt
    Q = np.zeros((n, n)); Q[1, 1] = (sig_acc*dt)**2
    P = np.zeros((n, n)); P[0, 0] = hacc**2; P[1, 1] = 1.0
    Hp = np.zeros(n); Hp[0] = 1; Hv = np.zeros(n); Hv[1] = 1
    i = 2
    if pos == 'bias':
        ph = np.exp(-dt/tau_m); F[i, i] = ph; Q[i, i] = sig_b_m**2*(1-ph**2); P[i, i] = sig_b_m**2
        Hp[i] = 1; Rp = sig_w**2; i += 1
    else:
        Rp = hacc**2*r_scale
    if vel == 'bias':
        ph = np.exp(-dt/tau_v); F[i, i] = ph; Q[i, i] = sig_vb**2*(1-ph**2); P[i, i] = sig_vb**2
        Hv[i] = 1; Rv = sig_vw**2
    else:
        Rv = sacc**2*v_scale
    Rgate = hacc**2 if gate_unscaled else None
    x = np.zeros((RUNS, n))
    pb = np.exp(-dt/tau_p); qb = sig_b*np.sqrt(1-pb**2); b = rng.normal(0, sig_b, RUNS)
    pv = np.exp(-dt/tau_v); qv = sig_vb*np.sqrt(1-pv**2); bv = rng.normal(0, sig_vb, RUNS)
    pe2 = ve2 = pp = vp = 0.0; cnt = 0
    s_pos = nis = 0.0; nfuse = 0
    for k in range(N):
        b = pb*b + rng.normal(0, qb, RUNS); bv = pv*bv + rng.normal(0, qv, RUNS)
        x = x@F.T; x[:, 1] += rng.normal(0, sig_acc, RUNS)*dt
        P = F@P@F.T + Q
        if k % step == 0:
            for H, z, R in ((Hp, b + rng.normal(0, sig_w, RUNS), Rp),
                            (Hv, bv + rng.normal(0, sig_vw, RUNS), Rv)):
                S = H@P@H + R; K = P@H/S; innov = z - x@H
                if H is Hp and k*dt > WARM:
                    Sg = S if Rgate is None else H@P@H + Rgate
                    s_pos += Sg; nis += np.mean(innov**2)/Sg; nfuse += 1
                x = x + np.outer(innov, K); P = P - np.outer(K, H@P)
        if k*dt > WARM:
            pe2 += np.mean(x[:, 0]**2); ve2 += np.mean(x[:, 1]**2)
            pp += P[0, 0]; vp += P[1, 1]; cnt += 1
    pr, ps, vr, vs = (np.sqrt(v/cnt) for v in (pe2, pp, ve2, vp))
    return pr, ps, vr, vs, np.sqrt(s_pos/nfuse), nis/nfuse

TAU = lambda f: dict(r_scale=tau_p*f, gate_unscaled=True)
TAU_PV = lambda f: dict(r_scale=tau_p*f, v_scale=VEL_TRUTH['correlated'][1]*f, gate_unscaled=True)
# (velocity truth, label, rates, kwargs or f -> kwargs)
cases = [('white', 'today (EKF2/EKF3)', (5, 10, 20), {}),
         ('white', 'R x rate/5Hz', (10, 20), lambda f: dict(r_scale=f/5.0)),
         ('white', 'R x 100', (10,), dict(r_scale=100.0)),
         ('white', 'pos R x tau/dt', (10,), lambda f: dict(r_scale=tau_p*f)),
         ('white', 'pos R x tau/dt, gate on hAcc^2', (5, 10), TAU),
         ('white', 'pos bias states', (5, 10, 20), dict(pos='bias')),
         ('white', 'pos bias, tau 20s, sig 1.5m', (10,), dict(pos='bias', tau_m=20.0, sig_b_m=1.5)),
         ('white', 'pos bias, tau 300s, sig 0.7m', (10,), dict(pos='bias', tau_m=300.0, sig_b_m=0.7)),
         ('correlated', 'today (EKF2/EKF3)', (5, 10), {}),
         ('correlated', 'pos R x tau/dt, gate on hAcc^2', (10,), TAU),
         ('correlated', 'pos+vel R x tau/dt, gate hAcc^2', (5, 10), TAU_PV),
         ('correlated', 'pos bias states', (10,), dict(pos='bias')),
         ('correlated', 'pos+vel bias states', (5, 10), dict(pos='bias', vel='bias'))]
print(f"position error: GM sigma {sig_b} m tau {tau_p} s + white {sig_w} m (hAcc {hacc:.2f})")
print("velocity error: white 0.1 m/s, or correlated GM 0.05 m/s tau 20 s + white 0.05 m/s")
print(f"{'vel truth':10s} {'filter':32s} {'Hz':>3s} {'pos rms':>7s} {'pos sig':>7s} {'x':>5s} "
      f"{'vel rms':>7s} {'vel sig':>7s} {'x':>4s} {'gate sig':>8s} {'NIS':>5s}")
for vt, name, rates, kw in cases:
    for f in rates:
        args = kw(f) if callable(kw) else kw
        pr, ps, vr, vs, si, ni = sim(f, vt, **args)
        print(f"{vt:10s} {name:32s} {f:3d} {pr:7.2f} {ps:7.2f} {pr/ps:5.1f} "
              f"{vr:7.3f} {vs:7.3f} {vr/vs:4.1f} {si:8.2f} {ni:5.2f}")

def step_response(f_gps, r_scale, v_scale, sacc):
    # Noise-free mean response to a 2 m GNSS position step with no change in reported accuracy:
    # seconds until the estimate covers 63% of it.
    step = int(round(1/(f_gps*dt)))
    F = np.eye(2); F[0, 1] = dt; Q = np.zeros((2, 2)); Q[1, 1] = (sig_acc*dt)**2
    P = np.diag([hacc**2, 1.0]); x = np.zeros(2)
    Hp = np.array([1., 0]); Hv = np.array([0., 1])
    for k in range(N):
        x = F@x; P = F@P@F.T + Q
        if k % step == 0:
            for H, z, R in ((Hp, 2.0 if k*dt >= 600 else 0.0, hacc**2*r_scale), (Hv, 0.0, sacc**2*v_scale)):
                S = H@P@H + R; K = P@H/S; x = x + K*(z - x@H); P = P - np.outer(K, H@P)
        if k*dt >= 600 and x[0] >= 2*0.632:
            return k*dt - 600
    return float('inf')

sacc_c = np.hypot(*VEL_TRUTH['correlated'][::2])
print("\n2 m GNSS step, no eph change, time to 63% (s):")
for f in (5, 10):
    print(f"{f:3d} Hz  today {step_response(f, 1, 1, sacc_c):6.0f}  pos R x tau/dt {step_response(f, tau_p*f, 1, sacc_c):6.0f}"
          f"  pos+vel R x tau/dt {step_response(f, tau_p*f, VEL_TRUTH['correlated'][1]*f, sacc_c):6.0f}")
