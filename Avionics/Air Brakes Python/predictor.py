# airbrakes_smc/predictor.py
from __future__ import annotations

import numpy as np

# Keep these as module-level constants (same physics as your script)
G0 = 9.80665
RE = 6371000.0

MIN_VEL_FOR_CB = 1.0
MIN_ADRAG_FOR_CB = 0.1


def ballistic_coefficient_from_state(
    h_m: float,
    v_mps: float,
    a_mps2: float,
    rho_kgpm3: float,
    g0: float = G0,
    re_m: float = RE,
    min_vel: float = MIN_VEL_FOR_CB,
    min_adrag: float = MIN_ADRAG_FOR_CB,
) -> float:
    """
    Estimate ballistic coefficient Cb from instantaneous state.

    This mirrors your script's Cb(h,v,a,rho):

        a_drag = a - g(h)
        Cb = rho*v^2 / (2*|a_drag|)

    Where g(h) = -g0*(Re/(Re+h))^2  (negative upward axis convention).

    Notes:
      - Returns np.nan if velocity is too small or inferred drag accel is too small.
      - Assumes 'a_mps2' is *net vertical acceleration in the same sign convention as v*
        (i.e., + upward). If your accel is sensor-frame, convert before calling.
    """
    if abs(v_mps) < float(min_vel):
        return float("nan")

    g_val = -float(g0) * (float(re_m) / (float(re_m) + float(h_m))) ** 2  # negative
    a_drag = float(a_mps2) - g_val

    if abs(a_drag) < float(min_adrag):
        return float("nan")

    cb = float(rho_kgpm3) * float(v_mps) ** 2 / (2.0 * abs(a_drag))
    return float(cb)


def _coast_dynamics(
    h: float,
    v: float,
    cb: float,
    rho: float,
    g0: float,
    re_m: float,
) -> tuple[float, float]:
    """
    Returns time derivatives (dh/dt, dv/dt) for coast model:
      dh/dt = v
      dv/dt = g(h) + a_drag(v)
    """
    g_val = -float(g0) * (float(re_m) / (float(re_m) + h)) ** 2
    a_drag = -(float(rho) * v * abs(v)) / (2.0 * float(cb))
    a = g_val + a_drag
    return v, a


def simulate_to_apogee_cb(
    h0_m: float,
    v0_mps: float,
    cb: float,
    dt_s: float,
    rho_kgpm3: float,
    g0: float = G0,
    re_m: float = RE,
    t_max_s: float = 300.0,
    max_steps: int = 50000,
) -> tuple[float, float]:
    """
    Coast-only apogee predictor using ballistic coefficient Cb.

    Model:
      g(h) = -g0*(Re/(Re+h))^2
      a_drag = -(rho*v*|v|)/(2*Cb)
      dv/dt = g(h) + a_drag
      dh/dt = v

    Integration:
      - Uses RK4 (4th-order Runge-Kutta) with step dt_s.

    Returns:
      (predicted_apogee_m, time_to_apogee_s)

    Returns (nan, nan) if cb invalid.

    Important:
      - rho is treated constant over the prediction horizon (matches your script).
      - dt_s should be > 0; you can pass your control loop dt.
    """
    if cb is None or (not np.isfinite(cb)) or cb <= 0.0:
        return float("nan"), float("nan")

    if v0_mps <= 0.0:
        return float(h0_m), 0.0

    dt = float(dt_s)
    if dt <= 0.0:
        return float("nan"), float("nan")

    h = float(h0_m)
    v = float(v0_mps)
    t = 0.0

    steps = 0
    while v > 0.0 and t < float(t_max_s) and steps < int(max_steps):
        # --- RK4 for state x = [h, v] ---
        dh1, dv1 = _coast_dynamics(h, v, cb, rho_kgpm3, g0, re_m)

        h2 = h + 0.5 * dt * dh1
        v2 = v + 0.5 * dt * dv1
        dh2, dv2 = _coast_dynamics(h2, v2, cb, rho_kgpm3, g0, re_m)

        h3 = h + 0.5 * dt * dh2
        v3 = v + 0.5 * dt * dv2
        dh3, dv3 = _coast_dynamics(h3, v3, cb, rho_kgpm3, g0, re_m)

        h4 = h + dt * dh3
        v4 = v + dt * dv3
        dh4, dv4 = _coast_dynamics(h4, v4, cb, rho_kgpm3, g0, re_m)

        h_next = h + (dt / 6.0) * (dh1 + 2.0 * dh2 + 2.0 * dh3 + dh4)
        v_next = v + (dt / 6.0) * (dv1 + 2.0 * dv2 + 2.0 * dv3 + dv4)

        # Optional: stop closer to the true apogee when we cross v=0 within this step
        # Linear interpolation in v for time fraction; and in h between endpoints.
        if v_next <= 0.0:
            # fraction f in [0,1] where v crosses zero: v + f*(v_next - v) = 0
            dv_step = (v_next - v)
            if dv_step != 0.0:
                f = (-v) / dv_step
                f = max(0.0, min(1.0, float(f)))
                h = h + f * (h_next - h)
                t = t + f * dt
            else:
                h = h_next
                t = t + dt
            v = v_next
            break

        h, v = h_next, v_next
        t += dt
        steps += 1

    return float(h), float(t)


def predict_apogee_from_velocity(
    h_m: float,
    v_mps: float,
    cd: float,
    area_m2: float,
    mass_kg: float,
    rho_kgpm3: float,
    dt_s: float,
) -> tuple[float, float, float]:
    """
    Convenience wrapper when you already have velocity and can compute Cd.

    Uses:
      Cb = m / (Cd*A)
      then simulate_to_apogee_cb(...)

    Returns:
      (predicted_apogee_m, time_to_apogee_s, cb)

    Returns (nan, nan, nan) if Cd/A invalid.
    """
    cdA = float(cd) * float(area_m2)
    if not np.isfinite(cdA) or cdA <= 0.0:
        return float("nan"), float("nan"), float("nan")

    cb = float(mass_kg) / cdA
    h_ap, t_ap = simulate_to_apogee_cb(h_m, v_mps, cb, dt_s, rho_kgpm3)
    return float(h_ap), float(t_ap), float(cb)
