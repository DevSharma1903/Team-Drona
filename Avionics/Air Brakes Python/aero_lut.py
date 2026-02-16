import numpy as np

A0 = 33e-4  # m^2 per "u"

def cd_delta(mach: float, u: float) -> float:
    Ma = float(mach)
    u = float(np.clip(u, 0.0, 1.0))
    return (
        9.6187 * Ma * u
        - 19.0139 * (Ma**2) * u
        + 1.1245 * Ma * (u**2)
        + 10.2736 * (Ma**3) * u
        + 3.7661 * (Ma**2) * (u**2)
        - 3.0457 * Ma * (u**3)
    )

def drag_subsystem(v: float, u_with_lag: float, m: float, a_sound: float, rho: float,
                   Cd0: float, Aref: float):
    """
    Cd0: baseline rocket Cd (no brakes)
    Aref: reference area for baseline drag (body cross-section)
    Returns:
      mach, F_drag_signed (N), Dm_wo, Dm_only
    """
    u = float(np.clip(u_with_lag, 0.0, 1.0))
    mach = abs(v) / max(a_sound, 1e-6)

    q = 0.5 * rho * v * v

    # Baseline drag (no brakes)
    F_wo_mag = q * Aref * Cd0

    # Delta drag from brakes (your formula)
    dCd = cd_delta(mach, u)
    A_u = A0 * u
    F_delta_mag = q * A_u * dCd
    F_delta_mag = 2*F_delta_mag

    F_mag = F_wo_mag + F_delta_mag

    if v > 0:
        F_drag = -F_mag
        sgn = 1.0
    elif v < 0:
        F_drag = +F_mag
        sgn = -1.0
    else:
        F_drag = 0.0
        sgn = 0.0

    Dm_wo   = -(sgn * F_wo_mag) / m
    Dm_only = -(sgn * F_delta_mag) / m  # actual delta accel at current u (recommended)

    return float(mach), float(F_drag), float(Dm_wo), float(Dm_only)
