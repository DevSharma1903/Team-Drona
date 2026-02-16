from airbrakes_smc.config import G0, Re


def gravity(h: float) -> float:
    """Returns vertical gravity acceleration (negative during ascent)."""
    return -G0 * (Re / (Re + h))**2


def first_order_lag(u_cmd: float, u_lag: float, tau: float, dt: float) -> float:
    if tau <= 0:
        return float(u_cmd)
    return float(u_lag + (dt / tau) * (u_cmd - u_lag))


def integrate_euler(h: float, v: float, a: float, dt: float):
    """Euler step using v_new in h update (matches your script)."""
    v_new = v + a * dt
    h_new = h + v_new * dt
    return float(h_new), float(v_new)
