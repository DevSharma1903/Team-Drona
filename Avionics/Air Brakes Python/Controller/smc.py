import numpy as np
from airbrakes_smc.models import SMCReachingLawParams


def deadband_v(v: float, v_eps: float) -> float:
    return 0.0 if abs(v) <= v_eps else v


def theta_saturation(s: float, phi: float) -> float:
    if s > phi:
        return 1.0
    if s < -phi:
        return -1.0
    return s / phi if phi > 0 else float(np.sign(s))

'''
greater the reaching law faster the correction and vice versa, h(s)'s sign should not flip fast otherwise chattering

eta controls how hard system is pulled towards sliding surface 's' increase of eta will give faster convergence and stron correction if overshoot is large
but will put more stress on the actuator, lower eta gives smooth control and low mechanical stress but may not be able to kill overshoot in time
in one line 'how urgently you care about being wrong'


if alpha is small the controller is agressive if system is far away from the system and gentle if near s
smaller (~0) alpha makes controller feel like bang bang
bigger (~1) alpha makes it feel like PID
in one line 'how sharp the transition is between wrong and almost correct'

K is shock absorber increasing it will give smooth behaviour and less oscillation near apogee but slower convergence increase if actuator lag is more
controls how much you hate oscillations

phi defines soft zone near s=0 increasing it reduces chatter but precision is less decreasing it gives bang bang tendencies

tuning priority - eta,alpha,K,phi

'''
def reaching_law_h(s: float, p: SMCReachingLawParams) -> float:
    th = theta_saturation(s, p.phi)
    return -(p.eta * (abs(s) ** p.alpha) * th) - (p.K * s)


def smc_reaching_law_u(x: np.ndarray, fx: np.ndarray, gx: np.ndarray, p: SMCReachingLawParams):
    """
    u = (C^T g(x))^-1 ( -C^T f(x) + h(s) ),  s = C^T x
    """
    C = p.C.reshape(2, 1)

    s = float((C.T @ x).item())
    Ct_f = float((C.T @ fx).item())
    Ct_g = float((C.T @ gx).item())

    if abs(Ct_g) < 1e-12:
        return 0.0, s

    u = (-Ct_f + reaching_law_h(s, p)) / Ct_g
    return float(u), float(s)
