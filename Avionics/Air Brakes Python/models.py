from dataclasses import dataclass, field
import numpy as np


@dataclass
class SMCReachingLawParams:
    C: np.ndarray = field(default_factory=lambda: np.array([[2.0], [1.0]], dtype=float))
    eta: float = 1.25
    K: float = 0.01
    alpha: float = 0.5
    phi: float = 200


@dataclass
class ModelParams:
    m: float
    g: float = 9.81
    tau: float = 0.1
    v_eps: float = 2.5
    u_min: float = 0.0
    u_max: float = 1.0
    a_sound: float = 340.0


@dataclass
class ModelState:
    h: float = 0.0
    v: float = 0.0
    u_lag: float = 0.0
    s: float = 0.0


@dataclass
class CtrlHysteresis:
    E_on: float = 15.0
    E_off: float = 5.0
    enabled: bool = False


@dataclass
class CmdDebug:
    enabled: bool = False
    err: float = 0.0
    u_ff: float = 0.0
    u_smc: float = 0.0
    u_trim: float = 0.0
    s: float = 0.0
