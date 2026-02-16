import numpy as np

from airbrakes_smc.models import ModelState, ModelParams, SMCReachingLawParams, CmdDebug
from airbrakes_smc.controller.smc import deadband_v, smc_reaching_law_u


def compute_u_ff(pred_no: float, pred_full: float, target: float) -> float: #feed forward u clipped to 1
    den = pred_no - pred_full
    if den > 1e-6:
        u_ff = (pred_no - target) / den
    else:
        u_ff = 0.0
    return float(np.clip(u_ff, 0.0, 1.0))


def compute_u_trim(
    st: ModelState,
    v_clone: float,
    Dm_wo: float,
    Dm_only: float,
    pred_no: float,
    target: float,
    params: ModelParams,
    smc_params: SMCReachingLawParams,
    k_trim: float,
    trim_limit: float
):
    apogee_error = pred_no - target
    v_db = deadband_v(st.v, params.v_eps)

    x = np.array([[apogee_error], [v_db]], dtype=float)
    fx = np.array([[v_clone], [Dm_wo]], dtype=float)
    gx = np.array([[0.0], [Dm_only]], dtype=float)

    '''SMC used to trim perfect feed forward u not given full authority'''
    u_smc, s = smc_reaching_law_u(x, fx, gx, smc_params)
    u_trim = float(np.clip(k_trim * u_smc, -trim_limit, +trim_limit)) #k is used to trim 
    return u_trim, u_smc, s


def compose_u_cmd(u_ff: float, u_trim: float, u_min: float, u_max: float) -> float: 
    return float(np.clip(u_ff + u_trim, u_min, u_max)) #clip u_ff + u_trim to u_min and u_max


def compute_control_command(
    st: ModelState,
    pred_no: float,
    pred_full: float,
    target: float,
    enabled: bool,
    v_clone: float,
    Dm_wo: float,
    Dm_only: float,
    params: ModelParams,
    smc_params: SMCReachingLawParams,
    k_trim: float,
    trim_limit: float
):
    dbg = CmdDebug(enabled=enabled, err=float(pred_no - target))

    if not enabled:
        dbg.u_ff = 0.0
        dbg.u_smc = 0.0
        dbg.u_trim = 0.0
        dbg.s = st.s
        return 0.0, dbg

    u_ff = compute_u_ff(pred_no, pred_full, target)
    u_trim, u_smc, s = compute_u_trim(
        st=st,
        v_clone=v_clone,
        Dm_wo=Dm_wo,
        Dm_only=Dm_only,
        pred_no=pred_no,
        target=target,
        params=params,
        smc_params=smc_params,
        k_trim=k_trim,
        trim_limit=trim_limit
    )

    u_cmd = compose_u_cmd(u_ff, u_trim, params.u_min, params.u_max)

    dbg.u_ff = u_ff
    dbg.u_smc = u_smc
    dbg.u_trim = u_trim
    dbg.s = s
    return u_cmd, dbg
