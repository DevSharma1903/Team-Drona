# airbrakes_smc/runner.py
import numpy as np

from airbrakes_smc.models import ModelParams, ModelState, SMCReachingLawParams, CtrlHysteresis
from airbrakes_smc.io_csv import load_timeseries
from airbrakes_smc.logging import RunLogger

# CHANGED: no longer import delta_drag_lut
from airbrakes_smc.aero_lut import drag_subsystem, cd_delta

from airbrakes_smc.dynamics import first_order_lag, integrate_euler
from airbrakes_smc.controller.hysteresis import update_hysteresis
from airbrakes_smc.controller.command import compute_control_command

from airbrakes_smc.predictor import simulate_to_apogee_cb
from airbrakes_smc.atmosphere import density_isa, speed_of_sound
from airbrakes_smc.aero_cd import cd_lut

A0_BRAKE = 20e-4

def run_closed_loop_from_csv(
    csv_path: str,
    mass_kg: float,
    dt: float | None = None,
    v_eps: float = 2.5,
    tau: float = 0.1,
    h0: float | None = None,
    v0: float | None = None,
    a_sound: float = 340.0,
    target_apogee: float = 9300.0,
    E_on: float = 15.0,
    E_off: float = 5.0,
    k_trim: float = 0.1,
    trim_limit: float = 0.2,
    verbose: bool = True,
    diameter_m: float = 0.126,
    dt_pred_min: float = 0.02,
):
    t, h0, v0, dt, _cols = load_timeseries(csv_path, dt=dt, h0=h0, v0=v0)

    params = ModelParams(m=mass_kg, g=9.81, tau=tau, v_eps=v_eps, a_sound=a_sound)
    smc_params = SMCReachingLawParams()
    st = ModelState(h=h0, v=v0, u_lag=0.0, s=0.0)

    ctrl = CtrlHysteresis(E_on=E_on, E_off=E_off, enabled=False)
    logger = RunLogger()

    # Reference area (baseline rocket drag area uses body cross-section)
    AREA = 0.0176

    for k in range(len(t)):
        dt_pred = max(float(dt), float(dt_pred_min))

        rho = density_isa(st.h)
        a_sound_loc = speed_of_sound(st.h)

        mach_pred = abs(st.v) / max(a_sound_loc, 1e-6)
        Cd0 = cd_lut(mach_pred)

        # -----------------------------
        # Predictor Cb endpoints
        # -----------------------------
        CdA_no = Cd0 * AREA
        Cb_no = params.m / max(CdA_no, 1e-9)

        dCd_full = cd_delta(mach_pred, 1.0)      
        delta_CdA_full = dCd_full * A0_BRAKE       

        CdA_full = CdA_no + delta_CdA_full
        Cb_full = params.m / max(CdA_full, 1e-9)

        pred_no, _ = simulate_to_apogee_cb(st.h, st.v, Cb_no, dt_pred, rho)
        pred_full, _ = simulate_to_apogee_cb(st.h, st.v, Cb_full, dt_pred, rho)

        if not np.isfinite(pred_no):
            pred_no = st.h
        if not np.isfinite(pred_full):
            pred_full = st.h

        # -----------------------------
        # Supervisor
        # -----------------------------
        err = pred_no - target_apogee
        enabled = update_hysteresis(ctrl, float(err))

        # -----------------------------
        # Plant drag terms
        # -----------------------------
        mach_now = abs(st.v) / max(a_sound_loc, 1e-6)
        Cd0_now = cd_lut(mach_now)

        # pass a_sound_loc and rho, plus baseline Cd0 + AREA
        mach, F_drag, Dm_wo, Dm_only = drag_subsystem(
            v=st.v,
            u_with_lag=st.u_lag,
            m=params.m,
            a_sound=a_sound_loc,
            rho=rho,
            Cd0=Cd0_now,
            Aref=AREA
        )

        # -----------------------------
        # Controller command
        # -----------------------------
        v_clone = st.v
        u_cmd, dbg = compute_control_command(
            st=st,
            pred_no=float(pred_no),
            pred_full=float(pred_full),
            target=target_apogee,
            enabled=enabled,
            v_clone=v_clone,
            Dm_wo=Dm_wo,
            Dm_only=Dm_only,
            params=params,
            smc_params=smc_params,
            k_trim=k_trim,
            trim_limit=trim_limit,
        )

        if verbose:
            print(
                f"{k:03d} | pred_no={pred_no:.1f} | pred_full={pred_full:.1f} | "
                f"err={err:.1f} | enable={int(enabled)} | u={st.u_lag:.3f}"
            )

        # Actuator lag
        u_lag_new = first_order_lag(u_cmd, st.u_lag, params.tau, float(dt))

        # Plant dynamics
        a_drag = F_drag / params.m
        a = a_drag - params.g

        h_new, v_new = integrate_euler(st.h, st.v, a, float(dt))

        st = ModelState(h=h_new, v=v_new, u_lag=u_lag_new, s=dbg.s)

        logger.push(
            t=float(t[k]),
            h=float(st.h),
            v=float(st.v),
            a=float(a),
            u_lag=float(st.u_lag),
            u_cmd=float(u_cmd),
            pred_no=float(pred_no),
            pred_full=float(pred_full),
            err=float(err),
            enabled=float(enabled),
            u_ff=float(dbg.u_ff),
            u_smc=float(dbg.u_smc),
            u_trim=float(dbg.u_trim),
            s=float(dbg.s),
            mach=float(mach),
            F_drag=float(F_drag),
        )

    logs = logger.to_numpy()
    logs["dt"] = float(dt)
    return logs
