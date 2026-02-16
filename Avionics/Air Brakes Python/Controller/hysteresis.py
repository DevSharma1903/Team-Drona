from airbrakes_smc.models import CtrlHysteresis


def update_hysteresis(ctrl: CtrlHysteresis, err_m: float) -> bool:
    """
    Schmitt trigger:
      - If disabled, enable when err > +E_on
      - If enabled, disable when err < -E_off
    """
    if not ctrl.enabled:
        if err_m > ctrl.E_on:
            ctrl.enabled = True
    else:
        if err_m < -ctrl.E_off:
            ctrl.enabled = False
    return ctrl.enabled
