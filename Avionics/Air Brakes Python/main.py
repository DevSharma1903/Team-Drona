import argparse
import time
import numpy as np

from airbrakes_smc.config import (
    TARGET_APOGEE_DEFAULT,
    DEFAULT_V_EPS, DEFAULT_TAU, DEFAULT_A_SOUND,
    DEFAULT_E_ON, DEFAULT_E_OFF,
    DEFAULT_K_TRIM, DEFAULT_TRIM_LIMIT,
)
from airbrakes_smc.runner import run_closed_loop_from_csv
from airbrakes_smc.plotting import plot_run


def parse_args():
    p = argparse.ArgumentParser(description="Modular closed-loop airbrake sim (SMC trim + predictor + hysteresis).")
    p.add_argument("--csv", required=True, help="Path to CSV file (must have time column; altitude/velocity optional if h0/v0 provided).")
    p.add_argument("--mass", type=float, required=True, help="Mass (kg).")

    p.add_argument("--target", type=float, default=TARGET_APOGEE_DEFAULT)
    p.add_argument("--dt", type=float, default=None)
    p.add_argument("--v_eps", type=float, default=DEFAULT_V_EPS)
    p.add_argument("--tau", type=float, default=DEFAULT_TAU)
    p.add_argument("--a_sound", type=float, default=DEFAULT_A_SOUND)

    p.add_argument("--E_on", type=float, default=DEFAULT_E_ON)
    p.add_argument("--E_off", type=float, default=DEFAULT_E_OFF)

    p.add_argument("--k_trim", type=float, default=DEFAULT_K_TRIM)
    p.add_argument("--trim_limit", type=float, default=DEFAULT_TRIM_LIMIT)

    p.add_argument("--h0", type=float, default=None)
    p.add_argument("--v0", type=float, default=None)

    p.add_argument("--quiet", action="store_true", help="Disable per-step prints.")
    return p.parse_args()


def main():
    args = parse_args()

    t0 = time.perf_counter()
    logs = run_closed_loop_from_csv(
        csv_path=args.csv,
        mass_kg=args.mass,
        dt=args.dt,
        v_eps=args.v_eps,
        tau=args.tau,
        h0=args.h0,
        v0=args.v0,
        a_sound=args.a_sound,
        target_apogee=args.target,
        E_on=args.E_on,
        E_off=args.E_off,
        k_trim=args.k_trim,
        trim_limit=args.trim_limit,
        verbose=not args.quiet,
    )
    t1 = time.perf_counter()

    # ✅ Always print total runtime (works even if runner logging not updated)
    total_ms = (t1 - t0) * 1000.0
    print("\n=== MAIN TIMING SUMMARY ===")
    print(f"Total runtime       : {total_ms:.3f} ms")

    # Optional: print per-iteration stats if runner.py logged them
    iter_times = logs.get("iter_time_ms", None)
    if iter_times is None:
        print("Per-iteration timing: NOT FOUND (runner.py/logger not updated or not loaded)")
        # Helpful debug line:
        print(f"Available log keys   : {', '.join(sorted(logs.keys()))}")
    else:
        if len(iter_times) == 0:
            print("Per-iteration timing: FOUND but empty")
        else:
            print(f"Total iterations    : {len(iter_times)}")
            print(f"Avg iter time       : {float(np.mean(iter_times)):.3f} ms")
            print(f"Max iter time       : {float(np.max(iter_times)):.3f} ms")
            print(f"Min iter time       : {float(np.min(iter_times)):.3f} ms")
            print(f"Std dev             : {float(np.std(iter_times)):.3f} ms")

            dt_val = logs.get("dt", np.nan)
            if not np.isnan(dt_val):
                dt_ms = float(dt_val) * 1000.0
                rt_margin = logs.get("rt_margin_ms", None)
                misses = int(np.sum(rt_margin < 0)) if rt_margin is not None else 0
                print(f"Control dt          : {dt_ms:.3f} ms")
                print(f"Deadline misses     : {misses}")

    plot_run(logs)


if __name__ == "__main__":
    main()
