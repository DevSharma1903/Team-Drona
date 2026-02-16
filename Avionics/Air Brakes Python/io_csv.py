import numpy as np
import pandas as pd


def pick_col(df, candidates):
    for c in candidates:
        if c in df.columns:
            return c
    return None


def load_timeseries(csv_path: str, dt: float | None = None, h0: float | None = None, v0: float | None = None):
    df = pd.read_csv(csv_path)

    t_col = pick_col(df, ["t", "time", "timestamp", "Time", "Timestamp"])
    h_col = pick_col(df, ["h", "alt", "altitude", "Altitude_P"])
    v_col = pick_col(df, ["v", "vel", "velocity", "vz", "EKF_velocity"])

    if t_col is None:
        raise ValueError("CSV must contain a time column: t/time/timestamp.")

    df = df[np.isfinite(df[t_col])].copy()
    df.reset_index(drop=True, inplace=True)

    t = df[t_col].to_numpy(dtype=float)

    if dt is None:
        diffs = np.diff(t)
        diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
        if len(diffs) == 0:
            raise ValueError("Cannot infer dt from timestamp. Pass dt manually.")
        dt = float(np.median(diffs))

    if h0 is None:
        if h_col is None:
            raise ValueError("Provide h0 or include altitude column (h/alt/altitude).")
        h0 = float(df[h_col].iloc[0])

    if v0 is None:
        if v_col is None:
            raise ValueError("Provide v0 or include velocity column (v/vel/velocity).")
        v0 = float(df[v_col].iloc[0])

    return t, float(h0), float(v0), float(dt), {"t_col": t_col, "h_col": h_col, "v_col": v_col}
