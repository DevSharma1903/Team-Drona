import numpy as np

# -------------------------------------------------
# Mach vs Cd lookup table (example structure)
# Replace values with your wind-tunnel / RASAero data
# -------------------------------------------------

CD_BP = np.array(
    [0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00, 1.05, 1.10],
    dtype=float
)

CD_TABLE = np.array(
    [1.38, 0.501, 0.502, 0.504, 0.507, 0.509, 0.513, 0.517, 0.522, 0.528, 0.534, 0.541, 0.549, 0.557, 0.566, 0.575, 0.586, 0.597, 0.608, 0.616, 0.625, 0.664, 0.664],
    dtype=float
)


# -------------------------------------------------
# Generic clipped 1-D interpolation (same as yours)
# -------------------------------------------------

def interp_1d_clip(x: float, bp: np.ndarray, table: np.ndarray) -> float:
    if x <= bp[0]:
        return float(table[0])
    if x >= bp[-1]:
        return float(table[-1])
    return float(np.interp(x, bp, table))


# -------------------------------------------------
# Mach → Cd lookup
# -------------------------------------------------

def cd_lut(mach: float) -> float:
    """
    Returns Cd for given Mach number using clipped interpolation.
    """
    return interp_1d_clip(mach, CD_BP, CD_TABLE)
