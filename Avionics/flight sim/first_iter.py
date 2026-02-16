'''
Time column should be named as - timestamp
Pressure column should be named as - Pres
Acceleration (along longitudnal axis) column should be named as - AcclY
Airspeed column should be named as - Vair
'''
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def estimate_fs_from_time(t: np.ndarray) -> float:
    t = np.asarray(t, dtype=float)
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    if dt.size == 0:
        raise ValueError("Could not estimate fs: time column has no valid increasing values.")
    dt_med = np.median(dt)
    return 1.0 / dt_med

def iir_lowpass_1st_order(x: np.ndarray, fs: float, fc: float, y0: float | None = None) -> np.ndarray:
    if fs <= 0 or fc <= 0:
        raise ValueError("fs and fc must be > 0.")
    if fc >= 0.45 * fs:
        raise ValueError("fc is too high for this sampling rate. Keep fc < ~0.45*fs.")

    x = np.asarray(x, dtype=float)
    alpha = (2 * np.pi * fc) / (2 * np.pi * fc + fs)

    y = np.empty_like(x)
    y[0] = x[0] if y0 is None else float(y0)
    for n in range(1, len(x)):
        y[n] = alpha * x[n] + (1.0 - alpha) * y[n - 1]
    return y

Boost_thresh = 5
High_mach_thresh = 5
Mid_mach_thresh = 10
Low_mach_thresh = 10
Coast_thresh = 10
Drogue_thresh = 10
Main_thresh = 5
Recovery_thresh = 20

Tnot = 310.5
l = 0.0065

g = 9.80665

dataset = "Test1.csv" #change this for dataset name
df = pd.read_csv(dataset)
t_col = df["timestamp"].to_numpy()
p_col = df["Pres"].to_numpy() # comment out if altitude column is there
#alt_col = df["Alt"].to_numpy() #uncomment if altitude column is present
a_col = df["AcclY"].to_numpy()
#vair_col = df["Vair"].to_numpy()
Pnot = p_col[0] # Pa at MSL

'''only if altitude isn't available start commenting from here'''

altitude = (Tnot/l)*(1- ((p_col/Pnot)**0.1903))
df["alt"] = altitude

''' run if time in ms
for i in range(1,len(df)):
    t_col[i] = t_col[i]/1000
'''
fs = estimate_fs_from_time(t_col) #sampling frequency
fc = 3 #cutoff frequency
a_smooth = iir_lowpass_1st_order(a_col, fs, fc)
alt_smooth = iir_lowpass_1st_order(altitude, fs, fc)

FSM = ["Standby", "Boost", "Coast_high_mach", "Coast_mid_mach", "Coast_low_mach", "Coast", "Drogue", "Main", "Recovery"]
STATE = "Standby"
count = 0

# ADDED: transition logging containers
transitions = []  # ADDED
state_series = np.empty(len(df), dtype=object)  # ADDED
state_series[0] = STATE  # ADDED

# ADDED: helper to log transitions
def _log_transition(i, prev_state, new_state):  # ADDED
    transitions.append({  # ADDED
        "idx": int(i),  # ADDED
        "time": float(t_col[i]),  # ADDED
        "from": str(prev_state),  # ADDED
        "to": str(new_state),  # ADDED
        "alt": float(alt_smooth[i]),  # ADDED
        "pres": float(p_col[i]),  # ADDED
        "accl": float(a_smooth[i])  # ADDED
    })  # ADDED

#if airspeed is available run this FSM
for i in range (1,len(df)) :
    prev_state = STATE  # ADDED
    match STATE:
        case "Standby":
            if a_smooth[i] > 2*g:
                count += 1
            else:
                count = 0
            if count == Boost_thresh:
                count = 0
                STATE = "Boost"
        case "Boost":
            if a_smooth[i] < 0:
                count += 1
            else:
                count = 0
            if count == Coast_thresh:
                count = 0
                STATE = "Coast"
        case "Coast":
            if alt_smooth[i-1] > alt_smooth[i]:
                count += 1
            else:
                count = 0
            if count == Drogue_thresh:
                count = 0
                STATE = "Drogue"
        case "Drogue":
            if alt_smooth[i] < 457:
                count += 1
            else:
                count = 0
            if count == Main_thresh:
                count = 0
                STATE = "Main"
        case "Main":
            if altitude[i] < 0:
                count += 1
            else:
                count = 0
            if count == Recovery_thresh:
                count = 0
                STATE = "Recovery"
                
    # ADDED: record per-sample state + log any transition
    state_series[i] = STATE  # ADDED
    if STATE != prev_state:  # ADDED
        _log_transition(i, prev_state, STATE)  # ADDED
    
    # ADDED: write state transition log to CSV
    if len(transitions) > 0:  # ADDED
        trans_df = pd.DataFrame(transitions)  # ADDED
        trans_df = trans_df[["idx", "time", "from", "to", "alt", "pres", "accl"]]  # ADDED
        trans_df.to_csv("fsm_state_transitions.csv", index=False)  # ADDED

# ADDED: print transition log as a neat table
if len(transitions) == 0:  # ADDED
    print("No state transitions detected.")  # ADDED
else:  # ADDED
    trans_df = pd.DataFrame(transitions)  # ADDED
    print("\nSTATE TRANSITIONS:\n")  # ADDED
    print(trans_df[["idx", "time", "from", "to", "alt", "pres", "accl"]].to_string(index=False))  # ADDED

# ADDED: pretty plots with transition markers
plt.style.use("seaborn-v0_8-whitegrid")  # ADDED
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(13, 9), constrained_layout=True)  # ADDED

ax0, ax1, ax2 = axes  # ADDED

ax0.plot(t_col, alt_smooth, linewidth=2.2)  # ADDED
ax0.set_ylabel("Altitude (m)")  # ADDED
ax0.set_title("Flight Telemetry with FSM State Transitions")  # ADDED

ax1.plot(t_col, p_col, linewidth=2.2)  # ADDED
ax1.set_ylabel("Pressure (Pa)")  # ADDED

ax2.plot(t_col, a_smooth, linewidth=2.2)  # ADDED
ax2.set_ylabel("Acceleration (m/s²)")  # ADDED
ax2.set_xlabel("Time (s)")  # ADDED

if len(transitions) > 0:  # ADDED
    t_tr = np.array([tr["time"] for tr in transitions], dtype=float)  # ADDED

    for ax in axes:  # ADDED
        for tt in t_tr:  # ADDED
            ax.axvline(tt, linestyle="--", linewidth=1.2, alpha=0.75)  # ADDED

    ax0.scatter(t_tr, [tr["alt"] for tr in transitions], s=55, zorder=5)  # ADDED
    ax1.scatter(t_tr, [tr["pres"] for tr in transitions], s=55, zorder=5)  # ADDED
    ax2.scatter(t_tr, [tr["accl"] for tr in transitions], s=55, zorder=5)  # ADDED

    for tr in transitions:  # ADDED
        label = f'{tr["from"]} → {tr["to"]}'  # ADDED
        ax0.annotate(  # ADDED
            label,  # ADDED
            xy=(tr["time"], tr["alt"]),  # ADDED
            xytext=(8, 10),  # ADDED
            textcoords="offset points",  # ADDED
            fontsize=9,  # ADDED
            bbox=dict(boxstyle="round,pad=0.25", alpha=0.9),  # ADDED
            arrowprops=dict(arrowstyle="->", alpha=0.6)  # ADDED
        )  # ADDED

for ax in axes:  # ADDED
    ax.grid(True, which="major", alpha=0.35)  # ADDED
    ax.margins(x=0.01)  # ADDED

plt.show()  # ADDED

#run if airspeed not available
'''
for i in range (1,len(df)) :
    v_sound = 0.5**(1.4*287.05*(288.15 - 0.0065*altitude[i]))
    mach = vair_col[i]/v_sound
    match STATE:
        case "Standby":
            if a_smooth[i] > 2*g:
                count += 1
            else:
                count = 0
            if count == Boost_thresh:
                count = 0
                STATE = "Boost"
        case "Boost":
            if a_smooth[i] < 0:
                count += 1
            else:
                count = 0
            if count == High_mach_thresh:
                count = 0
                STATE = "Coast_high_mach"
        case "Coast_high_mach":
            if mach < 0.65:
                count += 1
            else:
                count = 0
            if count == Mid_mach_thresh:
                count = 0 
                STATE = "Coast_mid_mach"
        case "Coast_mid_mach":
            if mach < 0.3:
                count += 1
            else:
                count = 0
            if count == Low_mach_thresh:
                count = 0 
                STATE = "Coast_low_mach"
        case "Coast_low_mach": #Apogee Detection
            if h_smooth[i-1] > h_smooth[i]:
                count += 1
            else:
                count = 0
            if count == Drogue_thresh:
                count = 0
                STATE = "Drogue"
        case "Drogue": 
            if h_smooth[i] < 457:
                count += 1
            else:
                count = 0
            if count == Main_thresh:
                count = 0
                STATE = "Main"
        case "Main": 
            if abs(p_col[i] - p_col[i-1]) < 0.5:
                count += 1
            else:
                count = 0
            if count == Recovery_thresh:
                count = 0
                STATE = "Recovery"
'''
