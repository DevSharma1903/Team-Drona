import matplotlib.pyplot as plt


def plot_run(logs, title="Altitude vs Time (Ran on nominal conditions)"):
    t = logs["t"]
    h = logs["h"]
    u_lag = logs["u_lag"]
    u_cmd = logs["u_cmd"]

    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    plt.plot(t, h)
    plt.ylabel("Altitude (m)")
    plt.title(title)
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(t, u_lag, label="u_lag")
    plt.plot(t, u_cmd, "--", label="u_cmd")
    plt.ylabel("Airbrake u")
    plt.xlabel("Time (s)")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()
