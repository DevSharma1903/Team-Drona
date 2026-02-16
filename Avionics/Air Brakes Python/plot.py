import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("Velocity_EKF.csv")

plt.figure(figsize=(10,5))
plt.plot(df["timestamp"], df["alt"], linewidth=2)

plt.xlabel("Time")
plt.ylabel("Altitude")
plt.title("Altitude vs Time (no air brakes)")
plt.grid(True)
plt.tight_layout()
plt.show()
