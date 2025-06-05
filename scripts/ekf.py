import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load data correctly
raw = pd.read_csv("../build/oxts_out.csv", parse_dates=["timestamp"])
ekf = pd.read_csv("../build/ekf_out.csv", parse_dates=["timestamp"])

# Ensure numeric conversion for relevant fields
for col in ["ve", "vn"]:
    if col in raw.columns:
        raw[col] = pd.to_numeric(raw[col], errors="coerce")

for col in ["ve", "vn", "ae", "an"]:
    if col in ekf.columns:
        ekf[col] = pd.to_numeric(ekf[col], errors="coerce")

# Drop any rows with NaN values after conversion
raw.dropna(inplace=True)
ekf.dropna(inplace=True)

plt.figure(figsize=(12, 6))

# 2D Trajectory
plt.subplot(2, 2, 1)
plt.plot(raw["lon"], raw["lat"], label="Raw", alpha=0.5)
plt.plot(ekf["lon"], ekf["lat"], label="EKF", linestyle="--")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("2D Trajectory")
plt.legend()
plt.axis("auto")

# Velocity components over time
plt.subplot(2, 2, 2)
plt.plot(raw["timestamp"], raw["ve"], label="Raw VE (East)", alpha=0.5)
plt.plot(ekf["timestamp"], ekf["ve"], label="EKF VE", linestyle="--")
plt.plot(raw["timestamp"], raw["vn"], label="Raw VN (North)", alpha=0.5)
plt.plot(ekf["timestamp"], ekf["vn"], label="EKF VN", linestyle="--")
plt.xlabel("Time")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity Components Over Time")
plt.legend()

# **EKF Earth-Frame Accelerations (Ae & An)**
plt.subplot(2, 2, 3)
plt.plot(ekf["timestamp"], ekf["ax"], label="EKF Ae (East Acceleration)", linestyle="--")
plt.plot(ekf["timestamp"], ekf["ay"], label="EKF An (North Acceleration)", linestyle="--")
plt.xlabel("Time")
plt.ylabel("Acceleration (m/s²)")
plt.title("EKF Earth-Frame Acceleration (Ae & An)")
plt.legend()

# Velocity magnitude over time
def velocity_magnitude(df):
    return np.sqrt(df["ve"]**2 + df["vn"]**2)

plt.subplot(2, 2, 4)
plt.plot(raw["timestamp"], velocity_magnitude(raw), label="Raw Speed", alpha=0.5)
plt.plot(ekf["timestamp"], velocity_magnitude(ekf), label="EKF Speed", linestyle="--")
plt.xlabel("Time")
plt.ylabel("Speed (m/s)")
plt.title("Speed Over Time")
plt.legend()

plt.tight_layout()
plt.savefig("ekf_vs_raw_comparison.png")
plt.show()
