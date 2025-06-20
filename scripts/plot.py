import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- Load Data ---
raw = pd.read_csv("../build/oxts_out.csv", parse_dates=["timestamp"])
ekf = pd.read_csv("../build/ekf_out.csv", parse_dates=["timestamp"])

# raw = raw.iloc[:40]
# ekf = ekf.iloc[:40]

# --- Calculate raw speed magnitude ---
if "ve" in raw.columns and "vn" in raw.columns:
    raw["speed"] = np.sqrt(raw["ve"]**2 + raw["vn"]**2)
else:
    raw["speed"] = np.nan

plt.figure(figsize=(16, 12))  # Bigger figure for 6 subplots

# 1. 2D Trajectory (lon vs lat)
plt.subplot(3, 2, 1)
plt.plot(raw["lon"], raw["lat"], label="Raw OXTS (lon,lat)", linestyle="--", alpha=0.8)
plt.plot(ekf["lon"], ekf["lat"], label="EKF Estimate (lon,lat)", linestyle="-", linewidth=0.8)
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("2D Trajectory")
plt.legend()
plt.axis("auto")

# 2. Velocity: raw (ve/vn/speed), EKF (v)
plt.subplot(3, 2, 2)
plt.plot(raw["timestamp"], raw["speed"], label="Raw Speed", linestyle="--", alpha=0.8, color="green")
plt.plot(ekf["timestamp"], ekf["v"], label="EKF Speed", linestyle="-", linewidth=0.8, color="red")
plt.xlabel("Time")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity Components & Speed")
plt.legend()

# 3. Yaw comparison
plt.subplot(3, 2, 3)
plt.plot(raw["timestamp"], raw["yaw"], label="Raw Yaw", linestyle="--", alpha=0.8)
plt.plot(ekf["timestamp"], ekf["yaw"], label="EKF Yaw", linestyle="-", linewidth=0.8)
plt.xlabel("Time")
plt.ylabel("Yaw (rad)")
plt.title("Yaw Angle Over Time")
plt.legend()

# 4. Acceleration (af) and yaw rate (wz)
plt.subplot(3, 2, 4)
plt.plot(raw["timestamp"], raw["af"], label="Raw af (Forward Accel)", linestyle="--", alpha=0.8)
plt.plot(ekf["timestamp"], ekf["af"], label="EKF af (Forward Accel)", linestyle="-", linewidth=0.8)
plt.plot(raw["timestamp"], raw["wz"], label="Raw Wz (Yaw Rate)", linestyle="--", alpha=0.8)
plt.plot(ekf["timestamp"], ekf["wz"], label="EKF Wz (Yaw Rate)", linestyle="-", linewidth=0.8)
plt.xlabel("Time")
plt.ylabel("Accel (m/s²) / Yaw Rate (rad/s)")
plt.title("Forward Acceleration & Yaw Rate")
plt.legend()

# 5. Latitude vs Time
plt.subplot(3, 2, 5)
plt.plot(raw["timestamp"], raw["lat"], label="Raw Latitude", linestyle="--", alpha=0.8)
plt.plot(ekf["timestamp"], ekf["lat"], label="EKF Latitude", linestyle="-", linewidth=0.8)
plt.xlabel("Time")
plt.ylabel("Latitude")
plt.title("Latitude Over Time")
plt.legend()

# 6. Longitude vs Time
plt.subplot(3, 2, 6)
plt.plot(raw["timestamp"], raw["lon"], label="Raw Longitude", linestyle="--", alpha=0.8)
plt.plot(ekf["timestamp"], ekf["lon"], label="EKF Longitude", linestyle="-", linewidth=0.8)
plt.xlabel("Time")
plt.ylabel("Longitude")
plt.title("Longitude Over Time")
plt.legend()

plt.tight_layout()
plt.savefig("../build/ekf_vs_raw_comparison.png", dpi=300)
plt.show()
