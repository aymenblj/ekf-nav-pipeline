# NavStream

NavStream is a C++ Extended Kalman Filter (EKF) framework for vehicle localization using GNSS + IMU fusion, with robust dead-reckoning during GNSS outages. This EKF estimates position, velocity, and attitude from real-world sensor logs (e.g., KITTI dataset OXTS), supporting configurable prediction models and noise parameters.

---

## Build Instructions

```sh
cmake -S . -B build -G Ninja
cmake --build build
```

---

## Run Example

```sh
.\NavStream.exe ../data/2011_09_26/2011_09_26_drive_0013_gps_loss/oxts oxts_out.csv ekf_out.csv
.\NavStream.exe ../data/2011_09_26/2011_09_26_drive_0001_unsync/oxts oxts_out.csv ekf_out.csv
.\NavStream.exe ../data/2011_10_03/2011_10_03_drive_0042_unsync_gps_loss/oxts oxts_out.csv ekf_out.csv
```

- **First argument:** Path to OXTS directory (e.g., from KITTI dataset)
- **Second argument:** Output CSV for raw OXTS data
- **Third argument:** Output CSV for EKF state trajectory

---

## Core Features

### 1. Prediction Model

NavStream uses a physically-motivated, 2D kinematic vehicle model for the EKF state propagation. The state vector includes:

- Latitude, Longitude (position)
- Velocity (forward)
- Yaw (heading)
- Forward acceleration
- Yaw rate

**Prediction step:**  
At each time step, the EKF predicts the new state using the previous velocity, yaw, and measured IMU inputs (acceleration, yaw rate):

- Position is updated using dead-reckoning:  
  - `lat += (v * sin(yaw) * dt) / kLatToMeters`
  - `lon += (v * cos(yaw) * dt) / lonToMeters(lat, lon)`
- Velocity and yaw are updated using IMU acceleration/yaw-rate measurements.

**Why this model?**  
- It balances accuracy and simplicity, capturing primary vehicle dynamics while remaining computationally efficient.
- Works well for typical automotive scenarios, especially with noisy or intermittent GNSS.
- Supports seamless dead-reckoning during GNSS loss, using only IMU data.

### 2. Noise Parameters

Noise parameters are fully configurable for both process and measurement models:

- **Process noise:**  
  - Controls how much the filter "trusts" the model between measurements.
  - Tuning allows filtering to be robust to real-world sensor drift and model mismatch.

- **Measurement noise:**  
  - Set per-sensor (GNSS, velocity, yaw, etc.)
  - Allows weighting each sensor by its expected accuracy.

**Design rationale:**  
- Noise parameters can be easily adapted to different hardware, environments, or sensor qualities.
- Proper tuning ensures smooth fusion, fast convergence, and reliable dead-reckoning.

### 3. GNSS + IMU Fusion

- **GNSS (latitude, longitude):** Corrects drift and long-term bias, provides global reference.
- **IMU (acceleration, yaw rate):** Enables high-rate propagation and dead-reckoning when GNSS is unavailable or degraded.
- **Velocity and yaw measurements:** Fused separately if available, improving robustness.

The EKF framework fuses all available measurements in a principled way, yielding high-accuracy localization even during GNSS dropouts.

### 4. Dead-Reckoning Robustness

When GNSS is lost or degraded, the EKF prediction model uses only IMU and last known state to propagate position and heading. This maintains stable and plausible trajectories until GNSS resumes, minimizing jumps or drift.

---

## Source Overview

- **ekf/ekf_interface.hpp** – Abstract EKF interface
- **ekf/ekf_vehicleModel.hpp** – Vehicle kinematic EKF implementation
- **utils/oxts_parser.hpp** – KITTI OXTS data parser
- **utils/logger.hpp** – CSV logger for trajectory/results
- **utils/utm_converter.hpp** – Coordinate transformations

---

## References

- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/)
- [Probabilistic Robotics (Thrun, Burgard, Fox)](https://probabilistic-robotics.org/)
- [Kalman Filtering: Theory and Practice](https://en.wikipedia.org/wiki/Kalman_filter)

---

## License

MIT License.

---