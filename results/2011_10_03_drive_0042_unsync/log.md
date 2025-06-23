**Test:** Dead reckoning—long straight run vs. 360° turn (no GNSS)

---

## Observations

- **Long straight run at high speed (no GNSS):**
  - Noticeable position shift (drag/offset) accumulated over time.
  - Yaw estimation remains consistent and precise.
  - Observation aligns with literature: linear dead reckoning accumulates more error due to uncompensated velocity bias and integration drift.

- **360° turn at moderate speed (no GNSS):**
  - No noticeable shift. Position returns close to the starting point.
  - Suggests that rotational maneuvers and repeated turns reduce cumulative drift in position and heading.

---

## Notes

- Test performed without GNSS; pure dead reckoning.
- Straight high-speed runs show noticeable position drag/offset.
- Moderate-speed rotations (360°) do not show drift; yaw estimation is reliable.

**Suggestions:**  
- Verify wheel speed and odometry updates during GNSS loss, as uncorrected biases lead to position drift.
- Observation matches established IMU dead reckoning error characteristics: heading (yaw) is typically more stable than integrated velocity/position in ground vehicles.

---
**References:**  
- Titterton, D.H., & Weston, J.L. (2004). Strapdown Inertial Navigation Technology.  
- Groves, P.D. (2013). Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems.  
