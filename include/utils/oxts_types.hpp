#pragma once

#include <string>
#include <cmath>
#include <limits>

/**
 * @file oxts_types.hpp
 * @brief Defines the OXTSData structure for holding parsed OXTS (e.g., KITTI) sensor data.
 */

namespace utils {

/**
 * @struct OXTSData
 * @brief Holds all sensor and navigation data for a single OXTS record.
 *
 * All members are initialized per-record from OXTS log files.
 *
 * Fields:
 * - lat, lon, alt: Latitude, longitude (degrees), altitude (meters)
 * - roll, pitch, yaw: Orientation (radians)
 * - vn, ve, vf, vl, vu: Velocities (north, east, forward, left, up) in m/s
 * - ax, ay, az: Accelerations (X, Y, Z) in m/s^2
 * - af, al, au: Accelerations (forward, left, up) in m/s^2
 * - wx, wy, wz: Angular rates (X, Y, Z) in rad/s
 * - wf, wl, wu: Angular rates (forward, left, up) in rad/s
 * - pos_accuracy, vel_accuracy: Position and velocity accuracy estimates
 * - navstat, numsats, posmode, velmode, orimode: Status and mode fields
 * - timestamp: Timestamp string for this record
 */
struct OXTSData {
    long double lat = std::numeric_limits<long double>::quiet_NaN();
    long double lon = std::numeric_limits<long double>::quiet_NaN();
    long double alt = std::numeric_limits<long double>::quiet_NaN();

    long double roll = std::numeric_limits<long double>::quiet_NaN();
    long double pitch = std::numeric_limits<long double>::quiet_NaN();
    long double yaw = std::numeric_limits<long double>::quiet_NaN();

    long double vn = std::numeric_limits<long double>::quiet_NaN();
    long double ve = std::numeric_limits<long double>::quiet_NaN();
    long double vf = std::numeric_limits<long double>::quiet_NaN();
    long double vl = std::numeric_limits<long double>::quiet_NaN();
    long double vu = std::numeric_limits<long double>::quiet_NaN();

    long double ax = std::numeric_limits<long double>::quiet_NaN();
    long double ay = std::numeric_limits<long double>::quiet_NaN();
    long double az = std::numeric_limits<long double>::quiet_NaN();

    long double af = std::numeric_limits<long double>::quiet_NaN();
    long double al = std::numeric_limits<long double>::quiet_NaN();
    long double au = std::numeric_limits<long double>::quiet_NaN();

    long double wx = std::numeric_limits<long double>::quiet_NaN();
    long double wy = std::numeric_limits<long double>::quiet_NaN();
    long double wz = std::numeric_limits<long double>::quiet_NaN();

    long double wf = std::numeric_limits<long double>::quiet_NaN();
    long double wl = std::numeric_limits<long double>::quiet_NaN();
    long double wu = std::numeric_limits<long double>::quiet_NaN();

    long double pos_accuracy = std::numeric_limits<long double>::quiet_NaN();
    long double vel_accuracy = std::numeric_limits<long double>::quiet_NaN();

    int navstat = 0;
    int numsats = 0;
    int posmode = 0;
    int velmode = 0;
    int orimode = 0;

    double timelapse = 0;
    std::string timestamp;

    /**
     * @brief Check if velocity data is available (ve/vn).
     * @return True if both east and north velocities are valid.
     */
    [[nodiscard]]
    bool hasVelocity() const noexcept {
        return !std::isnan(ve) && !std::isnan(vn);
    }

    /**
     * @brief Check if acceleration data is available (ax/ay).
     * @return True if both X and Y accelerations are valid.
     */
    [[nodiscard]]
    bool hasAcceleration() const noexcept {
        return !std::isnan(ax) && !std::isnan(ay);
    }

    /**
     * @brief Check if GPS data is available (lat/lon).
     * @return True if both latitude and longitude are valid.
     */
    [[nodiscard]]
    bool hasGPS() const noexcept {
        return !std::isnan(lat) && !std::isnan(lon);
    }
};
} // namespace utils