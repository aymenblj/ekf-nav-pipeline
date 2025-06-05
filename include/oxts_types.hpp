#pragma once

#include <string>
#include <cmath>

namespace utils {

struct OXTSData {
    long double lat, lon, alt;
    long double roll, pitch, yaw;
    long double vn, ve, vf, vl, vu;
    long double ax, ay, az;
    long double af, al, au;
    long double wx, wy, wz;
    long double wf, wl, wu;
    long double pos_accuracy;
    long double vel_accuracy;
    int navstat;
    int numsats;
    int posmode;
    int velmode;
    int orimode;
    std::string timestamp;

    // Check if velocity data is available
    bool hasVelocity() const {
        return !std::isnan(ve) && !std::isnan(vn);
    }

    // Check if acceleration data is available
    bool hasAcceleration() const {
        return !std::isnan(ax) && !std::isnan(ay);
    }

    // Check if GPS is available
    bool hasGPS() const {
        return !std::isnan(lat) && !std::isnan(lon);
    }
};

} // namespace utils
