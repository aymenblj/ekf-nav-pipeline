#pragma once

#include "utils/oxts_parser.hpp"
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>

/**
 * @class GuiPlotter
 * @brief Handles plotting and visualization of EKF and GNSS data in real time.
 *
 * This class manages the plotting of estimated trajectories, speed, traveled distance,
 * GNSS outage events, and related statistics. It runs the GUI in a separate thread,
 * maintains synchronization, and supports real-time playback.
 */
class GuiPlotter {
public:
    GuiPlotter();
    ~GuiPlotter();

    void startGui();
    void handleData(const utils::OXTSData& data, const Eigen::VectorXd& ekf_state);
    void realtimeSleep(double timelapse);
    void stopGui();
    bool isRunning() const { return running_; }

private:
    std::mutex mtx_;
    std::atomic<bool> running_{false};
    std::thread gui_thread_;

    struct PlotData {
        std::vector<double> xs; ///< X coordinates in meters (local frame).
        std::vector<double> ys; ///< Y coordinates in meters (local frame).
        std::vector<double> vs; ///< Speeds in km/h.
    };

    PlotData ekf_data_;
    PlotData meas_data_;

    bool origin_set_ = false;
    double lat0_ = 0.0;
    double lon0_ = 0.0;

    bool realtime_first_ = true;
    double realtime_t0_ = 0.0;
    double realtime_last_timelapse_ = -1.0;
    std::chrono::steady_clock::time_point realtime_wall_start_;

    double traveled_distance_ = 0.0;

    // GNSS outage handling
    bool gnss_outage_active_ = false;
    bool prev_gnss_nan_ = false;
    double outage_distance_ = 0.0;
    size_t outage_start_idx_ = 0;
    double last_outage_shift_ = 0.0;
    double last_outage_distance_ = 0.0;
    double last_ekf_x_in_outage_ = 0.0, last_ekf_y_in_outage_ = 0.0;

    void guiLoop();
};