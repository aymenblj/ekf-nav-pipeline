#pragma once
#include "utils/oxts_parser.hpp"
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>

class GuiPlotter {
public:
    GuiPlotter();
    ~GuiPlotter();

    void startGui();
    void handleData(const utils::OXTSData& data, const Eigen::VectorXd& ekf_state);
    void realtimeSleep(double timelapse);
    void stopGui();

private:
    struct PlotData {
        std::vector<double> xs, ys, vs;
    };

    std::mutex mtx_;
    PlotData ekf_data_, meas_data_;
    bool origin_set_ = false;
    double lat0_ = 0.0, lon0_ = 0.0;
    std::atomic<bool> running_{false};
    std::thread gui_thread_;

    // Real-time playback state
    bool realtime_first_ = true;
    double realtime_t0_ = 0.0;
    double realtime_last_timelapse_ = -1.0;
    std::chrono::steady_clock::time_point realtime_wall_start_;

    void guiLoop();
};