#pragma once

#include <vector>
#include <functional>
#include <mutex>
#include <atomic>

class GuiPlotter {
public:
    GuiPlotter();
    ~GuiPlotter();

    void add_ekf_estimate(double timestamp, const std::vector<double>& values);

    // Run the GUI loop and streaming function
    void run(std::function<void()> stream_func);

private:
    // Plot data
    std::vector<double> ts_;
    std::vector<double> ts_ekf_, xs_ekf_, ys_ekf_;

    std::mutex mtx_;
    std::atomic<bool> running_;
    std::atomic<bool> zoom_out_full_;
    std::atomic<bool> zoom_stabilized_{false};
};