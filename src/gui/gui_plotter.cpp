#include "gui/gui_plotter.hpp"
#include "utils/utm_converter.hpp"
#include <imgui.h>
#include <implot.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

// --- Utility for zoom ---
namespace {
double get_zoom_from_speed(double speed_kmh, double min_zoom = 40.0, double max_zoom = 250.0) {
    if (std::isnan(speed_kmh) || speed_kmh < 0.0) speed_kmh = 0.0;
    if (speed_kmh > 130.0) speed_kmh = 130.0;
    double alpha = speed_kmh / 130.0;
    return min_zoom + (max_zoom - min_zoom) * (alpha * alpha);
}
}

GuiPlotter::GuiPlotter() {}
GuiPlotter::~GuiPlotter() {
    stopGui();
}

void GuiPlotter::startGui() {
    running_ = true;
    gui_thread_ = std::thread(&GuiPlotter::guiLoop, this);
}

void GuiPlotter::stopGui() {
    running_ = false;
    // Only join if not in the same thread!
    if (gui_thread_.joinable() && std::this_thread::get_id() != gui_thread_.get_id())
        gui_thread_.join();
}

void GuiPlotter::handleData(const utils::OXTSData& data, const Eigen::VectorXd& ekf_state) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!origin_set_) {
        lat0_ = ekf_state(0);
        lon0_ = ekf_state(1);
        origin_set_ = true;
    }
    auto [ekf_x, ekf_y] = utils::UTMConverter::latlonToLocalXY(ekf_state(0), ekf_state(1), lat0_, lon0_);
    double ekf_v_kmh = ekf_state(2) * 3.6;
    ekf_data_.xs.push_back(ekf_x);
    ekf_data_.ys.push_back(ekf_y);
    ekf_data_.vs.push_back(ekf_v_kmh);

    auto [meas_x, meas_y] = utils::UTMConverter::latlonToLocalXY(data.lat, data.lon, lat0_, lon0_);
    double gnss_v_kmh = std::hypot(data.ve, data.vn) * 3.6;
    meas_data_.xs.push_back(meas_x);
    meas_data_.ys.push_back(meas_y);
    meas_data_.vs.push_back(gnss_v_kmh);
}

void GuiPlotter::realtimeSleep(double timelapse) {
    if (realtime_first_) {
        realtime_t0_ = timelapse;
        realtime_last_timelapse_ = timelapse;
        realtime_wall_start_ = std::chrono::steady_clock::now();
        realtime_first_ = false;
    } else {
        double dt = timelapse - realtime_last_timelapse_;
        if (dt > 0) {
            auto target = realtime_wall_start_ + std::chrono::duration<double>(timelapse - realtime_t0_);
            std::this_thread::sleep_until(target);
        }
        realtime_last_timelapse_ = timelapse;
    }
}

// THIS LOOP WILL EXIT IF: user closes window, or main thread sets running_ to false.
void GuiPlotter::guiLoop() {
    if (!glfwInit()) {
        std::cerr << "[GuiPlotter] Failed to initialize GLFW.\n";
        return;
    }
    GLFWwindow* window = glfwCreateWindow(1600, 1000, "NavPlotterPro", nullptr, nullptr);
    if (!window) {
        std::cerr << "[GuiPlotter] Failed to create GLFW window.\n";
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    bool zoom_out_full = false;

    // --- Main GUI Loop ---
    while (running_) {
        glfwPollEvents();
        if (glfwWindowShouldClose(window)) {
            running_ = false; // break out of the loop if user closes window!
            break;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        ImGui::SetNextWindowSize(ImVec2((float)display_w, (float)display_h), ImGuiCond_Always);
        ImGui::Begin("Trajectory Plot", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        PlotData ekf, meas;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            ekf = ekf_data_;
            meas = meas_data_;
        }

        ImGui::Text("EKF Points: %zu", ekf.xs.size());
        ImGui::Text("Measurement Points: %zu", meas.xs.size());

        double speed = 0.0;
        if (!ekf.vs.empty() && std::isfinite(ekf.vs.back()))
            speed = ekf.vs.back();
        else if (!meas.vs.empty() && std::isfinite(meas.vs.back()))
            speed = meas.vs.back();
        if (speed > 0)
            ImGui::Text("EKF Speed: %.2f km/h", speed);
        else
            ImGui::Text("Speed: N/A");

        // --- Quit button for user ---
        if (ImGui::Button("Quit")) {
            glfwSetWindowShouldClose(window, true);
        }

        if ((!ekf.xs.empty() && !ekf.ys.empty()) || (!meas.xs.empty() && !meas.ys.empty())) {
            ImVec2 plot_size = ImGui::GetContentRegionAvail();

            if (ImPlot::BeginPlot("##Trajectory", plot_size)) {
                ImPlot::SetupAxes("X [m]", "Y [m]");
                if (zoom_out_full) {
                    double x_min = INFINITY, x_max = -INFINITY, y_min = INFINITY, y_max = -INFINITY;
                    auto fit_bounds = [&](const std::vector<double>& xs, const std::vector<double>& ys) {
                        if (!xs.empty()) {
                            auto [min_x, max_x] = std::minmax_element(xs.begin(), xs.end());
                            auto [min_y, max_y] = std::minmax_element(ys.begin(), ys.end());
                            x_min = std::min(x_min, *min_x); x_max = std::max(x_max, *max_x);
                            y_min = std::min(y_min, *min_y); y_max = std::max(y_max, *max_y);
                        }
                    };
                    fit_bounds(ekf.xs, ekf.ys);
                    fit_bounds(meas.xs, meas.ys);

                    double x_pad = std::max(2.0, (x_max - x_min) * 0.10);
                    double y_pad = std::max(2.0, (y_max - y_min) * 0.10);
                    ImPlot::SetupAxisLimits(ImAxis_X1, x_min - x_pad, x_max + x_pad, ImGuiCond_Always);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, y_min - y_pad, y_max + y_pad, ImGuiCond_Always);
                } else {
                    constexpr double min_zoom = 40.0;
                    constexpr double max_zoom = 250.0;
                    double zoom = get_zoom_from_speed(speed, min_zoom, max_zoom);
                    if (!ekf.xs.empty()) {
                        double cx = ekf.xs.back(), cy = ekf.ys.back();
                        ImPlot::SetupAxisLimits(ImAxis_X1, cx - zoom / 2, cx + zoom / 2, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, cy - zoom / 2, cy + zoom / 2, ImGuiCond_Always);
                    } else if (!meas.xs.empty()) {
                        double cx = meas.xs.back(), cy = meas.ys.back();
                        ImPlot::SetupAxisLimits(ImAxis_X1, cx - zoom / 2, cx + zoom / 2, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, cy - zoom / 2, cy + zoom / 2, ImGuiCond_Always);
                    }
                }

                if (!ekf.xs.empty() && !ekf.ys.empty())
                    ImPlot::PlotLine("EKF", ekf.xs.data(), ekf.ys.data(), (int)ekf.xs.size());
                if (!meas.xs.empty() && !meas.ys.empty())
                    ImPlot::PlotScatter("Measurement", meas.xs.data(), meas.ys.data(), (int)meas.xs.size());

                // Draw sedan at latest EKF position
                if (!ekf.xs.empty() && !ekf.ys.empty()) {
                    double cx = ekf.xs.back(), cy = ekf.ys.back();
                    double theta = 0.0;
                    if (ekf.xs.size() >= 2) {
                        double dx = ekf.xs.back() - ekf.xs[ekf.xs.size()-2];
                        double dy = ekf.ys.back() - ekf.ys[ekf.ys.size()-2];
                        if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6)
                            theta = std::atan2(dy, dx);
                    }
                    constexpr double L = 4.5, W = 1.8, hl = L/2.0, hw = W/2.0;
                    struct Pt { double x, y; };
                    Pt corners[4] = {
                        {+hl, +hw}, {+hl, -hw}, {-hl, -hw}, {-hl, +hw}
                    };
                    ImVec2 poly[4];
                    for (int i = 0; i < 4; ++i) {
                        double wx = cx + (corners[i].x * std::cos(theta) - corners[i].y * std::sin(theta));
                        double wy = cy + (corners[i].x * std::sin(theta) + corners[i].y * std::cos(theta));
                        poly[i] = ImPlot::PlotToPixels(wx, wy);
                    }
                    ImDrawList* draw_list = ImPlot::GetPlotDrawList();
                    draw_list->AddConvexPolyFilled(poly, 4, IM_COL32(30, 144, 255, 180));
                    draw_list->AddPolyline(poly, 4, IM_COL32(30, 144, 255, 255), true, 2.0f);
                    ImVec2 nose_center = ImPlot::PlotToPixels(
                        cx + hl * std::cos(theta),
                        cy + hl * std::sin(theta)
                    );
                    ImVec2 car_center = ImPlot::PlotToPixels(cx, cy);
                    draw_list->AddLine(car_center, nose_center, IM_COL32(0,0,0,255), 3.0f);
                }
                ImPlot::EndPlot();
            }
        } else {
            ImGui::Text("No EKF or measurement data received yet.");
        }
        ImGui::End();

        ImGui::Render();
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}