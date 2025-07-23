#include "gui/gui_plotter.hpp"
#include "gui/road_map.hpp"
#include "io/utm_converter.hpp"
#include <imgui.h>
#include <implot.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

static RoadMap road_map_;
static bool road_map_loaded_ = false;
static bool road_map_converted_ = false;

namespace {
float get_zoom_from_speed(float speed_kmh, float min_zoom = 40.0, float max_zoom = 250.0) {
    if (std::isnan(speed_kmh) || speed_kmh < 0.0) speed_kmh = 0.0;
    if (speed_kmh > 130.0) speed_kmh = 130.0;
    float alpha = speed_kmh / 130.0;
    return min_zoom + (max_zoom - min_zoom) * (alpha * alpha);
}
}

GuiPlotter::GuiPlotter() {
    if (!road_map_.loadFromJson("../scripts/roads.json")) {
        std::cerr << "[GuiPlotter] Failed to load scripts/roads.json\n";
        road_map_loaded_ = false;
    } else {
        std::cerr << "[GuiPlotter] Loaded " << road_map_.getPolylines().size() << " polylines from roads.json\n";
        road_map_loaded_ = true;
    }
}
GuiPlotter::~GuiPlotter() {
    stopGui();
}

void GuiPlotter::startGui() {
    running_ = true;
    gui_thread_ = std::thread(&GuiPlotter::guiLoop, this);
}

void GuiPlotter::stopGui() {
    running_ = false;
    if (gui_thread_.joinable() && std::this_thread::get_id() != gui_thread_.get_id())
        gui_thread_.join();
}

void GuiPlotter::handleData(const io::OXTSData& data, const Eigen::VectorXf& ekf_state) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!origin_set_) {
        lat0_ = ekf_state(0);
        lon0_ = ekf_state(1);
        origin_set_ = true;

        // Only convert ONCE after origin is set
        if (road_map_loaded_ && !road_map_converted_) {
            road_map_.convertAllToLocalXY(lat0_, lon0_);
            road_map_converted_ = true;
            std::cerr << "[GuiPlotter] Road map converted to local XY using origin (lat,lon): (" << lat0_ << ", " << lon0_ << ")\n";
        }
    }
    auto [ekf_x, ekf_y] = io::UTMConverter::latlonToLocalXY(ekf_state(0), ekf_state(1), lat0_, lon0_);
    float ekf_v_kmh = ekf_state(2) * 3.6;
    auto [meas_x, meas_y] = io::UTMConverter::latlonToLocalXY(data.lat, data.lon, lat0_, lon0_);
    float gnss_v_kmh = std::hypot(data.ve, data.vn) * 3.6;

    // --- Lever Arm Correction Parameters ---
    constexpr float lever_dx = 0.85;   // forward (meters)
    constexpr float lever_dy = 4.0;    // left (positive Y)

    // --- Compute heading (yaw) from EKF state ---
    float heading = 0.0;
    if (ekf_state.size() > 3) heading = ekf_state(3);
    else if (ekf_data_.xs.size() >= 2) {
        float dx = ekf_x - ekf_data_.xs.back();
        float dy = ekf_y - ekf_data_.ys.back();
        if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6)
            heading = std::atan2(dy, dx);
    }

    // --- Rotate lever arm to world frame ---
    float dx_world =  lever_dx * std::cos(heading) - lever_dy * std::sin(heading);
    float dy_world =  lever_dx * std::sin(heading) + lever_dy * std::cos(heading);

    // --- Apply lever arm correction to EKF and GNSS ---
    float ekf_x_corr = ekf_x + dx_world;
    float ekf_y_corr = ekf_y + dy_world;
    float meas_x_corr = meas_x + dx_world;
    float meas_y_corr = meas_y + dy_world;

    // Accumulate traveled distance
    if (!ekf_data_.xs.empty()) {
        float last_x = ekf_data_.xs.back();
        float last_y = ekf_data_.ys.back();
        float dx = ekf_x_corr - last_x;
        float dy = ekf_y_corr - last_y;
        traveled_distance_ += std::hypot(dx, dy);

        // During GNSS outage, accumulate outage distance
        if (gnss_outage_active_)
            outage_distance_ += std::hypot(dx, dy);
    }

    ekf_data_.xs.push_back(ekf_x_corr);
    ekf_data_.ys.push_back(ekf_y_corr);
    ekf_data_.vs.push_back(ekf_v_kmh);

    meas_data_.xs.push_back(meas_x_corr);
    meas_data_.ys.push_back(meas_y_corr);
    meas_data_.vs.push_back(gnss_v_kmh);

    // --- GNSS outage detection and shift calculation ---
    bool gnss_nan = std::isnan(data.lat) || std::isnan(data.lon);

if (!prev_gnss_nan_ && gnss_nan) {
    // Outage just started
    gnss_outage_active_ = true;
    outage_distance_ = 0.0;
    outage_start_idx_ = ekf_data_.xs.size() - 1;
}
else if (gnss_outage_active_ && gnss_nan) {
    // Outage is ongoing, keep updating the last EKF pos in outage
    last_ekf_x_in_outage_ = ekf_x_corr;
    last_ekf_y_in_outage_ = ekf_y_corr;
}
else if (prev_gnss_nan_ && !gnss_nan) {
    // Outage just ended, calculate shift
    gnss_outage_active_ = false;
    last_outage_shift_ = std::hypot(
        ekf_x_corr - last_ekf_x_in_outage_,
        ekf_y_corr - last_ekf_y_in_outage_
    );
    last_outage_distance_ = outage_distance_;
}
prev_gnss_nan_ = gnss_nan;
}

void GuiPlotter::realtimeSleep(float timelapse) {
    if (realtime_first_) {
        realtime_t0_ = timelapse;
        realtime_last_timelapse_ = timelapse;
        realtime_wall_start_ = std::chrono::steady_clock::now();
        realtime_first_ = false;
    } else {
        float dt = timelapse - realtime_last_timelapse_;
        if (dt > 0) {
            auto target = realtime_wall_start_ + std::chrono::duration<float>(timelapse - realtime_t0_);
            std::this_thread::sleep_until(target);
        }
        realtime_last_timelapse_ = timelapse;
    }
}

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

    while (running_) {
        glfwPollEvents();
        if (glfwWindowShouldClose(window)) {
            running_ = false;
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

        float speed = 0.0;
        if (!ekf.vs.empty() && std::isfinite(ekf.vs.back()))
            speed = ekf.vs.back();
        else if (!meas.vs.empty() && std::isfinite(meas.vs.back()))
            speed = meas.vs.back();
        if (speed > 0)
            ImGui::Text("EKF Speed: %.2f km/h", speed);
        else
            ImGui::Text("Speed: N/A");

        ImGui::Text("Traveled distance: %.2f m", traveled_distance_);

        if (last_outage_distance_ > 0.0) {
            float pct = last_outage_distance_ > 0.0 ? 100.0 * last_outage_shift_ / last_outage_distance_ : 0.0;
            ImGui::Text("Last GNSS outage: shift = %.2f m, distance during outage = %.2f m, shift %% = %.2f%%", 
                last_outage_shift_, last_outage_distance_, pct);
        } else {
            ImGui::Text("Last GNSS outage: N/A");
        }

        if (ImGui::Button("Quit")) {
            glfwSetWindowShouldClose(window, true);
        }

        if ((!ekf.xs.empty() && !ekf.ys.empty()) || (!meas.xs.empty() && !meas.ys.empty())) {
            ImVec2 plot_size = ImGui::GetContentRegionAvail();

            if (ImPlot::BeginPlot("##Trajectory", plot_size)) {
                ImPlot::SetupAxes("X [m]", "Y [m]");
                if (zoom_out_full) {
                    float x_min = INFINITY, x_max = -INFINITY, y_min = INFINITY, y_max = -INFINITY;
                    auto fit_bounds = [&](const std::vector<float>& xs, const std::vector<float>& ys) {
                        if (!xs.empty()) {
                            auto [min_x, max_x] = std::minmax_element(xs.begin(), xs.end());
                            auto [min_y, max_y] = std::minmax_element(ys.begin(), ys.end());
                            x_min = std::min(x_min, *min_x); x_max = std::max(x_max, *max_x);
                            y_min = std::min(y_min, *min_y); y_max = std::max(y_max, *max_y);
                        }
                    };
                    fit_bounds(ekf.xs, ekf.ys);
                    fit_bounds(meas.xs, meas.ys);

                    float x_pad = std::max(2.0, (x_max - x_min) * 0.10);
                    float y_pad = std::max(2.0, (y_max - y_min) * 0.10);
                    ImPlot::SetupAxisLimits(ImAxis_X1, x_min - x_pad, x_max + x_pad, ImGuiCond_Always);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, y_min - y_pad, y_max + y_pad, ImGuiCond_Always);
                } else {
                    constexpr float min_zoom = 40.0;
                    constexpr float max_zoom = 250.0;
                    float zoom = get_zoom_from_speed(speed, min_zoom, max_zoom);
                    if (!ekf.xs.empty()) {
                        float cx = ekf.xs.back(), cy = ekf.ys.back();
                        ImPlot::SetupAxisLimits(ImAxis_X1, cx - zoom / 2, cx + zoom / 2, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, cy - zoom / 2, cy + zoom / 2, ImGuiCond_Always);
                    } else if (!meas.xs.empty()) {
                        float cx = meas.xs.back(), cy = meas.ys.back();
                        ImPlot::SetupAxisLimits(ImAxis_X1, cx - zoom / 2, cx + zoom / 2, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, cy - zoom / 2, cy + zoom / 2, ImGuiCond_Always);
                    }
                }

                // Plot background road map if loaded
                if (road_map_loaded_) {
                    for (size_t p = 0; p < road_map_.getPolylines().size(); ++p) {
                        const auto& poly = road_map_.getPolylines()[p];
                        if (poly.size() < 2) continue;
                        std::vector<float> xs, ys;
                        xs.reserve(poly.size());
                        ys.reserve(poly.size());
                        for (auto [x, y] : poly) {
                            xs.push_back(x);
                            ys.push_back(y);
                        }
                        ImPlot::SetNextLineStyle(ImVec4(1,1,1,1)); // white
                        ImPlot::PlotLine(("Road" + std::to_string(p)).c_str(), xs.data(), ys.data(), (int)xs.size());
                    }
                }

                if (!ekf.xs.empty() && !ekf.ys.empty())
                    ImPlot::PlotLine("EKF", ekf.xs.data(), ekf.ys.data(), (int)ekf.xs.size());
                if (!meas.xs.empty() && !meas.ys.empty())
                    ImPlot::PlotScatter("Measurement", meas.xs.data(), meas.ys.data(), (int)meas.xs.size());

                // Draw sedan at latest EKF position
                if (!ekf.xs.empty() && !ekf.ys.empty()) {
                    float cx = ekf.xs.back(), cy = ekf.ys.back();
                    float theta = 0.0;
                    if (ekf.xs.size() >= 2) {
                        float dx = ekf.xs.back() - ekf.xs[ekf.xs.size()-2];
                        float dy = ekf.ys.back() - ekf.ys[ekf.ys.size()-2];
                        if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6)
                            theta = std::atan2(dy, dx);
                    }
                    constexpr float L = 4.5, W = 1.8, hl = L/2.0, hw = W/2.0;
                    struct Pt { float x, y; };
                    Pt corners[4] = {
                        {+hl, +hw}, {+hl, -hw}, {-hl, -hw}, {-hl, +hw}
                    };
                    ImVec2 poly[4];
                    for (int i = 0; i < 4; ++i) {
                        float wx = cx + (corners[i].x * std::cos(theta) - corners[i].y * std::sin(theta));
                        float wy = cy + (corners[i].x * std::sin(theta) + corners[i].y * std::cos(theta));
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