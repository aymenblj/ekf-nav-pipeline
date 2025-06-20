#include "gui/gui_plotter.hpp"
#include <imgui.h>
#include <implot.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <algorithm>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <iostream>
#include <exception>
#include <cmath>
#include <iomanip>

GuiPlotter::GuiPlotter() : running_(false), zoom_out_full_(false) {
    std::cout << "[GuiPlotter] Constructed.\n";
}

GuiPlotter::~GuiPlotter() {
    running_ = false;
    std::cout << "[GuiPlotter] Destructor called.\n";
}

void GuiPlotter::add_ekf_estimate(double timestamp, const std::vector<double>& values) {
    if (std::isnan(timestamp) || std::isinf(timestamp)) {
        std::cout << "[GuiPlotter] Warning: Invalid EKF timestamp: " << timestamp << "\n";
        return;
    }
    if (values.size() >= 2 && std::isfinite(values[0]) && std::isfinite(values[1])) {
        std::lock_guard<std::mutex> lock(mtx_);
        ts_ekf_.push_back(timestamp);
        xs_ekf_.push_back(values[0]);
        ys_ekf_.push_back(values[1]);
        std::cout << std::fixed << std::setprecision(9);
        std::cout << "[GuiPlotter] EKF point: t=" << timestamp
                  << " x=" << values[0] << " y=" << values[1] << std::endl;
    } else {
        std::cout << "[GuiPlotter] Warning: Invalid or insufficient EKF values (size=" << values.size() << ")\n";
    }
}

void GuiPlotter::run(std::function<void()> stream_func) {
    std::cout << "[GuiPlotter] Starting run().\n";
    if (!glfwInit()) {
        std::cout << "[GuiPlotter] Failed to initialize GLFW.\n";
        return;
    }
    std::cout << "[GuiPlotter] GLFW initialized.\n";

    GLFWwindow* window = glfwCreateWindow(1600, 1000, "NavPlotterPro", nullptr, nullptr);
    if (!window) {
        std::cout << "[GuiPlotter] Failed to create GLFW window.\n";
        glfwTerminate();
        return;
    }
    std::cout << "[GuiPlotter] GLFW window created.\n";
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    try {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImPlot::CreateContext();
        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init("#version 130");
        std::cout << "[GuiPlotter] ImGui and ImPlot initialized.\n";
    } catch (const std::exception& e) {
        std::cout << "[GuiPlotter] Exception during ImGui/ImPlot init: " << e.what() << "\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return;
    } catch (...) {
        std::cout << "[GuiPlotter] Unknown error during ImGui/ImPlot init.\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return;
    }

    running_ = true;

    std::thread streaming_thread([this, stream_func]() {
        std::cout << "[GuiPlotter] Streaming thread started.\n";
        try {
            if (stream_func) stream_func();
            std::cout << "[GuiPlotter] Streaming finished.\n";
        } catch (const std::exception& e) {
            std::cout << "[GuiPlotter] Exception in streaming thread: " << e.what() << "\n";
        } catch (...) {
            std::cout << "[GuiPlotter] Unknown error in streaming thread.\n";
        }
        zoom_out_full_ = true;
        running_ = false;
    });

    std::cout << "[GuiPlotter] Entering main GUI loop.\n";
    try {
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();

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

            std::vector<double> xs_ekf, ys_ekf;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                xs_ekf = xs_ekf_;
                ys_ekf = ys_ekf_;
            }

            ImGui::Text("EKF Points: %zu", xs_ekf.size());
            if (!xs_ekf.empty() && !ys_ekf.empty()) {
                ImVec2 plot_size = ImGui::GetContentRegionAvail();

                if (ImPlot::BeginPlot("##Trajectory", plot_size)) {
                    ImPlot::SetupAxes("X [m]", "Y [m]");

                    if (zoom_out_full_) {
                        double x_min = *std::min_element(xs_ekf.begin(), xs_ekf.end());
                        double x_max = *std::max_element(xs_ekf.begin(), xs_ekf.end());
                        double y_min = *std::min_element(ys_ekf.begin(), ys_ekf.end());
                        double y_max = *std::max_element(ys_ekf.begin(), ys_ekf.end());
                        double x_pad = (x_max - x_min) * 0.03;
                        double y_pad = (y_max - y_min) * 0.03;
                        ImPlot::SetupAxisLimits(ImAxis_X1, x_min - x_pad, x_max + x_pad, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, y_min - y_pad, y_max + y_pad, ImGuiCond_Always);
                    } else {
                        double cx = xs_ekf.back();
                        double cy = ys_ekf.back();
                        double x_min = *std::min_element(xs_ekf.begin(), xs_ekf.end());
                        double x_max = *std::max_element(xs_ekf.begin(), xs_ekf.end());
                        double y_min = *std::min_element(ys_ekf.begin(), ys_ekf.end());
                        double y_max = *std::max_element(ys_ekf.begin(), ys_ekf.end());
                        double xrange = x_max - x_min + 1e-8;
                        double yrange = y_max - y_min + 1e-8;
                        double min_range = 0.001;
                        double xw = std::max(xrange * 1.15, min_range);
                        double yw = std::max(yrange * 1.15, min_range);
                        ImPlot::SetupAxisLimits(ImAxis_X1, cx - xw / 2, cx + xw / 2, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, cy - yw / 2, cy + yw / 2, ImGuiCond_Always);
                    }

                    ImPlot::SetNextLineStyle(ImVec4(1.f, 0.2f, 0.2f, 1.f), 2.0f);
                    ImPlot::PlotLine("EKF", xs_ekf.data(), ys_ekf.data(), (int)xs_ekf.size());

                    // Car marker for EKF
                    double cx = xs_ekf.back();
                    double cy = ys_ekf.back();
                    ImPlot::PlotScatter("Car", &cx, &cy, 1);

                    double vx = 0, vy = 0;
                    if (xs_ekf.size() >= 2) {
                        vx = xs_ekf.back() - xs_ekf[xs_ekf.size() - 2];
                        vy = ys_ekf.back() - ys_ekf[ys_ekf.size() - 2];
                    }
                    double norm = std::sqrt(vx * vx + vy * vy);
                    if (norm < 1e-10) { vx = 1; vy = 0; norm = 1; }
                    vx /= norm; vy /= norm;

                    ImDrawList* draw_list = ImPlot::GetPlotDrawList();

                    double arrow_size = 2.0;
                    double tip_x = cx + vx * arrow_size;
                    double tip_y = cy + vy * arrow_size;
                    double left_x = cx - vx * arrow_size * 0.5 + vy * arrow_size * 0.5;
                    double left_y = cy - vy * arrow_size * 0.5 - vx * arrow_size * 0.5;
                    double right_x = cx - vx * arrow_size * 0.5 - vy * arrow_size * 0.5;
                    double right_y = cy - vy * arrow_size * 0.5 + vx * arrow_size * 0.5;

                    ImVec2 tip = ImPlot::PlotToPixels(tip_x, tip_y);
                    ImVec2 left = ImPlot::PlotToPixels(left_x, left_y);
                    ImVec2 right = ImPlot::PlotToPixels(right_x, right_y);

                    draw_list->AddTriangleFilled(tip, left, right, IM_COL32(255, 200, 0, 255));
                    draw_list->AddTriangle(tip, left, right, IM_COL32(120, 70, 0, 255));

                    ImPlot::EndPlot();
                }
            } else {
                ImGui::Text("No EKF data received yet.");
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
        std::cout << "[GuiPlotter] Exiting main GUI loop.\n";
    } catch (const std::exception& e) {
        std::cout << "[GuiPlotter] Exception in GUI loop: " << e.what() << "\n";
    } catch (...) {
        std::cout << "[GuiPlotter] Unknown error in GUI loop.\n";
    }

    running_ = false;
    if (streaming_thread.joinable()) {
        std::cout << "[GuiPlotter] Waiting for streaming thread to join...\n";
        streaming_thread.join();
        std::cout << "[GuiPlotter] Streaming thread joined.\n";
    }

    std::cout << "[GuiPlotter] Shutting down ImGui/ImPlot/GLFW.\n";
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    std::cout << "[GuiPlotter] Clean shutdown complete.\n";
}