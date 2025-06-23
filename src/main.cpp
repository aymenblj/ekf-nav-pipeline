#include "utils/oxts_parser.hpp"
#include "ekf/ekf_vehicleModel.hpp"
#include "ekf/ekf_processor.hpp"
#include "gui/gui_plotter.hpp"
#include <iostream>
#include <filesystem>
#include <memory>

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <oxts_data_dir> <oxts_log_csv> <ekf_result_csv>\n";
        return 1;
    }

    const std::filesystem::path oxts_dir = argv[1];
    const std::string oxts_log_csv = argv[2];
    const std::string ekf_result_csv = argv[3];

    try {
        utils::OXTSParser parser(oxts_dir, oxts_log_csv);

        auto vehicle_ekf = std::make_unique<ekf::VehicleEKF>();
        ekf::VehicleEKF::NoiseParams noise;
        noise.lat_var = 1e-6;
        noise.lon_var = 1e-6;
        noise.v_var = 0.1;
        noise.yaw_var = 0.001;
        noise.gnss_var = 2e-6;
        noise.vel_var = 0.04;
        noise.yaw_meas_var = 0.0025;
        vehicle_ekf->setNoiseParams(noise);

        ekf::EKFProcessor ekfProcessor(std::move(vehicle_ekf), ekf_result_csv);

        GuiPlotter plotter;
        plotter.startGui(); // Launch background GUI thread

        while (auto data = parser.next()) {
            auto state = ekfProcessor.process(*data);
            plotter.handleData(*data, state);
            plotter.realtimeSleep(data->timelapse);
        }

        std::cout << "Processed " << ekfProcessor.results().size() << " EKF states.\n";
        std::cout << "OXTS data logged to: " << oxts_log_csv << "\n";
        std::cout << "EKF results logged to: " << ekf_result_csv << "\n";

        plotter.stopGui(); // Clean up and exit GUI

    } catch (const std::exception& ex) {
        std::cerr << "Fatal error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}