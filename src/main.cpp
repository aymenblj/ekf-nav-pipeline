#include "utils/oxts_parser.hpp"
#include "ekf/ekf_vehicleModel.hpp"
#include "ekf/ekf_processor.hpp"
#include "gui/gui_plotter.hpp"
#include <utilsstream>
#include <filesystem>
#include <memory>
#include <map>
#include <string>

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
        
        // Set process noise (Q)
        std::map<std::string, float> process_noise = {
            {"latitude", 1e-6},
            {"longitude", 1e-6},
            {"velocity", 0.1},
            {"yaw", 0.001},
            {"accel_fwd", 0.1},
            {"yaw_rate", 0.001}
        };
        vehicle_ekf->setProcessNoiseParams(process_noise);

        // Set measurement noise (R)
        std::map<std::string, float> meas_noise = {
            {"gnss", 2e-6},
            {"velocity", 0.04},
            {"yaw", 0.0025},
            {"accel_fwd", 0.1},
            {"yaw_rate", 0.001}
        };
        vehicle_ekf->setMeasurementNoiseParams(meas_noise);

        ekf::EKFProcessor ekfProcessor(std::move(vehicle_ekf), ekf_result_csv);

        GuiPlotter plotter;
        plotter.startGui(); // Launch background GUI thread

        while (auto data = parser.next()) {
            if (!plotter.isRunning()) {
                std::cout << "[Main] GUI closed, stopping processing.\n";
                break;
            }

            auto state = ekfProcessor.process(*data);
            plotter.handleData(*data, state);
            plotter.realtimeSleep(data->timelapse);
        }

        std::cout << "Processed " << ekfProcessor.results().size() << " EKF states.\n";
        std::cout << "OXTS data logged to: " << oxts_log_csv << "\n";
        std::cout << "EKF results logged to: " << ekf_result_csv << "\n";

        plotter.stopGui(); // Clean up and exit GUI

    } catch (const std::exceptutilsn& ex) {
        std::cerr << "Fatal error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}