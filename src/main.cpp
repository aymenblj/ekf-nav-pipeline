#include "oxts_parser.hpp"
#include "ekf_basic.hpp"
#include "ekf_processor.hpp"
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
        // Create the OXTSParser
        utils::OXTSParser parser(oxts_dir, oxts_log_csv);

        // Create EKF instance
        ekf::IEKFPtr ekf = std::make_unique<ekf::BasicEKF>();

        // Create EKFProcessor with result and log file paths
        ekf::EKFProcessor ekfProcessor(std::move(ekf), ekf_result_csv);

        // Step through each OXTSData record manually
        while (auto data = parser.next()) {
            ekfProcessor.process(*data);
        }

        std::cout << "Processed " << ekfProcessor.results().size() << " EKF states.\n";
        std::cout << "OXTS data logged to: " << oxts_log_csv << "\n";
        std::cout << "EKF results logged to: " << ekf_result_csv << "\n";

    } catch (const std::exception& ex) {
        std::cerr << "Fatal error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
