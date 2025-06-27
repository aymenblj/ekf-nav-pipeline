#pragma once

#include <string>
#include <fstream>
#include <filesystem>
#include <optional>
#include <cmath>
#include "utils/oxts_types.hpp"
#include "utils/csv_logger.hpp"

namespace utils {

// parseSafe declarations and specializations

template<typename T>
void parseSafe(std::stringstream& stream, T& value);

// Specialization for double
template<>
inline void parseSafe<double>(std::stringstream& stream, double& value) {
    std::string token;
    stream >> token;
    if (token == "NAN" || token == "nan") {
        value = NAN;
    } else {
        try {
            value = std::stold(token);
        } catch (...) {
            value = NAN;
        }
    }
}

// Specialization for int
template<>
inline void parseSafe<int>(std::stringstream& stream, int& value) {
    std::string token;
    stream >> token;
    if (token == "NAN" || token == "nan") {
        value = std::numeric_limits<int>::min(); // sentinel for invalid int
    } else {
        try {
            value = std::stoi(token);
        } catch (...) {
            value = std::numeric_limits<int>::min();
        }
    }
}

/**
 * @brief Parser for OXTS data directories (e.g., KITTI dataset).
 *
 * This class reads timestamp and data files from an OXTS directory (such as those used in the KITTI dataset),
 * parses each timestep, and optionally logs parsed values to a CSV file.
 *
 * Usage:
 * @code
 *   utils::OXTSParser parser("/path/to/oxts", "parsed_log.csv");
 *   while (auto data = parser.next()) {
 *       // Use data.value()
 *   }
 * @endcode
 */
class OXTSParser {
public:
    /**
     * @brief Construct a new OXTSParser.
     * @param oxts_dir Path to the OXTS data directory.
     * @param logCSV   Path to an optional CSV file for logging parsed values (empty string disables logging).
     */
    explicit OXTSParser(const std::filesystem::path& oxts_dir, const std::string& logCSV = "");

    /**
     * @brief Get the next OXTS data record.
     * @return std::optional<OXTSData> containing the next data row, or std::nullopt if no more data.
     */
    std::optional<OXTSData> next();

private:
    std::ifstream timestamp_stream_;     ///< Stream for timestamps.txt file.
    std::filesystem::path data_dir_;     ///< Path to directory containing OXTS data files.
    size_t index_ = 0;                   ///< Current file index.
    std::string resultCSVPath_;          ///< CSV log file path (if logging enabled).
    std::optional<Logger> logger_;       ///< Optional logger for parsed values.
    std::string first_timestamp_;        ///< Stores the first timestamp for timelapse calculation.
    bool first_read_ = true;             ///< True until first timestamp is read.
};
} // namespace utils