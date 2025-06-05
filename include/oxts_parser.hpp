#pragma once

#include <string>
#include <fstream>
#include <filesystem>
#include <optional>
#include <cmath>
#include "oxts_types.hpp"
#include "csv_logger.hpp"

namespace utils {

class OXTSParser {
public:
    explicit OXTSParser(const std::filesystem::path& oxts_dir, const std::string& logCSV = "");
    std::optional<OXTSData> next();

private:
    std::ifstream timestamp_stream_;
    std::filesystem::path data_dir_;
    size_t index_;
    std::string resultCSVPath_;
    std::optional<Logger> logger_; // Optional logger for parsed values
};

} // namespace utils
