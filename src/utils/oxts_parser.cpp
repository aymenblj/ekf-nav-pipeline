#include "utils/oxts_parser.hpp"
#include "utils/timestamp_parser.hpp"
#include <string>
#include <limits>
#include <cmath>
#include <type_traits>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <optional>

namespace utils {

// Helper to get tuple of references to all members (except timestamp)
inline auto getParseTuple(OXTSData& d) {
    return std::tie(
        d.lat, d.lon, d.alt,
        d.roll, d.pitch, d.yaw,
        d.vn, d.ve, d.vf, d.vl, d.vu,
        d.ax, d.ay, d.az,
        d.af, d.al, d.au,
        d.wx, d.wy, d.wz,
        d.wf, d.wl, d.wu,
        d.pos_accuracy, d.vel_accuracy,
        d.navstat, d.numsats,
        d.posmode, d.velmode, d.orimode
    );
}

// OXTSParser constructor as you wrote it
OXTSParser::OXTSParser(const std::filesystem::path& oxts_dir, const std::string& logCSV)
    : data_dir_(oxts_dir / "data"), index_(0) {
    timestamp_stream_.open(oxts_dir / "timestamps.txt");
    if (!timestamp_stream_.is_open()) {
        throw std::runtime_error("Failed to open timestamps.txt in: " + oxts_dir.string());
    }
    if (!std::filesystem::exists(data_dir_) || !std::filesystem::is_directory(data_dir_)) {
        throw std::runtime_error("Missing or invalid oxts/data directory: " + data_dir_.string());
    }

    if (!logCSV.empty()) {
        logger_.emplace(logCSV, std::vector<std::string> {
            "timestamp", "timelapse", "lat", "lon", "alt", "roll", "pitch", "yaw", 
            "vn", "ve", "vf", "vl", "vu",
            "ax", "ay", "az", "af", "al", "au",
            "wx", "wy", "wz", "wf", "wl", "wu",
            "pos_accuracy", "vel_accuracy",
            "navstat", "numsats", "posmode", "velmode", "orimode"
        });
    }
}

// next() function using the tuple helper and std::apply
std::optional<OXTSData> OXTSParser::next() {
    std::string ts_line;
    if (!std::getline(timestamp_stream_, ts_line)) {
        return std::nullopt; // no more data
    }

    if (first_read_) {
        first_timestamp_ = ts_line;
        first_read_ = false;
    }

    std::ostringstream fname;
    fname << std::setfill('0') << std::setw(10) << index_++ << ".txt";
    std::ifstream data_file(data_dir_ / fname.str());

    if (!data_file.is_open()) {
        throw std::runtime_error("Failed to open OXTS data file: " + (data_dir_ / fname.str()).string());
    }

    std::string data_line;
    std::getline(data_file, data_line);
    std::stringstream iss(data_line);

    OXTSData data{};
    data.timestamp = ts_line;
    data.timelapse = parse_timestamp(ts_line, first_timestamp_);

    auto members = getParseTuple(data);
    std::apply([&iss](auto&... member) {
        (parseSafe(iss, member), ...);
    }, members);

    if (logger_) {
        std::apply([&](auto&&... args){
            logger_->logRow(data.timestamp, data.timelapse, std::forward<decltype(args)>(args)...);
        }, members);
    }


    return data;
}

} // namespace utils
