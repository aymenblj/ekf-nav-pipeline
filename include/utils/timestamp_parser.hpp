#pragma once

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <stdexcept>

namespace utils {

/**
 * @brief Parse a timestamp string into seconds since epoch, or as an offset from a reference timestamp.
 *
 * The timestamp format is: "YYYY-MM-DD HH:MM:SS.ssssssssss" (e.g., "2011-09-26 13:10:52.415627742").
 *
 * @param ts   Timestamp string to parse.
 * @param ts0  (Optional) Reference timestamp string. If provided, result is offset (ts - ts0) in seconds.
 * @return Seconds since epoch (if ts0 empty), or relative seconds (if ts0 provided).
 * @throws std::invalid_argument if parsing fails.
 */
inline double parse_timestamp(std::string_view ts, std::string_view ts0 = "") {
    // Helper lambda to parse a single timestamp
    auto parse = [](std::string_view tsv) -> double {
        std::tm t = {};
        double subseconds = 0.0;
        std::string ts_str(tsv);
        std::istringstream ss(ts_str);
        ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S");
        if (ss.fail()) {
            throw std::invalid_argument("Failed to parse timestamp: " + ts_str);
        }
        if (ss.peek() == '.') {
            char dot;
            std::string nano;
            ss >> dot >> nano;
            if (!nano.empty())
                subseconds = std::stod("0." + nano);
        }
        auto tp = std::chrono::system_clock::from_time_t(std::mktime(&t));
        return std::chrono::duration<double>(tp.time_since_epoch()).count() + subseconds;
    };

    double seconds = parse(ts);
    if (!ts0.empty()) {
        double seconds0 = parse(ts0);
        return seconds - seconds0;
    }
    return seconds;
}

} // namespace utils