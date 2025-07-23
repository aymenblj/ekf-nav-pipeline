#pragma once

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>
#if __cplusplus >= 202002L
#include <concepts>
#endif

/**
 * @file logger.hpp
 * @brief CSV Logger utility for writing tabular data to a file.
 */

namespace io {

#if __cplusplus >= 202002L
// Concept: Type can be streamed to an ostream (e.g., std::cout << x)
template<typename T>
concept OStreamable = requires(std::ostream& os, T&& t) {
    { os << std::forward<T>(t) } -> std::same_as<std::ostream&>;
};
#endif

/**
 * @class Logger
 * @brief Utility class for writing rows of data to a CSV file with a header.
 *
 * Example usage:
 * @code
 *   io::Logger logger("output.csv", {"time", "lat", "lon"});
 *   logger.logRow(123.45, 49.01, 8.43);
 * @endcode
 *
 * The Logger writes the specified header as the first line, and each call to logRow()
 * writes a new row, with all values comma-separated and fixed precision.
 */
class Logger {
public:
    /**
     * @brief Construct a new Logger object and write the CSV header.
     * @param filepath  Output file path for the CSV.
     * @param header    Vector of column names for the header row.
     * @param precision Number of decimal places for floating-point output (default: 8).
     * @throws std::runtime_error if the file cannot be opened.
     */
    Logger(const std::string& filepath, std::vector<std::string> header, int precision = 8)
        : out_(filepath), header_(std::move(header)) {
        if (!out_.is_open()) {
            throw std::runtime_error("Failed to open file: " + filepath);
        }
        out_ << std::fixed << std::setprecision(precision);
        writeHeader();
    }

    /**
     * @brief Write a new row to the CSV file.
     *
     * The number and order of arguments must match the columns in the header.
     *
     * @tparam Args Types of values to log.
     * @param args  Values to log for each column.
     */
#if __cplusplus >= 202002L
    template<OStreamable... Args>
    void logRow(Args&&... args) {
#else
    template<typename... Args>
    void logRow(Args&&... args) {
#endif
        if constexpr (sizeof...(args) != 0) {
            if (sizeof...(args) != header_.size()) {
                throw std::runtime_error("Logger row/header size mismatch");
            }
        }
        writeRow(std::forward<Args>(args)...);
    }

private:
    std::ofstream out_;
    std::vector<std::string> header_;

    /// Write the column header to the CSV file.
    void writeHeader() {
        for (size_t i = 0; i < header_.size(); ++i) {
            out_ << header_[i];
            if (i < header_.size() - 1) out_ << ",";
        }
        out_ << "\n";
    }

    /// Recursive variadic helper to write a row to the CSV file.
    template<typename T, typename... Args>
    void writeRow(T&& val, Args&&... rest) {
        out_ << std::forward<T>(val);
        if constexpr (sizeof...(rest) > 0) {
            out_ << ",";
            writeRow(std::forward<Args>(rest)...);
        } else {
            out_ << "\n";
        }
    }
};

} // namespace io