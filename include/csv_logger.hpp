#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <iomanip>
#include <utility>

namespace utils {

class Logger {
public:
    Logger(const std::string& filepath, std::vector<std::string> header, int precision = 12);

    template <typename... Ts>
    void logRow(Ts&&... values);

private:
    std::ofstream out_;
    std::vector<std::string> header_;

    void writeHeader();

    template <typename... Ts>
    void writeRow(Ts&&... values);
};

// Inline template implementations

template <typename... Ts>
void Logger::logRow(Ts&&... values) {
    static_assert(sizeof...(Ts) > 0, "No data to log");
    if (sizeof...(Ts) != header_.size()) {
        throw std::runtime_error("Row size does not match header size");
    }
    writeRow(std::forward<Ts>(values)...);
}

template <typename... Ts>
void Logger::writeRow(Ts&&... values) {
    size_t count = 0;
    ((out_ << std::forward<Ts>(values) << (++count < sizeof...(Ts) ? "," : "")), ...);
    out_ << "\n";
}

} // namespace utils
