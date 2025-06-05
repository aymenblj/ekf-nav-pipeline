#include "csv_logger.hpp"

namespace utils {

Logger::Logger(const std::string& filepath, std::vector<std::string> header, int precision)
    : out_(filepath), header_(std::move(header)) {
    if (!out_.is_open()) {
        throw std::runtime_error("Failed to open file: " + filepath);
    }
    out_ << std::fixed << std::setprecision(precision);
    writeHeader();
}

void Logger::writeHeader() {
    for (size_t i = 0; i < header_.size(); ++i) {
        out_ << header_[i];
        if (i < header_.size() - 1) {
            out_ << ",";
        }
    }
    out_ << "\n";
}

} // namespace utils
