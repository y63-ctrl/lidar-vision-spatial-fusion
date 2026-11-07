#include "io/DegradationJsonWriter.h"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace lvf {
namespace {

std::string escapeJson(const std::string& text) {
    std::string escaped;
    escaped.reserve(text.size());
    for (const char c : text) {
        switch (c) {
            case '"':
                escaped += "\\\"";
                break;
            case '\\':
                escaped += "\\\\";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped += c;
                break;
        }
    }
    return escaped;
}

void writeReasons(std::ostream& out, const std::vector<std::string>& reasons) {
    out << '[';
    for (std::size_t i = 0; i < reasons.size(); ++i) {
        if (i != 0) {
            out << ", ";
        }
        out << '"' << escapeJson(reasons[i]) << '"';
    }
    out << ']';
}

void writeMetrics(std::ostream& out, const std::map<std::string, double>& metrics) {
    out << '{';
    bool first = true;
    for (const auto& [key, value] : metrics) {
        if (!first) {
            out << ", ";
        }
        first = false;
        out << '"' << escapeJson(key) << "\": " << value;
    }
    out << '}';
}

void writeModality(std::ostream& out, const ModalityStatus& status) {
    out << "{\n"
        << "    \"confidence\": " << status.confidence << ",\n"
        << "    \"degraded\": " << (status.degraded ? "true" : "false") << ",\n"
        << "    \"reasons\": ";
    writeReasons(out, status.reasons);
    out << ",\n"
        << "    \"metrics\": ";
    writeMetrics(out, status.metrics);
    out << "\n  }";
}

}  // namespace

void writeDegradationJson(const std::filesystem::path& path,
                          const DegradationReport& report,
                          const std::string& frameId) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        throw std::runtime_error("writeDegradationJson: cannot open " + path.string());
    }

    out << std::fixed << std::setprecision(4);
    out << "{\n"
        << "  \"frame\": \"" << escapeJson(frameId) << "\",\n"
        << "  \"image\": ";
    writeModality(out, report.image);
    out << ",\n"
        << "  \"lidar\": ";
    writeModality(out, report.lidar);
    out << ",\n"
        << "  \"overall\": " << report.overall << ",\n"
        << "  \"action\": \"" << escapeJson(report.action) << "\",\n"
        << "  \"map_trusted\": " << (report.map_trusted ? "true" : "false") << "\n"
        << "}\n";

    if (!out) {
        throw std::runtime_error("writeDegradationJson: failed to write " + path.string());
    }
}

}  // namespace lvf
