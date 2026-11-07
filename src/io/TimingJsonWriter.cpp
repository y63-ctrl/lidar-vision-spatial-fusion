#include "io/TimingJsonWriter.h"

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
        if (c == '"') {
            escaped += "\\\"";
        } else if (c == '\\') {
            escaped += "\\\\";
        } else {
            escaped += c;
        }
    }
    return escaped;
}

}  // namespace

void writeTimingJson(const std::filesystem::path& path,
                     const StageTimings& timings,
                     const std::string& frameId) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        throw std::runtime_error("writeTimingJson: cannot open " + path.string());
    }

    out << std::fixed << std::setprecision(4)
        << "{\n"
        << "  \"frame\": \"" << escapeJson(frameId) << "\",\n"
        << "  \"read_ms\": " << timings.read_ms << ",\n"
        << "  \"degradation_ms\": " << timings.degradation_ms << ",\n"
        << "  \"fusion_ms\": " << timings.fusion_ms << ",\n"
        << "  \"mapping_ms\": " << timings.mapping_ms << ",\n"
        << "  \"write_ms\": " << timings.write_ms << ",\n"
        << "  \"compute_ms\": " << timings.compute_ms << ",\n"
        << "  \"total_ms\": " << timings.total_ms << "\n"
        << "}\n";

    if (!out) {
        throw std::runtime_error("writeTimingJson: failed to write " + path.string());
    }
}

}  // namespace lvf
