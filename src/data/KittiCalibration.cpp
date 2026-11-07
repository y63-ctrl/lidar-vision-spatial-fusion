#include "data/KittiCalibration.h"

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace lvf {
namespace {

constexpr const char* kP2Key = "P2";
constexpr const char* kRectKey = "R0_rect";
constexpr const char* kTransformKey = "Tr_velo_to_cam";

std::string trim(std::string text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

[[noreturn]] void parseError(const std::string& source, const std::string& detail) {
    throw std::runtime_error("KittiCalibration(" + source + "): " + detail);
}

void appendNumbers(const std::string& text,
                   const std::string& source,
                   std::vector<double>& values) {
    std::istringstream stream(text);
    double value = 0.0;
    while (stream >> value) {
        if (!std::isfinite(value)) {
            parseError(source, "non-finite matrix value");
        }
        values.push_back(value);
    }
}

std::vector<double> requireKey(const std::map<std::string, std::vector<double>>& values,
                               const std::string& key,
                               std::size_t expected,
                               const std::string& source) {
    const auto it = values.find(key);
    if (it == values.end()) {
        parseError(source, "missing required key '" + key + "'");
    }
    if (it->second.size() != expected) {
        std::ostringstream message;
        message << "key '" << key << "' has " << it->second.size() << " values, expected "
                << expected;
        parseError(source, message.str());
    }
    return it->second;
}

template <typename Matrix>
Matrix fillMatrix(const std::vector<double>& values, int rows, int cols) {
    Matrix matrix;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            matrix(row, col) = static_cast<float>(values[static_cast<std::size_t>(row * cols + col)]);
        }
    }
    return matrix;
}

}  // namespace

Calibration KittiCalibration::parse(std::istream& input, const std::string& sourceName) {
    std::map<std::string, std::vector<double>> values;
    std::string currentKey;
    std::string line;

    while (std::getline(input, line)) {
        const std::string cleaned = trim(line);
        if (cleaned.empty()) {
            continue;
        }

        const auto colon = cleaned.find(':');
        if (colon != std::string::npos) {
            const std::string key = trim(cleaned.substr(0, colon));
            if (key == kP2Key || key == kRectKey || key == kTransformKey) {
                currentKey = key;
                values[currentKey].clear();
                appendNumbers(cleaned.substr(colon + 1), sourceName, values[currentKey]);
            } else {
                // A new, unrelated key ends any multi-line continuation.
                currentKey.clear();
            }
            continue;
        }

        if (!currentKey.empty()) {
            appendNumbers(cleaned, sourceName, values[currentKey]);
        }
    }

    if (!input.eof() && input.fail()) {
        parseError(sourceName, "failed while reading stream");
    }

    const auto p2Values = requireKey(values, kP2Key, 12, sourceName);
    const auto rectValues = requireKey(values, kRectKey, 9, sourceName);
    const auto transformValues = requireKey(values, kTransformKey, 12, sourceName);

    Calibration calibration;
    calibration.P2 = fillMatrix<Eigen::Matrix<float, 3, 4>>(p2Values, 3, 4);

    const auto rect3 = fillMatrix<Eigen::Matrix3f>(rectValues, 3, 3);
    calibration.R0_rect = Eigen::Matrix4f::Identity();
    calibration.R0_rect.topLeftCorner<3, 3>() = rect3;

    const auto transform3x4 = fillMatrix<Eigen::Matrix<float, 3, 4>>(transformValues, 3, 4);
    calibration.T_velo_cam = Eigen::Matrix4f::Identity();
    calibration.T_velo_cam.topLeftCorner<3, 4>() = transform3x4;

    if (!calibration.P2.allFinite() || !calibration.R0_rect.allFinite() ||
        !calibration.T_velo_cam.allFinite()) {
        parseError(sourceName, "calibration contains non-finite values");
    }

    // KITTI P2's last row is [0, 0, 1, small]; reject obviously wrong matrices.
    if (std::abs(calibration.P2(2, 0)) > 1e-2f ||
        std::abs(calibration.P2(2, 1)) > 1e-2f ||
        std::abs(calibration.P2(2, 2) - 1.0f) > 1e-2f) {
        parseError(sourceName, "P2 last row is not approximately [0, 0, 1, *]");
    }

    if (!calibration.isValid()) {
        parseError(sourceName, "calibration failed validation");
    }

    return calibration;
}

Calibration KittiCalibration::loadFromFile(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("KittiCalibration: file not found: " + path.string());
    }
    if (!std::filesystem::is_regular_file(path)) {
        throw std::runtime_error("KittiCalibration: not a regular file: " + path.string());
    }

    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("KittiCalibration: cannot open: " + path.string());
    }

    return parse(input, path.string());
}

}  // namespace lvf
