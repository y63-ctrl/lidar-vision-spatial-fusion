#include "data/KittiDataSource.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

struct Options {
    std::filesystem::path dataRoot{"data/sample"};
    std::string frameId{"000000"};
    std::filesystem::path calibOverride;
};

void printUsage() {
    std::cout << "Usage: lvf_inspect_frame [options]\n"
              << "  --data-root <dir>   KITTI-style data root (default: data/sample)\n"
              << "  --frame <id>        frame id, e.g. 000000 or 0 (default: 000000)\n"
              << "  --calib <file>      optional calibration override\n"
              << "  --help              show this message\n";
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto nextValue = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                throw std::invalid_argument(std::string("missing value for ") + name);
            }
            return argv[++i];
        };

        if (arg == "--data-root") {
            options.dataRoot = nextValue("--data-root");
        } else if (arg == "--frame") {
            options.frameId = nextValue("--frame");
        } else if (arg == "--calib") {
            options.calibOverride = nextValue("--calib");
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }
    return options;
}

void printMatrix(const Eigen::Matrix<float, 3, 4>& matrix) {
    std::cout << std::fixed << std::setprecision(4);
    for (int row = 0; row < matrix.rows(); ++row) {
        std::cout << "  [";
        for (int col = 0; col < matrix.cols(); ++col) {
            if (col != 0) {
                std::cout << ", ";
            }
            std::cout << std::setw(11) << matrix(row, col);
        }
        std::cout << " ]\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);

        lvf::KittiDataSource source(options.dataRoot, {options.frameId}, options.calibOverride);
        lvf::Frame frame;
        if (!source.next(frame)) {
            throw std::runtime_error("no frame returned by KittiDataSource");
        }

        float minRange = std::numeric_limits<float>::max();
        float maxRange = 0.0f;
        float minIntensity = std::numeric_limits<float>::max();
        float maxIntensity = std::numeric_limits<float>::lowest();

        for (const auto& point : frame.cloud->points) {
            const float range = std::sqrt(point.x * point.x + point.y * point.y);
            minRange = std::min(minRange, range);
            maxRange = std::max(maxRange, range);
            minIntensity = std::min(minIntensity, point.intensity);
            maxIntensity = std::max(maxIntensity, point.intensity);
        }

        std::cout << "source: " << source.name() << '\n';
        std::cout << "frame: " << frame.id << '\n';
        std::cout << "image: " << frame.image.cols << "x" << frame.image.rows
                  << " channels=" << frame.image.channels() << '\n';
        std::cout << "cloud: " << frame.cloud->size() << " points\n";
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "range: [" << minRange << ", " << maxRange << "] m\n";
        std::cout << "intensity: [" << minIntensity << ", " << maxIntensity << "]\n";
        std::cout << "calibration.valid: " << (frame.calib.isValid() ? "true" : "false") << '\n';
        std::cout << "T_img = P2 * R0_rect * Tr_velo_to_cam:\n";
        printMatrix(frame.calib.veloToImage());

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
