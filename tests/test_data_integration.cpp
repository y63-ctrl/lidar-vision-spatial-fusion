#include "data/KittiDataSource.h"

#include "TestHarness.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>

int main() {
    using namespace lvf::test;

    const std::filesystem::path sampleDir = LVF_SAMPLE_DIR;
    lvf::KittiDataSource source(sampleDir, {"000000"});

    lvf::Frame frame;
    if (!source.next(frame)) {
        std::cerr << "[FAIL] data integration: no frame returned\n";
        return 1;
    }

    check(frame.id == "000000", "frame id");
    check(frame.image.cols == 1242, "image width");
    check(frame.image.rows == 375, "image height");
    check(frame.image.channels() == 3, "image has 3 channels");
    check(frame.cloud != nullptr, "cloud pointer");
    check(frame.cloud->size() == 20000, "cloud point count");
    check(frame.calib.isValid(), "calibration valid");

    float minRange = std::numeric_limits<float>::max();
    float maxRange = 0.0f;
    float minIntensity = std::numeric_limits<float>::max();
    float maxIntensity = std::numeric_limits<float>::lowest();

    for (const auto& point : frame.cloud->points) {
        check(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z),
              "point coordinates are finite");
        const float range = std::sqrt(point.x * point.x + point.y * point.y);
        minRange = std::min(minRange, range);
        maxRange = std::max(maxRange, range);
        minIntensity = std::min(minIntensity, point.intensity);
        maxIntensity = std::max(maxIntensity, point.intensity);
    }

    check(minRange >= 0.0f, "minimum range is non-negative");
    check(maxRange > 1.0f, "maximum range is meaningful");
    check(minIntensity >= 0.0f && maxIntensity <= 1.0f, "intensity range for sample");

    const auto projection = frame.calib.veloToImage();
    check(projection.allFinite(), "T_img is finite");

    std::cout << "  image: " << frame.image.cols << 'x' << frame.image.rows << '\n'
              << "  cloud: " << frame.cloud->size() << " points\n"
              << "  range: [" << minRange << ", " << maxRange << "] m\n"
              << "  intensity: [" << minIntensity << ", " << maxIntensity << "]\n";

    return finish("data_integration");
}
