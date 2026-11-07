#include "degradation/DegradationEvaluator.h"

#include <algorithm>
#include <utility>

namespace lvf {

DegradationEvaluator::DegradationEvaluator(DegradationOptions options)
    : options_(options) {}

DegradationReport DegradationEvaluator::evaluate(const Frame& frame) const {
    ImageDegradationCheck imageCheck(options_.image);
    LidarDegradationCheck lidarCheck(options_.lidar);

    auto imageResult = imageCheck.evaluate(frame.image);
    auto lidarResult = lidarCheck.evaluate(frame);

    DegradationReport report;
    report.image = std::move(imageResult.status);
    report.lidar = std::move(lidarResult.status);

    const float imageWeight = std::clamp(options_.image_weight, 0.0f, 1.0f);
    report.overall = std::clamp(imageWeight * report.image.confidence +
                                    (1.0f - imageWeight) * report.lidar.confidence,
                                0.0f, 1.0f);

    if (!report.image.degraded && !report.lidar.degraded) {
        report.action = "FULL_FUSION";
    } else if (report.image.degraded && !report.lidar.degraded) {
        report.action = "GEOMETRY_ONLY";
    } else if (!report.image.degraded && report.lidar.degraded) {
        report.action = "IMAGE_ONLY";
    } else {
        report.action = "BOTH_DEGRADED";
    }

    // The occupancy map comes from LiDAR geometry; if LiDAR is degraded, the
    // map may still be produced but must not be treated as trusted.
    report.map_trusted = !report.lidar.degraded;
    return report;
}

}  // namespace lvf
