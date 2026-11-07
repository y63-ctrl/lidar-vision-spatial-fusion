#include "degradation/LidarDegradationCheck.h"

#include "fusion/Projection.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace lvf {
namespace {

double clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

}  // namespace

LidarDegradationCheck::LidarDegradationCheck(LidarDegradationOptions options)
    : options_(options) {}

LidarDegradationResult LidarDegradationCheck::evaluate(const Frame& frame) const {
    LidarDegradationResult result;
    result.status.confidence = 0.0f;
    result.status.degraded = true;

    if (frame.cloud == nullptr || frame.cloud->empty()) {
        result.status.reasons.push_back("EMPTY_CLOUD");
        result.status.metrics = {
            {"total_points", 0.0},
            {"near_points", 0.0},
            {"fov_points", 0.0},
            {"density_score", 0.0},
            {"azimuth_coverage", 0.0},
        };
        return result;
    }

    const int bins = std::max(1, options_.azimuth_bins);
    const double twoPi = 2.0 * std::acos(-1.0);
    std::vector<int> binCounts(static_cast<std::size_t>(bins), 0);
    std::size_t nearPoints = 0;

    for (const auto& point : frame.cloud->points) {
        const float range = std::sqrt(point.x * point.x + point.y * point.y);
        if (range > options_.near_range_m) {
            continue;
        }
        ++nearPoints;

        double angle = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
        if (angle < 0.0) {
            angle += twoPi;
        }
        const int bin = std::min(bins - 1, static_cast<int>(angle / twoPi * bins));
        ++binCounts[static_cast<std::size_t>(bin)];
    }

    const int nonEmptyBins =
        static_cast<int>(std::count_if(binCounts.begin(), binCounts.end(),
                                      [](int count) { return count > 0; }));
    const double coverage = static_cast<double>(nonEmptyBins) / static_cast<double>(bins);
    const double density =
        options_.expected_near_points == 0
            ? 0.0
            : clamp01(static_cast<double>(nearPoints) /
                      static_cast<double>(options_.expected_near_points));

    std::size_t fovPoints = 0;
    if (!frame.image.empty()) {
        std::vector<Eigen::Vector3f> points;
        points.reserve(frame.cloud->size());
        for (const auto& point : frame.cloud->points) {
            points.emplace_back(point.x, point.y, point.z);
        }
        const ProjectionOptions projectionOptions{
            frame.image.cols, frame.image.rows, options_.min_depth, options_.max_depth};
        const auto projected = Projection::projectBatch(
            frame.calib, points, projectionOptions, options_.projection_threads);
        fovPoints = static_cast<std::size_t>(
            std::count_if(projected.begin(), projected.end(),
                         [](const ProjectedPoint& point) { return point.valid; }));
    }

    LidarDegradationMetrics metrics;
    metrics.total_points = frame.cloud->size();
    metrics.near_points = nearPoints;
    metrics.fov_points = fovPoints;
    metrics.density_score = density;
    metrics.azimuth_coverage = coverage;

    result.metrics = metrics;
    result.status.confidence = static_cast<float>(
        clamp01(0.6 * density + 0.4 * coverage));

    if (density < 0.4 || nearPoints < options_.min_near_points) {
        result.status.reasons.push_back("SPARSE_POINTS");
    }
    if (coverage < static_cast<double>(options_.min_azimuth_coverage)) {
        result.status.reasons.push_back("LOW_COVERAGE");
    }
    if (!frame.image.empty() && fovPoints < options_.min_fov_points) {
        result.status.reasons.push_back("FEW_FOV_POINTS");
    }

    result.status.degraded =
        result.status.confidence < options_.confidence_threshold ||
        !result.status.reasons.empty();

    result.status.metrics = {
        {"total_points", static_cast<double>(metrics.total_points)},
        {"near_points", static_cast<double>(metrics.near_points)},
        {"fov_points", static_cast<double>(metrics.fov_points)},
        {"density_score", metrics.density_score},
        {"azimuth_coverage", metrics.azimuth_coverage},
    };

    return result;
}

}  // namespace lvf
