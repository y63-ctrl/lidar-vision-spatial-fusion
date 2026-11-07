#pragma once

#include "core/Types.h"

#include <cstddef>

namespace lvf {

struct LidarDegradationOptions {
    float near_range_m = 30.0f;
    std::size_t expected_near_points = 20000;
    std::size_t min_near_points = 2000;
    int azimuth_bins = 36;
    float min_azimuth_coverage = 0.4f;
    float confidence_threshold = 0.4f;

    std::size_t min_fov_points = 500;
    float min_depth = 0.5f;
    float max_depth = 80.0f;
    int projection_threads = 1;  // FOV counting is cheap; avoid OpenMP startup.
};

struct LidarDegradationMetrics {
    std::size_t total_points = 0;
    std::size_t near_points = 0;
    std::size_t fov_points = 0;
    double density_score = 0.0;
    double azimuth_coverage = 0.0;
};

struct LidarDegradationResult {
    ModalityStatus status;
    LidarDegradationMetrics metrics;
};

class LidarDegradationCheck {
public:
    explicit LidarDegradationCheck(LidarDegradationOptions options = {});

    LidarDegradationResult evaluate(const Frame& frame) const;

private:
    LidarDegradationOptions options_;
};

}  // namespace lvf
