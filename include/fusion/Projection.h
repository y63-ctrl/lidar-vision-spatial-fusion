#pragma once

#include "core/Types.h"

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace lvf {

struct ProjectionOptions {
    int image_width = 0;
    int image_height = 0;
    float min_depth = 0.5f;
    float max_depth = 80.0f;
};

struct ProjectedPoint {
    bool valid = false;
    float u = 0.0f;
    float v = 0.0f;
    float depth = 0.0f;
};

/// Pure Eigen projection core: no PCL, no OpenCV image access.
///
/// `projectPoint` is the scalar reference implementation. `projectBatch` uses
/// OpenMP when available; the output at each index is independent of the
/// thread count.
class Projection {
public:
    static ProjectedPoint projectPoint(const Calibration& calibration,
                                       const Eigen::Vector3f& veloPoint,
                                       const ProjectionOptions& options);

    static std::vector<ProjectedPoint> projectBatch(
        const Calibration& calibration,
        const std::vector<Eigen::Vector3f>& veloPoints,
        const ProjectionOptions& options,
        int threads = 0);
};

}  // namespace lvf
