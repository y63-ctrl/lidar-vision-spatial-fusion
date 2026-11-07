#include "fusion/Projection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace lvf {
namespace {

void validateOptions(const ProjectionOptions& options) {
    if (options.image_width <= 0 || options.image_height <= 0) {
        throw std::invalid_argument("Projection: image_width/image_height must be positive");
    }
    if (options.min_depth < 0.0f || options.max_depth <= options.min_depth) {
        throw std::invalid_argument("Projection: invalid depth range");
    }
}

ProjectedPoint projectWithMatrix(const Eigen::Matrix<float, 3, 4>& matrix,
                                 const Eigen::Vector3f& point,
                                 const ProjectionOptions& options) {
    const Eigen::Vector4f homogeneous(point.x(), point.y(), point.z(), 1.0f);
    const Eigen::Vector3f uvw = matrix * homogeneous;

    ProjectedPoint projected;
    projected.depth = uvw.z();
    if (projected.depth <= options.min_depth || projected.depth > options.max_depth) {
        return projected;
    }

    projected.u = uvw.x() / projected.depth;
    projected.v = uvw.y() / projected.depth;
    if (projected.u < 0.0f || projected.v < 0.0f ||
        projected.u >= static_cast<float>(options.image_width) ||
        projected.v >= static_cast<float>(options.image_height)) {
        return projected;
    }

    projected.valid = true;
    return projected;
}

}  // namespace

ProjectedPoint Projection::projectPoint(const Calibration& calibration,
                                        const Eigen::Vector3f& veloPoint,
                                        const ProjectionOptions& options) {
    validateOptions(options);
    return projectWithMatrix(calibration.veloToImage(), veloPoint, options);
}

std::vector<ProjectedPoint> Projection::projectBatch(
    const Calibration& calibration,
    const std::vector<Eigen::Vector3f>& veloPoints,
    const ProjectionOptions& options,
    int threads) {
    validateOptions(options);
    if (veloPoints.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("Projection: point count exceeds int range");
    }

    const auto matrix = calibration.veloToImage();
    const int count = static_cast<int>(veloPoints.size());
    std::vector<ProjectedPoint> projected(veloPoints.size());

    if (threads == 1) {
        for (int i = 0; i < count; ++i) {
            projected[static_cast<std::size_t>(i)] =
                projectWithMatrix(matrix, veloPoints[static_cast<std::size_t>(i)], options);
        }
        return projected;
    }

#ifdef _OPENMP
    const int threadCount = threads > 0 ? threads : omp_get_max_threads();
#pragma omp parallel for schedule(static) num_threads(threadCount)
#endif
    for (int i = 0; i < count; ++i) {
        projected[static_cast<std::size_t>(i)] =
            projectWithMatrix(matrix, veloPoints[static_cast<std::size_t>(i)], options);
    }

    return projected;
}

}  // namespace lvf
