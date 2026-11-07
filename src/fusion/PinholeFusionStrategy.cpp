#include "fusion/PinholeFusionStrategy.h"

#include "fusion/Projection.h"

#include "ColorUtils.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace lvf {

PinholeFusionStrategy::PinholeFusionStrategy(FusionOptions options) : options_(options) {}

FusionResult PinholeFusionStrategy::fuse(const Frame& frame) const {
    if (frame.cloud == nullptr || frame.cloud->empty()) {
        throw std::invalid_argument("PinholeFusionStrategy: empty point cloud");
    }
    if (frame.image.empty() || frame.image.type() != CV_8UC3) {
        throw std::invalid_argument("PinholeFusionStrategy: expected a non-empty BGR8 image");
    }
    if (frame.cloud->size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("PinholeFusionStrategy: point cloud exceeds int range");
    }

    const int count = static_cast<int>(frame.cloud->size());
    std::vector<Eigen::Vector3f> points(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const auto& point = frame.cloud->points[static_cast<std::size_t>(i)];
        points[static_cast<std::size_t>(i)] = Eigen::Vector3f(point.x, point.y, point.z);
    }

    const ProjectionOptions projectionOptions{
        frame.image.cols, frame.image.rows, options_.min_depth, options_.max_depth};
    const auto projected =
        Projection::projectBatch(frame.calib, points, projectionOptions, options_.threads);

    std::size_t validCount = 0;
    for (const auto& point : projected) {
        if (point.valid) {
            ++validCount;
        }
    }

    std::vector<cv::Vec3b> colors(static_cast<std::size_t>(count));
#ifdef _OPENMP
    const int threadCount = options_.threads > 0 ? options_.threads : omp_get_max_threads();
#pragma omp parallel for schedule(static) num_threads(threadCount)
#endif
    for (int i = 0; i < count; ++i) {
        const auto& projectedPoint = projected[static_cast<std::size_t>(i)];
        if (!projectedPoint.valid) {
            continue;
        }
        colors[static_cast<std::size_t>(i)] =
            options_.bilinear
                ? detail::sampleBilinear(frame.image, projectedPoint.u, projectedPoint.v)
                : detail::sampleNearest(frame.image, projectedPoint.u, projectedPoint.v);
    }

    auto cloud = std::make_shared<CloudXYZRGB>();
    cloud->points.resize(validCount);
    std::size_t outputIndex = 0;
    for (int i = 0; i < count; ++i) {
        const auto& projectedPoint = projected[static_cast<std::size_t>(i)];
        if (!projectedPoint.valid) {
            continue;
        }

        const auto& source = frame.cloud->points[static_cast<std::size_t>(i)];
        auto& target = cloud->points[outputIndex++];
        target.x = source.x;
        target.y = source.y;
        target.z = source.z;
        const cv::Vec3b color = colors[static_cast<std::size_t>(i)];
        target.b = color[0];
        target.g = color[1];
        target.r = color[2];
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;

    FusionResult result;
    result.cloud = std::move(cloud);
    result.input_points = static_cast<std::size_t>(count);
    result.projected_points = validCount;
    result.colored_points = validCount;
    return result;
}

}  // namespace lvf
