#include "mapping/PointCloudPreprocessor.h"

#include <pcl/filters/voxel_grid.h>

#include <cstdint>
#include <stdexcept>
#include <utility>

namespace lvf {
namespace {

void validateOptions(const MappingOptions& options) {
    if (!(options.x_min < options.x_max) || !(options.y_min < options.y_max) ||
        !(options.z_min < options.z_max)) {
        throw std::invalid_argument("PointCloudPreprocessor: invalid ROI bounds");
    }
    if (options.voxel_size <= 0.0f) {
        throw std::invalid_argument("PointCloudPreprocessor: voxel_size must be positive");
    }
}

}  // namespace

PointCloudPreprocessor::PointCloudPreprocessor(MappingOptions options) : options_(options) {
    validateOptions(options_);
}

PreprocessResult PointCloudPreprocessor::process(const CloudXYZI::Ptr& cloud) const {
    if (cloud == nullptr) {
        throw std::invalid_argument("PointCloudPreprocessor: null cloud");
    }

    PreprocessResult result;
    result.stats.input_points = cloud->size();

    auto filtered = std::make_shared<CloudXYZI>();
    filtered->reserve(cloud->size());
    for (const auto& point : cloud->points) {
        if (point.x < options_.x_min || point.x > options_.x_max ||
            point.y < options_.y_min || point.y > options_.y_max) {
            continue;
        }
        ++result.stats.roi_points;

        if (point.z < options_.z_min || point.z > options_.z_max) {
            continue;
        }
        ++result.stats.height_points;
        filtered->push_back(point);
    }

    if (filtered->empty()) {
        filtered->width = 0;
        filtered->height = 1;
        filtered->is_dense = true;
        result.cloud = std::move(filtered);
        return result;
    }

    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = true;

    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(filtered);
    voxel.setLeafSize(options_.voxel_size, options_.voxel_size, options_.voxel_size);

    auto downsampled = std::make_shared<CloudXYZI>();
    voxel.filter(*downsampled);
    downsampled->width = static_cast<std::uint32_t>(downsampled->size());
    downsampled->height = 1;
    downsampled->is_dense = true;

    result.stats.downsampled_points = downsampled->size();
    result.cloud = std::move(downsampled);
    return result;
}

}  // namespace lvf
