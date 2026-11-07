#pragma once

#include "core/Types.h"

namespace lvf {

struct MappingOptions {
    float x_min = -10.0f;
    float x_max = 50.0f;
    float y_min = -20.0f;
    float y_max = 20.0f;
    float z_min = -1.5f;
    float z_max = 2.5f;
    float voxel_size = 0.2f;
};

struct PreprocessStats {
    std::size_t input_points = 0;
    std::size_t roi_points = 0;
    std::size_t height_points = 0;
    std::size_t downsampled_points = 0;
};

struct PreprocessResult {
    CloudXYZI::Ptr cloud;
    PreprocessStats stats;
};

/// Crop a Velodyne cloud to the mapping ROI, apply a height band, then run
/// PCL VoxelGrid downsampling. The output stays in the Velodyne frame.
class PointCloudPreprocessor {
public:
    explicit PointCloudPreprocessor(MappingOptions options = {});

    PreprocessResult process(const CloudXYZI::Ptr& cloud) const;

private:
    MappingOptions options_;
};

}  // namespace lvf
