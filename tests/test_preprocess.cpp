#include "mapping/PointCloudPreprocessor.h"

#include "TestHarness.h"

#include <memory>

int main() {
    using namespace lvf::test;

    auto cloud = std::make_shared<lvf::CloudXYZI>();
    auto add = [&](float x, float y, float z) {
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = 0.5f;
        cloud->push_back(point);
    };
    add(0.0f, 0.0f, 0.0f);
    add(0.1f, 0.1f, 0.1f);
    add(10.0f, 0.0f, 0.0f);   // outside x
    add(0.0f, 10.0f, 0.0f);   // outside y
    add(0.0f, 0.0f, 2.0f);    // above height
    add(0.0f, 0.0f, -2.0f);   // below height
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;

    lvf::MappingOptions options;
    options.x_min = -5.0f;
    options.x_max = 5.0f;
    options.y_min = -5.0f;
    options.y_max = 5.0f;
    options.z_min = -1.0f;
    options.z_max = 1.0f;
    options.voxel_size = 1.0f;

    lvf::PointCloudPreprocessor preprocessor(options);
    const auto result = preprocessor.process(cloud);
    check(result.stats.input_points == 6, "input point count");
    check(result.stats.roi_points == 4, "ROI point count");
    check(result.stats.height_points == 2, "height point count");
    check(result.stats.downsampled_points == 1, "voxel downsampled count");
    check(result.cloud != nullptr && result.cloud->size() == 1, "output cloud size");
    if (result.cloud && !result.cloud->empty()) {
        checkNear(result.cloud->points.front().x, 0.05f, 1e-4f, "voxel centroid x");
        checkNear(result.cloud->points.front().y, 0.05f, 1e-4f, "voxel centroid y");
        checkNear(result.cloud->points.front().z, 0.05f, 1e-4f, "voxel centroid z");
    }

    auto empty = std::make_shared<lvf::CloudXYZI>();
    empty->width = 0;
    empty->height = 1;
    const auto emptyResult = preprocessor.process(empty);
    check(emptyResult.stats.input_points == 0, "empty input count");
    check(emptyResult.cloud && emptyResult.cloud->empty(), "empty output cloud");

    checkThrows([&] { (void)preprocessor.process(nullptr); }, "null cloud throws");

    lvf::MappingOptions invalid = options;
    invalid.voxel_size = 0.0f;
    checkThrows([&] { lvf::PointCloudPreprocessor bad(invalid); }, "invalid voxel size throws");

    return finish("preprocess");
}
