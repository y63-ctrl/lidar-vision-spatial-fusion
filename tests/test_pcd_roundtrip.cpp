#include "io/PcdWriter.h"

#include "TestHarness.h"

#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <memory>

int main() {
    using namespace lvf::test;

    auto cloud = std::make_shared<lvf::CloudXYZRGB>();
    for (int i = 0; i < 3; ++i) {
        pcl::PointXYZRGB point;
        point.x = static_cast<float>(i);
        point.y = static_cast<float>(i * 2);
        point.z = static_cast<float>(i * 3);
        point.r = static_cast<std::uint8_t>(10 + i);
        point.g = static_cast<std::uint8_t>(20 + i);
        point.b = static_cast<std::uint8_t>(30 + i);
        cloud->push_back(point);
    }
    cloud->width = 3;
    cloud->height = 1;
    cloud->is_dense = true;

    const auto path = std::filesystem::temp_directory_path() / "lvf_test_fused.pcd";
    lvf::writeColoredCloud(path, *cloud);
    check(std::filesystem::exists(path), "PCD file exists");
    check(std::filesystem::file_size(path) > 0, "PCD file is non-empty");

    lvf::CloudXYZRGB loaded;
    check(pcl::io::loadPCDFile(path.string(), loaded) == 0, "PCD can be read back");
    check(loaded.size() == 3, "PCD point count");
    if (loaded.size() == 3) {
        check(loaded[1].r == 11 && loaded[1].g == 21 && loaded[1].b == 31,
              "PCD RGB round-trip");
    }

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    return finish("pcd_roundtrip");
}
