#include "io/PcdWriter.h"

#include <pcl/io/pcd_io.h>

#include <chrono>
#include <filesystem>
#include <stdexcept>

namespace lvf {

void writeColoredCloud(const std::filesystem::path& path, const CloudXYZRGB& cloud) {
    if (path.empty()) {
        throw std::invalid_argument("writeColoredCloud: empty path");
    }
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto temporary =
        std::filesystem::temp_directory_path() / ("lvf_" + std::to_string(stamp) + ".pcd");

    const int result = pcl::io::savePCDFileBinary(temporary.string(), cloud);
    if (result != 0) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        throw std::runtime_error("writeColoredCloud: PCL failed to write " + path.string());
    }

    std::error_code copyError;
    std::filesystem::copy_file(temporary, path,
                               std::filesystem::copy_options::overwrite_existing, copyError);
    std::error_code removeError;
    std::filesystem::remove(temporary, removeError);
    if (copyError) {
        throw std::runtime_error("writeColoredCloud: failed to move PCD to " + path.string() +
                                 ": " + copyError.message());
    }
}

}  // namespace lvf
