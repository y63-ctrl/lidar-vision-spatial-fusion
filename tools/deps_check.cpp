#include "core/Version.h"

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void checkOpenCv() {
    cv::Mat image(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
    require(!image.empty(), "OpenCV: failed to allocate cv::Mat");
    require(image.total() == 64, "OpenCV: unexpected Mat size");

    std::vector<unsigned char> encoded;
    require(cv::imencode(".png", image, encoded), "OpenCV: PNG encoder unavailable");
    require(!encoded.empty(), "OpenCV: empty encoded image");

    std::cout << "[OK] OpenCV " << CV_VERSION << " (core, imgcodecs)\n";
}

void checkEigen() {
    Eigen::Matrix3f matrix;
    matrix << 2.0f, 0.0f, 0.0f,
              0.0f, 4.0f, 0.0f,
              0.0f, 0.0f, 8.0f;

    const Eigen::Vector3f input(1.0f, 2.0f, 3.0f);
    const Eigen::Vector3f result = matrix * input;
    require(result.isApprox(Eigen::Vector3f(2.0f, 8.0f, 24.0f)),
            "Eigen: matrix multiplication returned an unexpected result");

    std::cout << "[OK] Eigen " << EIGEN_WORLD_VERSION << '.' << EIGEN_MAJOR_VERSION << '.'
              << EIGEN_MINOR_VERSION << '\n';
}

void checkPcl() {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
    cloud->push_back(pcl::PointXYZ(0.05f, 0.05f, 0.05f));
    cloud->push_back(pcl::PointXYZ(3.0f, 4.0f, 5.0f));
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(1.0f, 1.0f, 1.0f);

    pcl::PointCloud<pcl::PointXYZ> filtered;
    voxel.filter(filtered);
    require(filtered.size() == 2, "PCL: VoxelGrid did not merge the two nearby points");

    const auto pcdPath = std::filesystem::temp_directory_path() / "lvf_deps_check.pcd";
    require(pcl::io::savePCDFileBinary(pcdPath.string(), filtered) == 0,
            "PCL: failed to write PCD");
    pcl::PointCloud<pcl::PointXYZ> loaded;
    require(pcl::io::loadPCDFile(pcdPath.string(), loaded) == 0,
            "PCL: failed to read PCD");
    require(loaded.size() == filtered.size(), "PCL: PCD round-trip changed the point count");
    std::error_code ignored;
    std::filesystem::remove(pcdPath, ignored);

    std::cout << "[OK] PCL " << PCL_MAJOR_VERSION << '.' << PCL_MINOR_VERSION << '.'
              << PCL_REVISION_VERSION << " (common, io, filters)\n";
}

void checkOpenMp() {
#ifdef _OPENMP
    std::cout << "[OK] OpenMP " << _OPENMP << " (max threads: " << omp_get_max_threads() << ")\n";
#else
    std::cout << "[WARN] OpenMP is not enabled\n";
#endif
}

}  // namespace

int main() {
    std::cout << "LiDAR-Vision Spatial Fusion Pipeline dependency check\n";
    std::cout << "Version: " << lvf::version() << '\n';
    std::cout << "Summary: " << lvf::dependencySummary() << "\n\n";

    try {
        checkOpenCv();
        checkEigen();
        checkPcl();
        checkOpenMp();
    } catch (const std::exception& error) {
        std::cerr << "[FAIL] " << error.what() << '\n';
        return 1;
    }

    std::cout << "\nAll required dependencies are available.\n";
    return 0;
}
