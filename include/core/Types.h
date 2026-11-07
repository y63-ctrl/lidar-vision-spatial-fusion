#pragma once

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace lvf {

using CloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using CloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;

/// KITTI calibration for camera 2 (left color camera).
///
/// Coordinate conventions:
///   - Velodyne: x forward, y left, z up
///   - Camera:   x right,  y down, z forward
///
/// The projection chain is:
///   T_img = P2 * R0_rect * T_velo_cam
/// where R0_rect and T_velo_cam are stored as 4x4 homogeneous matrices.
struct Calibration {
    Eigen::Matrix<float, 3, 4> P2 = Eigen::Matrix<float, 3, 4>::Zero();
    Eigen::Matrix4f R0_rect = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_velo_cam = Eigen::Matrix4f::Identity();

    /// Full Velodyne -> image projection matrix (3x4).
    Eigen::Matrix<float, 3, 4> veloToImage() const;

    /// Transform a Velodyne point into camera coordinates.
    Eigen::Vector3f veloToCamera(const Eigen::Vector3f& veloPoint) const;

    /// Basic finite/shape checks; parsing performs stricter validation.
    bool isValid() const;
};

/// One synchronized frame: RGB image, LiDAR cloud and calibration.
///
/// Ownership: `image` is a cv::Mat header (ref-counted pixels) and `cloud` is a
/// shared point-cloud pointer. Copying a Frame does not deep-copy pixels/points.
struct Frame {
    std::string id;
    cv::Mat image;              // BGR8
    CloudXYZI::Ptr cloud;       // Velodyne coordinates
    Calibration calib;
};

/// Output of the early-fusion stage (implemented in M2).
struct FusionResult {
    CloudXYZRGB::Ptr cloud;     // still in Velodyne coordinates, with RGB
    cv::Mat overlay;            // original image + projected points
    std::size_t input_points = 0;
    std::size_t projected_points = 0;
    std::size_t colored_points = 0;
};

struct ModalityStatus {
    float confidence = 0.0f;
    bool degraded = false;
    std::vector<std::string> reasons;              // stable codes, e.g. LOW_CONTRAST
    std::map<std::string, double> metrics;         // JSON-friendly metric values
};

struct DegradationReport {
    ModalityStatus image;
    ModalityStatus lidar;
    float overall = 0.0f;
    std::string action;  // FULL_FUSION / GEOMETRY_ONLY / IMAGE_ONLY / BOTH_DEGRADED
    bool map_trusted = true;
};

/// 2D BEV occupancy grid. `cells` uses -1 unknown, 0 free, 100 occupied.
struct GridMap {
    int width = 0;
    int height = 0;
    float resolution = 0.2f;
    Eigen::Vector2f origin{0.0f, 0.0f};
    std::vector<std::int8_t> cells;
};

/// Runtime configuration shared by the CLI, pipeline and writers.
struct PipelineConfig {
    std::filesystem::path data_root{"data/sample"};
    std::filesystem::path output_dir{"results"};
    std::filesystem::path calib_override;
    std::string frame_id{"000000"};

    float min_depth = 0.5f;
    float max_depth = 80.0f;
    float resolution = 0.2f;
    float voxel_size = 0.2f;

    float x_min = -10.0f;
    float x_max = 50.0f;
    float y_min = -20.0f;
    float y_max = 20.0f;
    float z_min = -1.5f;
    float z_max = 2.5f;

    int threads = 0;
    int inflation_cells = 0;
    int preview_scale = 2;
    bool make_visualization = true;
    bool draw_grid = true;
    bool raycast = true;
    bool write_pcd = true;
    bool fail_on_degraded = false;
    std::string fusion_mode = "auto";  // auto | pinhole | depth
};

}  // namespace lvf
