#include "degradation/LidarDegradationCheck.h"

#include "TestHarness.h"

#include <opencv2/core.hpp>

#include <cmath>
#include <memory>
#include <string>

namespace {

lvf::Calibration makeCalibration() {
    lvf::Calibration calibration;
    calibration.P2 << 50.0f, 0.0f, 50.0f, 0.0f,
                      0.0f, 50.0f, 50.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f;
    calibration.R0_rect = Eigen::Matrix4f::Identity();
    calibration.T_velo_cam = Eigen::Matrix4f::Identity();
    return calibration;
}

lvf::Frame makeFrame() {
    lvf::Frame frame;
    frame.id = "lidar_degradation";
    frame.image = cv::Mat(100, 100, CV_8UC3, cv::Scalar(80, 80, 80));
    frame.calib = makeCalibration();
    frame.cloud = std::make_shared<lvf::CloudXYZI>();
    return frame;
}

void addRing(lvf::Frame& frame, std::size_t count, double maxAngle) {
    for (std::size_t i = 0; i < count; ++i) {
        const double t = count == 0 ? 0.0 : static_cast<double>(i) / static_cast<double>(count);
        const double angle = maxAngle * t;
        const double range = 5.0 + 20.0 * static_cast<double>(i % 1000) / 1000.0;
        pcl::PointXYZI point;
        point.x = static_cast<float>(range * std::cos(angle));
        point.y = static_cast<float>(range * std::sin(angle));
        point.z = 10.0f;
        point.intensity = 0.5f;
        frame.cloud->push_back(point);
    }
    frame.cloud->width = static_cast<std::uint32_t>(frame.cloud->size());
    frame.cloud->height = 1;
}

bool hasReason(const lvf::ModalityStatus& status, const std::string& code) {
    for (const auto& reason : status.reasons) {
        if (reason == code) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    using namespace lvf::test;

    lvf::LidarDegradationCheck checker;
    const double pi = std::acos(-1.0);

    {
        lvf::Frame frame = makeFrame();
        addRing(frame, 20000, 2.0 * pi);
        const auto result = checker.evaluate(frame);
        check(!result.status.degraded, "dense cloud is not degraded");
        check(result.metrics.near_points == 20000, "dense near points");
        check(result.metrics.azimuth_coverage > 0.95, "dense azimuth coverage");
        check(result.metrics.fov_points > 500, "dense FOV points");
        check(result.status.confidence > 0.9f, "dense confidence");
        check(!hasReason(result.status, "EMPTY_CLOUD"), "dense not empty");
    }

    {
        lvf::Frame frame = makeFrame();
        addRing(frame, 100, 2.0 * pi);
        const auto result = checker.evaluate(frame);
        check(result.status.degraded, "sparse cloud is degraded");
        check(hasReason(result.status, "SPARSE_POINTS"), "sparse reason");
        check(result.metrics.near_points == 100, "sparse near points");
    }

    {
        lvf::Frame frame = makeFrame();
        addRing(frame, 1000, 0.05);
        const auto result = checker.evaluate(frame);
        check(result.status.degraded, "low-coverage cloud is degraded");
        check(hasReason(result.status, "LOW_COVERAGE"), "low coverage reason");
        check(result.metrics.azimuth_coverage < 0.1, "low coverage metric");
    }

    {
        lvf::Frame frame = makeFrame();
        const auto result = checker.evaluate(frame);
        check(result.status.degraded, "empty cloud is degraded");
        check(hasReason(result.status, "EMPTY_CLOUD"), "empty cloud reason");
        checkNear(result.status.confidence, 0.0f, 1e-6f, "empty cloud confidence");
    }

    return finish("lidar_degradation");
}
