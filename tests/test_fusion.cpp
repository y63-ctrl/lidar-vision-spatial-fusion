#include "fusion/DepthOnlyFusionStrategy.h"
#include "fusion/FusionStrategyFactory.h"
#include "fusion/PinholeFusionStrategy.h"

#include "TestHarness.h"

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace {

lvf::Calibration makeCalibration() {
    lvf::Calibration calibration;
    calibration.P2 << 10.0f, 0.0f, 4.0f, 0.0f,
                      0.0f, 10.0f, 4.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f;
    calibration.R0_rect = Eigen::Matrix4f::Identity();
    calibration.T_velo_cam = Eigen::Matrix4f::Identity();
    return calibration;
}

lvf::Frame makeFrame() {
    lvf::Frame frame;
    frame.id = "fusion_test";
    frame.image = cv::Mat(9, 9, CV_8UC3);
    for (int y = 0; y < frame.image.rows; ++y) {
        for (int x = 0; x < frame.image.cols; ++x) {
            frame.image.at<cv::Vec3b>(y, x) =
                cv::Vec3b(static_cast<unsigned char>(x * 10),
                          static_cast<unsigned char>(y * 10),
                          static_cast<unsigned char>((x + y) * 5));
        }
    }
    frame.calib = makeCalibration();

    frame.cloud = std::make_shared<lvf::CloudXYZI>();
    auto addPoint = [&](float x, float y, float z) {
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = 0.0f;
        frame.cloud->push_back(point);
    };
    addPoint(1.0f, 0.0f, 10.0f);    // valid: u=5, v=4
    addPoint(100.0f, 0.0f, 10.0f);  // outside image
    addPoint(0.0f, 0.0f, -10.0f);   // behind camera
    frame.cloud->width = static_cast<std::uint32_t>(frame.cloud->size());
    frame.cloud->height = 1;
    frame.cloud->is_dense = true;
    return frame;
}

}  // namespace

int main() {
    using namespace lvf::test;

    const lvf::Frame frame = makeFrame();
    lvf::FusionOptions options;
    options.min_depth = 0.5f;
    options.max_depth = 80.0f;
    options.threads = 4;
    options.bilinear = false;

    {
        lvf::PinholeFusionStrategy strategy(options);
        const lvf::FusionResult result = strategy.fuse(frame);
        check(result.input_points == 3, "pinhole input points");
        check(result.projected_points == 1, "pinhole projected points");
        check(result.colored_points == 1, "pinhole colored points");
        check(result.cloud != nullptr && result.cloud->size() == 1, "pinhole output cloud size");
        if (result.cloud && !result.cloud->empty()) {
            const auto& point = result.cloud->points.front();
            checkNear(point.b, 50.0f, 1e-4f, "pinhole sampled B");
            checkNear(point.g, 40.0f, 1e-4f, "pinhole sampled G");
            checkNear(point.r, 45.0f, 1e-4f, "pinhole sampled R");
        }
    }

    {
        lvf::DepthOnlyFusionStrategy strategy(options);
        const lvf::FusionResult result = strategy.fuse(frame);
        check(result.cloud != nullptr && result.cloud->size() == 1, "depth output cloud size");
        if (result.cloud && !result.cloud->empty()) {
            const auto& point = result.cloud->points.front();
            check(point.r + point.g + point.b > 0, "depth pseudo-color is not black");
        }
    }

    {
        lvf::Frame gridFrame = makeFrame();
        gridFrame.cloud->clear();
        for (int x = -4; x <= 4; ++x) {
            for (int y = -4; y <= 4; ++y) {
                pcl::PointXYZI point;
                point.x = static_cast<float>(x);
                point.y = static_cast<float>(y);
                point.z = 10.0f;
                gridFrame.cloud->push_back(point);
            }
        }
        gridFrame.cloud->width = static_cast<std::uint32_t>(gridFrame.cloud->size());
        gridFrame.cloud->height = 1;

        lvf::FusionOptions oneThread = options;
        oneThread.threads = 1;
        oneThread.bilinear = true;
        lvf::FusionOptions fourThreads = options;
        fourThreads.threads = 4;
        fourThreads.bilinear = true;

        lvf::PinholeFusionStrategy single(oneThread);
        lvf::PinholeFusionStrategy multi(fourThreads);
        const auto a = single.fuse(gridFrame);
        const auto b = multi.fuse(gridFrame);
        check(a.cloud->size() == 81, "thread comparison cloud size");
        check(b.cloud->size() == 81, "thread comparison cloud size (multi)");
        if (a.cloud->size() == b.cloud->size()) {
            for (std::size_t i = 0; i < a.cloud->size(); ++i) {
                checkNear(a.cloud->points[i].r, b.cloud->points[i].r, 1e-4f,
                          "thread-independent R");
                checkNear(a.cloud->points[i].g, b.cloud->points[i].g, 1e-4f,
                          "thread-independent G");
                checkNear(a.cloud->points[i].b, b.cloud->points[i].b, 1e-4f,
                          "thread-independent B");
            }
        }
    }

    {
        lvf::Frame bilinearFrame;
        bilinearFrame.id = "bilinear";
        bilinearFrame.image = cv::Mat(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
        bilinearFrame.image.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0);
        bilinearFrame.image.at<cv::Vec3b>(0, 1) = cv::Vec3b(100, 0, 0);
        bilinearFrame.image.at<cv::Vec3b>(1, 0) = cv::Vec3b(0, 100, 0);
        bilinearFrame.image.at<cv::Vec3b>(1, 1) = cv::Vec3b(0, 0, 100);

        lvf::Calibration bilinearCalibration;
        bilinearCalibration.P2 << 10.0f, 0.0f, 0.0f, 0.0f,
                                  0.0f, 10.0f, 0.0f, 0.0f,
                                  0.0f, 0.0f, 1.0f, 0.0f;
        bilinearCalibration.R0_rect = Eigen::Matrix4f::Identity();
        bilinearCalibration.T_velo_cam = Eigen::Matrix4f::Identity();
        bilinearFrame.calib = bilinearCalibration;

        bilinearFrame.cloud = std::make_shared<lvf::CloudXYZI>();
        pcl::PointXYZI point;
        point.x = 0.5f;
        point.y = 0.5f;
        point.z = 10.0f;
        point.intensity = 0.0f;
        bilinearFrame.cloud->push_back(point);
        bilinearFrame.cloud->width = 1;
        bilinearFrame.cloud->height = 1;

        lvf::FusionOptions bilinearOptions;
        bilinearOptions.bilinear = true;
        bilinearOptions.threads = 1;
        lvf::PinholeFusionStrategy bilinearStrategy(bilinearOptions);
        const auto bilinearResult = bilinearStrategy.fuse(bilinearFrame);
        check(bilinearResult.cloud && bilinearResult.cloud->size() == 1,
              "bilinear output cloud size");
        if (bilinearResult.cloud && !bilinearResult.cloud->empty()) {
            const auto& sampled = bilinearResult.cloud->points.front();
            checkNear(sampled.b, 25.0f, 1e-4f, "bilinear B");
            checkNear(sampled.g, 25.0f, 1e-4f, "bilinear G");
            checkNear(sampled.r, 25.0f, 1e-4f, "bilinear R");
        }
    }

    {
        auto pinhole = lvf::FusionStrategyFactory::create("pinhole", options);
        auto depth = lvf::FusionStrategyFactory::create("depth", options);
        check(dynamic_cast<lvf::PinholeFusionStrategy*>(pinhole.get()) != nullptr,
              "factory creates pinhole strategy");
        check(dynamic_cast<lvf::DepthOnlyFusionStrategy*>(depth.get()) != nullptr,
              "factory creates depth strategy");

        auto full = lvf::FusionStrategyFactory::createForAction("FULL_FUSION", options);
        auto geometry = lvf::FusionStrategyFactory::createForAction("GEOMETRY_ONLY", options);
        check(dynamic_cast<lvf::PinholeFusionStrategy*>(full.get()) != nullptr,
              "FULL_FUSION maps to pinhole");
        check(dynamic_cast<lvf::DepthOnlyFusionStrategy*>(geometry.get()) != nullptr,
              "GEOMETRY_ONLY maps to depth-only");
        checkThrows(
            [&] { (void)lvf::FusionStrategyFactory::create("unknown", options); },
            "unknown strategy throws");
    }

    return finish("fusion");
}
