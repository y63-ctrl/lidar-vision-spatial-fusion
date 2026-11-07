#include "degradation/DegradationEvaluator.h"
#include "io/DegradationJsonWriter.h"

#include "TestHarness.h"

#include <opencv2/core.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
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

cv::Mat makeNormalImage() {
    cv::Mat image(96, 128, CV_8UC3);
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<unsigned char>((x * 3 + y) % 256),
                static_cast<unsigned char>((x + y * 5) % 256),
                static_cast<unsigned char>((x * 7 + y * 2) % 256));
        }
    }
    return image;
}

void addRing(lvf::Frame& frame, std::size_t count) {
    const double pi = std::acos(-1.0);
    for (std::size_t i = 0; i < count; ++i) {
        const double angle = 2.0 * pi * static_cast<double>(i) / static_cast<double>(count);
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

lvf::Frame makeFrame(bool darkImage, std::size_t pointCount) {
    lvf::Frame frame;
    frame.id = "000000";
    frame.image = darkImage
                      ? cv::Mat(96, 128, CV_8UC3, cv::Scalar(0, 0, 0))
                      : makeNormalImage();
    frame.calib = makeCalibration();
    frame.cloud = std::make_shared<lvf::CloudXYZI>();
    addRing(frame, pointCount);
    return frame;
}

std::string readFile(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

}  // namespace

int main() {
    using namespace lvf::test;

    lvf::DegradationEvaluator evaluator;

    const auto normal = evaluator.evaluate(makeFrame(false, 20000));
    check(normal.action == "FULL_FUSION", "normal action");
    check(normal.map_trusted, "normal map trusted");
    check(normal.image.metrics.count("stddev") == 1, "image metrics present");
    check(normal.lidar.metrics.count("near_points") == 1, "lidar metrics present");

    const auto geometryOnly = evaluator.evaluate(makeFrame(true, 20000));
    check(geometryOnly.action == "GEOMETRY_ONLY", "dark image action");
    check(geometryOnly.image.degraded, "dark image degraded");
    check(!geometryOnly.lidar.degraded, "dense lidar not degraded");
    check(geometryOnly.map_trusted, "geometry-only map trusted");

    const auto imageOnly = evaluator.evaluate(makeFrame(false, 100));
    check(imageOnly.action == "IMAGE_ONLY", "sparse lidar action");
    check(!imageOnly.image.degraded, "normal image not degraded");
    check(imageOnly.lidar.degraded, "sparse lidar degraded");
    check(!imageOnly.map_trusted, "image-only map untrusted");

    const auto both = evaluator.evaluate(makeFrame(true, 100));
    check(both.action == "BOTH_DEGRADED", "both degraded action");
    check(!both.map_trusted, "both degraded map untrusted");

    const auto path = std::filesystem::temp_directory_path() / "lvf_degradation_test.json";
    lvf::writeDegradationJson(path, normal, "000000");
    const std::string json = readFile(path);
    check(!json.empty(), "degradation JSON is non-empty");
    check(json.find("\"frame\": \"000000\"") != std::string::npos, "JSON frame");
    check(json.find("\"action\": \"FULL_FUSION\"") != std::string::npos, "JSON action");
    check(json.find("\"map_trusted\": true") != std::string::npos, "JSON map_trusted");
    check(json.find("\"image\"") != std::string::npos, "JSON image");
    check(json.find("\"lidar\"") != std::string::npos, "JSON lidar");
    check(json.find("\"metrics\"") != std::string::npos, "JSON metrics");
    check(json.find("\"reasons\"") != std::string::npos, "JSON reasons");

    lvf::writeDegradationJson(path, geometryOnly, "000000");
    const std::string degradedJson = readFile(path);
    check(degradedJson.find("\"action\": \"GEOMETRY_ONLY\"") != std::string::npos,
          "degraded JSON action");
    check(degradedJson.find("DARK_IMAGE") != std::string::npos, "degraded JSON reason");

    std::error_code ignored;
    std::filesystem::remove(path, ignored);

    return finish("degradation_report");
}
