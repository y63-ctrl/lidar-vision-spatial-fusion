#include "fusion/PinholeFusionStrategy.h"
#include "viz/OverlayRenderer.h"

#include "TestHarness.h"

#include <opencv2/core.hpp>

#include <memory>

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
    frame.id = "overlay_test";
    frame.image = cv::Mat(9, 9, CV_8UC3, cv::Scalar(10, 20, 30));
    frame.image.at<cv::Vec3b>(4, 4) = cv::Vec3b(200, 100, 50);
    frame.calib = makeCalibration();

    frame.cloud = std::make_shared<lvf::CloudXYZI>();
    pcl::PointXYZI point;
    point.x = 0.0f;
    point.y = 0.0f;
    point.z = 10.0f;
    point.intensity = 0.0f;
    frame.cloud->push_back(point);
    frame.cloud->width = 1;
    frame.cloud->height = 1;
    return frame;
}

bool sameColor(const cv::Vec3b& a, const cv::Vec3b& b) {
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

int countNonBackground(const cv::Mat& image, const cv::Vec3b& background) {
    int count = 0;
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            if (!sameColor(image.at<cv::Vec3b>(y, x), background)) {
                ++count;
            }
        }
    }
    return count;
}

}  // namespace

int main() {
    using namespace lvf::test;

    const lvf::Frame frame = makeFrame();
    lvf::FusionOptions fusionOptions;
    fusionOptions.threads = 2;
    lvf::PinholeFusionStrategy fusion(fusionOptions);
    const lvf::FusionResult result = fusion.fuse(frame);

    lvf::OverlayOptions options;
    options.point_size = 2;
    options.draw_text = false;

    const cv::Mat overlay = lvf::OverlayRenderer::renderOverlay(frame, result, options);
    check(overlay.cols == frame.image.cols, "overlay width");
    check(overlay.rows == frame.image.rows, "overlay height");
    check(overlay.type() == CV_8UC3, "overlay type");
    check(countNonBackground(overlay, cv::Vec3b(10, 20, 30)) > 0, "overlay draws points");
    check(sameColor(overlay.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30)),
          "overlay preserves background");

    const cv::Mat fusedView = lvf::OverlayRenderer::renderFusedView(frame, result, options);
    check(fusedView.cols == frame.image.cols, "fused view width");
    check(fusedView.rows == frame.image.rows, "fused view height");
    check(fusedView.type() == CV_8UC3, "fused view type");
    check(countNonBackground(fusedView, cv::Vec3b(0, 0, 0)) > 0, "fused view draws points");

    return finish("overlay");
}
