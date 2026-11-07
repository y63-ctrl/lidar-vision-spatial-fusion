#include "degradation/ImageDegradationCheck.h"

#include "TestHarness.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <string>

namespace {

bool hasReason(const lvf::ModalityStatus& status, const std::string& code) {
    return std::find(status.reasons.begin(), status.reasons.end(), code) != status.reasons.end();
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

cv::Mat makeLowContrastImage() {
    cv::Mat image(96, 128, CV_8UC3, cv::Scalar(120, 120, 120));
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            if ((x + y) % 2 == 0) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(121, 121, 121);
            }
        }
    }
    return image;
}

}  // namespace

int main() {
    using namespace lvf::test;

    lvf::ImageDegradationCheck checker;

    const auto normal = checker.evaluate(makeNormalImage());
    check(!normal.status.degraded, "normal image is not degraded");
    check(normal.status.confidence > 0.35f, "normal image confidence");
    check(normal.status.metrics.count("stddev") == 1, "normal image has stddev metric");

    const auto dark = checker.evaluate(cv::Mat(96, 128, CV_8UC3, cv::Scalar(0, 0, 0)));
    check(dark.status.degraded, "dark image is degraded");
    check(hasReason(dark.status, "DARK_IMAGE"), "dark image reason");
    check(hasReason(dark.status, "LOW_CONTRAST"), "dark image low contrast");
    check(dark.metrics.dark_ratio > 0.99, "dark image ratio");

    const auto bright = checker.evaluate(cv::Mat(96, 128, CV_8UC3, cv::Scalar(255, 255, 255)));
    check(bright.status.degraded, "bright image is degraded");
    check(hasReason(bright.status, "OVEREXPOSED"), "bright image reason");
    check(bright.metrics.bright_ratio > 0.99, "bright image ratio");

    const auto lowContrast = checker.evaluate(makeLowContrastImage());
    check(lowContrast.status.degraded, "low-contrast image is degraded");
    check(hasReason(lowContrast.status, "LOW_CONTRAST"), "low-contrast reason");

    checkThrows([&] { (void)checker.evaluate(cv::Mat()); }, "empty image throws");

    return finish("image_degradation");
}
