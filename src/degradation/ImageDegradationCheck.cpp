#include "degradation/ImageDegradationCheck.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace lvf {
namespace {

double clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

cv::Mat toGray(const cv::Mat& image) {
    if (image.empty()) {
        throw std::invalid_argument("ImageDegradationCheck: empty image");
    }
    if (image.depth() != CV_8U) {
        throw std::invalid_argument("ImageDegradationCheck: expected 8-bit image");
    }

    if (image.channels() == 1) {
        return image;
    }
    if (image.channels() == 3) {
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }
    if (image.channels() == 4) {
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        return gray;
    }
    throw std::invalid_argument("ImageDegradationCheck: unsupported channel count");
}

}  // namespace

ImageDegradationCheck::ImageDegradationCheck(ImageDegradationOptions options)
    : options_(options) {}

ImageDegradationResult ImageDegradationCheck::evaluate(const cv::Mat& image) const {
    const cv::Mat gray = toGray(image);
    const std::size_t pixelCount =
        static_cast<std::size_t>(gray.cols) * static_cast<std::size_t>(gray.rows);
    if (pixelCount == 0) {
        throw std::invalid_argument("ImageDegradationCheck: zero-sized image");
    }

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(gray, mean, stddev);

    std::size_t histogram[256] = {};
    std::size_t darkPixels = 0;
    std::size_t brightPixels = 0;
    for (int y = 0; y < gray.rows; ++y) {
        const auto* row = gray.ptr<unsigned char>(y);
        for (int x = 0; x < gray.cols; ++x) {
            const unsigned char value = row[x];
            ++histogram[value];
            if (static_cast<double>(value) <= options_.dark_pixel_threshold) {
                ++darkPixels;
            }
            if (static_cast<double>(value) >= options_.bright_pixel_threshold) {
                ++brightPixels;
            }
        }
    }

    double entropyBits = 0.0;
    for (const auto count : histogram) {
        if (count == 0) {
            continue;
        }
        const double probability =
            static_cast<double>(count) / static_cast<double>(pixelCount);
        entropyBits -= probability * std::log2(probability);
    }

    ImageDegradationMetrics metrics;
    metrics.mean = mean[0];
    metrics.stddev = stddev[0];
    metrics.entropy = clamp01(entropyBits / 8.0);
    metrics.dark_ratio = static_cast<double>(darkPixels) / static_cast<double>(pixelCount);
    metrics.bright_ratio = static_cast<double>(brightPixels) / static_cast<double>(pixelCount);
    metrics.contrast_score = clamp01(metrics.stddev / 64.0);
    metrics.exposure_score =
        clamp01(1.0 - std::max(metrics.dark_ratio, metrics.bright_ratio));

    ModalityStatus status;
    status.confidence = static_cast<float>(clamp01(
        0.5 * metrics.contrast_score + 0.3 * metrics.entropy + 0.2 * metrics.exposure_score));

    if (metrics.stddev < options_.low_contrast_stddev || metrics.contrast_score < 0.30) {
        status.reasons.push_back("LOW_CONTRAST");
    }
    if (metrics.dark_ratio > options_.dark_ratio_threshold) {
        status.reasons.push_back("DARK_IMAGE");
    }
    if (metrics.bright_ratio > options_.bright_ratio_threshold) {
        status.reasons.push_back("OVEREXPOSED");
    }
    if (metrics.entropy < options_.low_entropy) {
        status.reasons.push_back("LOW_ENTROPY");
    }
    status.degraded =
        status.confidence < static_cast<float>(options_.confidence_threshold) ||
        !status.reasons.empty();

    status.metrics = {
        {"mean", metrics.mean},
        {"stddev", metrics.stddev},
        {"entropy", metrics.entropy},
        {"dark_ratio", metrics.dark_ratio},
        {"bright_ratio", metrics.bright_ratio},
        {"contrast_score", metrics.contrast_score},
        {"exposure_score", metrics.exposure_score},
    };

    return ImageDegradationResult{std::move(status), metrics};
}

}  // namespace lvf
