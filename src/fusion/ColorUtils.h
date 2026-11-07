#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace lvf::detail {

inline cv::Vec3b sampleBilinear(const cv::Mat& image, float u, float v) {
    if (image.type() != CV_8UC3) {
        return cv::Vec3b(0, 0, 0);
    }

    const int x0 = std::clamp(static_cast<int>(std::floor(u)), 0, image.cols - 1);
    const int y0 = std::clamp(static_cast<int>(std::floor(v)), 0, image.rows - 1);
    const int x1 = std::min(x0 + 1, image.cols - 1);
    const int y1 = std::min(y0 + 1, image.rows - 1);

    const float fx = std::clamp(u - static_cast<float>(x0), 0.0f, 1.0f);
    const float fy = std::clamp(v - static_cast<float>(y0), 0.0f, 1.0f);

    const cv::Vec3b c00 = image.at<cv::Vec3b>(y0, x0);
    const cv::Vec3b c10 = image.at<cv::Vec3b>(y0, x1);
    const cv::Vec3b c01 = image.at<cv::Vec3b>(y1, x0);
    const cv::Vec3b c11 = image.at<cv::Vec3b>(y1, x1);

    cv::Vec3b result;
    for (int channel = 0; channel < 3; ++channel) {
        const float top = static_cast<float>(c00[channel]) * (1.0f - fx) +
                          static_cast<float>(c10[channel]) * fx;
        const float bottom = static_cast<float>(c01[channel]) * (1.0f - fx) +
                             static_cast<float>(c11[channel]) * fx;
        result[channel] = static_cast<unsigned char>(std::clamp(
            top * (1.0f - fy) + bottom * fy, 0.0f, 255.0f));
    }
    return result;
}

inline cv::Vec3b sampleNearest(const cv::Mat& image, float u, float v) {
    if (image.type() != CV_8UC3) {
        return cv::Vec3b(0, 0, 0);
    }
    const int x = std::clamp(static_cast<int>(std::lround(u)), 0, image.cols - 1);
    const int y = std::clamp(static_cast<int>(std::lround(v)), 0, image.rows - 1);
    return image.at<cv::Vec3b>(y, x);
}

/// Warm (near) -> cool (far) depth color. `t` is normalized distance [0,1].
inline cv::Vec3b depthColor(float depth, float minDepth, float maxDepth) {
    const float span = std::max(maxDepth - minDepth, 1e-3f);
    const float normalized = std::clamp((depth - minDepth) / span, 0.0f, 1.0f);
    const float t = 1.0f - normalized;  // near = 1 (red), far = 0 (blue)

    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    if (t < 0.25f) {
        b = 1.0f;
        g = t * 4.0f;
    } else if (t < 0.5f) {
        b = (0.5f - t) * 4.0f;
        g = 1.0f;
    } else if (t < 0.75f) {
        g = 1.0f;
        r = (t - 0.5f) * 4.0f;
    } else {
        r = 1.0f;
        g = (1.0f - t) * 4.0f;
    }

    return cv::Vec3b(static_cast<unsigned char>(b * 255.0f),
                     static_cast<unsigned char>(g * 255.0f),
                     static_cast<unsigned char>(r * 255.0f));
}

}  // namespace lvf::detail
