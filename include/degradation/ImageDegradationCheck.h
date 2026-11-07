#pragma once

#include "core/Types.h"

#include <opencv2/core.hpp>

namespace lvf {

struct ImageDegradationOptions {
    double dark_pixel_threshold = 25.0;    // grayscale <= this is "dark"
    double bright_pixel_threshold = 230.0; // grayscale >= this is "bright"
    double dark_ratio_threshold = 0.35;
    double bright_ratio_threshold = 0.20;
    double low_contrast_stddev = 20.0;
    double low_entropy = 0.20;
    double confidence_threshold = 0.35;
};

struct ImageDegradationMetrics {
    double mean = 0.0;
    double stddev = 0.0;
    double entropy = 0.0;  // normalized to [0, 1] (8-bit max entropy = 8)
    double dark_ratio = 0.0;
    double bright_ratio = 0.0;
    double contrast_score = 0.0;
    double exposure_score = 0.0;
};

struct ImageDegradationResult {
    ModalityStatus status;
    ImageDegradationMetrics metrics;
};

class ImageDegradationCheck {
public:
    explicit ImageDegradationCheck(ImageDegradationOptions options = {});

    ImageDegradationResult evaluate(const cv::Mat& image) const;

private:
    ImageDegradationOptions options_;
};

}  // namespace lvf
