#pragma once

#include "core/Types.h"

#include <opencv2/core.hpp>

namespace lvf {

struct OverlayOptions {
    float min_depth = 0.5f;
    float max_depth = 80.0f;
    int point_size = 2;
    bool draw_text = true;
};

class OverlayRenderer {
public:
    static cv::Mat renderOverlay(const Frame& frame,
                                 const FusionResult& result,
                                 const OverlayOptions& options = {});

    static cv::Mat renderFusedView(const Frame& frame,
                                   const FusionResult& result,
                                   const OverlayOptions& options = {});
};

}  // namespace lvf
