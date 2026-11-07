#pragma once

#include "core/Types.h"

#include <opencv2/core.hpp>

namespace lvf {

struct OccupancyRenderOptions {
    int scale = 1;
    bool draw_grid = true;
};

/// White = free, black = occupied, gray = unknown.
cv::Mat renderOccupancyMap(const GridMap& grid,
                           const OccupancyRenderOptions& options = {});

}  // namespace lvf
