#pragma once

#include "core/Types.h"

#include <cstddef>

namespace lvf {

struct BevOptions {
    float x_min = -10.0f;
    float x_max = 50.0f;
    float y_min = -20.0f;
    float y_max = 20.0f;
    float resolution = 0.2f;
    bool raycast = true;
    int inflation_cells = 0;
};

struct BevStats {
    std::size_t occupied = 0;
    std::size_t free = 0;
    std::size_t unknown = 0;
    std::size_t total = 0;
};

struct BevResult {
    GridMap grid;
    BevStats stats;
};

/// Build a 2D BEV occupancy grid from a preprocessed Velodyne cloud.
///
/// Row 0 corresponds to y_max; origin is the lower-left corner [x_min, y_min].
/// Values follow the ROS/map_server convention: 100 occupied, 0 free,
/// -1 unknown.
class BevOccupancyGrid {
public:
    explicit BevOccupancyGrid(BevOptions options = {});

    BevResult build(const CloudXYZI& cloud) const;

private:
    BevOptions options_;
};

}  // namespace lvf
