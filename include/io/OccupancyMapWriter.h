#pragma once

#include "core/Types.h"

#include <filesystem>
#include <string>

namespace lvf {

struct OccupancyMapMetadata {
    std::string frameId;
    float heightMin = -1.5f;
    float heightMax = 2.5f;
    bool raycast = true;
    int inflationCells = 0;
};

/// Write `<stem>_occupancy.pgm` and `<stem>_occupancy.yaml` into `outputDir`.
/// The YAML is compatible with ROS map_server's image/origin/resolution keys.
void writeOccupancyMap(const std::filesystem::path& outputDir,
                       const std::string& stem,
                       const GridMap& grid,
                       const OccupancyMapMetadata& metadata);

}  // namespace lvf
