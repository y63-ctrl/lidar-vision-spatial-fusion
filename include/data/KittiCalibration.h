#pragma once

#include "core/Types.h"

#include <filesystem>
#include <iosfwd>
#include <string>

namespace lvf {

/// Parser for KITTI object-benchmark calibration files.
///
/// Required keys:
///   P2:          3x4 projection matrix (camera 2, left color)
///   R0_rect:     3x3 rectification matrix
///   Tr_velo_to_cam: 3x4 Velodyne -> camera transform
///
/// P0/P1/P3/Tr_imu_to_velo are ignored. Values may continue on subsequent
/// lines; parsing stops at the next key.
class KittiCalibration {
public:
    static Calibration loadFromFile(const std::filesystem::path& path);

    /// Parse from an arbitrary stream. `sourceName` is used only in errors.
    static Calibration parse(std::istream& input, const std::string& sourceName = "<stream>");
};

}  // namespace lvf
