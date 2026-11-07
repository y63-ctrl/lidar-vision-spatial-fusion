#pragma once

#include <string>

namespace lvf {

/// Semantic project version (kept in sync with CMake project()).
std::string version();

/// Human-readable dependency versions, e.g. "OpenCV 5.1.0 | Eigen 3.4.0 | PCL 1.15.1".
std::string dependencySummary();

/// True when the build has OpenMP enabled.
bool hasOpenMp();

}  // namespace lvf
