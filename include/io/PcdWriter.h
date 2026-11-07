#pragma once

#include "core/Types.h"

#include <filesystem>

namespace lvf {

/// Write a binary PCD. Uses a temporary ASCII path because PCL's path handling
/// on Windows is not reliable for non-ASCII project directories.
void writeColoredCloud(const std::filesystem::path& path, const CloudXYZRGB& cloud);

}  // namespace lvf
