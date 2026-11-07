#pragma once

#include "core/StageTimings.h"

#include <filesystem>
#include <string>

namespace lvf {

void writeTimingJson(const std::filesystem::path& path,
                     const StageTimings& timings,
                     const std::string& frameId);

}  // namespace lvf
