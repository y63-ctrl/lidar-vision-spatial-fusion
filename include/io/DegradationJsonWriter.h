#pragma once

#include "core/Types.h"

#include <filesystem>
#include <string>

namespace lvf {

void writeDegradationJson(const std::filesystem::path& path,
                          const DegradationReport& report,
                          const std::string& frameId);

}  // namespace lvf
