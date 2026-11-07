#pragma once

#include "core/StageTimings.h"
#include "core/Types.h"

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace lvf {

struct FrameRunResult {
    std::string frame_id;
    DegradationReport degradation;
    StageTimings timing;

    std::size_t input_points = 0;
    std::size_t projected_points = 0;
    std::size_t colored_points = 0;
    std::size_t occupied_cells = 0;
    std::size_t free_cells = 0;
    std::size_t unknown_cells = 0;

    std::vector<std::filesystem::path> outputs;
};

/// End-to-end per-frame orchestration: degradation -> fusion -> mapping -> writers.
class Pipeline {
public:
    explicit Pipeline(PipelineConfig config);

    FrameRunResult run(const Frame& frame, double readMs = 0.0) const;

private:
    PipelineConfig config_;
};

}  // namespace lvf
