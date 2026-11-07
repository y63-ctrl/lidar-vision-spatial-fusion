#pragma once

namespace lvf {

struct StageTimings {
    double read_ms = 0.0;
    double degradation_ms = 0.0;
    double fusion_ms = 0.0;
    double mapping_ms = 0.0;
    double write_ms = 0.0;
    double compute_ms = 0.0;
    double total_ms = 0.0;
};

}  // namespace lvf
