#pragma once

namespace lvf {

struct FusionOptions {
    float min_depth = 0.5f;
    float max_depth = 80.0f;
    int threads = 0;        // 0 = OpenMP default
    bool bilinear = true;   // false = nearest-neighbor sampling
};

}  // namespace lvf
